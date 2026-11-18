#!/usr/bin/env python3
"""Differential-pair length / skew / coupling report for a KiCad 9 board (read-only).

Run with KiCad's bundled python:

    $KICAD_PY diffpair_skew.py --board X.kicad_pcb --out report.json [--md report.md]
        [--limits limits.json]          # {"pairs": [{"name":..,"p":[nets],"n":[nets],"max_skew_mm":..,"source":..}]}
        [--fine-step 0.05]

Pair discovery (each pair records how it was found):
  * name:   nets whose names differ only in one polarity token: `_P`/`_N` (anywhere, e.g.
            `X_P_RAW`/`X_N_RAW`), trailing `P`/`N` on the last token (KiCad's own rule, e.g. `INP`/`INN`),
            `DP`/`DM`/`DN` tokens (USB), trailing `+`/`-`, and trailing `H`/`L` on a token that
            contains `CAN` (CAN bus). Both members must exist on the board.
  * netclass: nets in any netclass whose name contains `DIFF` are paired by the same rules
            (the netclass is recorded).
  * dru:    `inDiffPair('<pattern>')` conditions in the project's .kicad_dru (recorded with the
            rule's constraints, e.g. `skew`, `diff_pair_gap`, `diff_pair_uncoupled`).
  * limits: groups from --limits (lets multi-net pairs such as `USB_DP_CONN`+`USB_DP` be summed
            and carries a repo-declared limit).

Per net: routed length = sum of track (and arc) lengths + via barrel length. Barrel length is
the stackup distance between the outermost copper layers on which this net's tracks or pads
actually connect to the via (copper-centre to copper-centre, from the board's own (stackup)).
Co-located vias (stacked microvia/buried chains) are treated as one stack. A stack touched on one
layer only contributes 0.

Delay: each track segment uses its own layer's effective permittivity. Outer layers:
quasi-static microstrip, Hammerstad: eps_eff = (er+1)/2 + (er-1)/2 * (1 + 12 h/w)^-0.5, h = the
dielectric to the adjacent copper layer, w = segment width. Solder mask, trace thickness,
dispersion and the odd/even-mode split of a coupled pair are ignored. Inner layers:
eps_eff = er (stripline). Via barrels: er. Delay = L * sqrt(eps_eff) / c.

Coupling: each net's centreline is sampled every --fine-step mm; at each sample the edge gap to
the nearest same-layer segment of the partner net is (centre distance - (w1 + w2)/2). A sample
is "coupled" if the edge gap <= coupled_max_mm = 2 x the netclass diff_pair_gap (stated per pair).
Uncoupled length = length of samples not coupled; also reported with a 1.0 mm threshold.
Gap statistics are over samples whose partner is within 1.0 mm (i.e. where the nets run parallel).
"""
import argparse, json, math, os, re, statistics, sys, hashlib
from collections import defaultdict

import pcbnew

C_MM_PER_PS = 0.299792458  # speed of light, mm per ps


# ---------------------------------------------------------------- stackup
def _sexpr(text, i):
    """Parse one s-expression starting at text[i] == '('. Returns (list, next_index)."""
    out = []
    i += 1
    tok = ''
    while i < len(text):
        ch = text[i]
        if ch == '(':
            sub, i = _sexpr(text, i)
            out.append(sub)
            continue
        if ch == ')':
            if tok:
                out.append(tok)
            return out, i + 1
        if ch == '"':
            j = i + 1
            while text[j] != '"':
                j += 2 if text[j] == '\\' else 1
            out.append(text[i + 1:j])
            i = j + 1
            continue
        if ch.isspace():
            if tok:
                out.append(tok); tok = ''
        else:
            tok += ch
        i += 1
    raise ValueError('unterminated s-expression')


def read_stackup(board_path):
    text = open(board_path, encoding='utf-8', errors='replace').read()
    i = text.find('(stackup')
    if i < 0:
        raise SystemExit('board has no (stackup) block: cannot compute via barrel or delay')
    st, _ = _sexpr(text, i)
    layers = []
    for item in st[1:]:
        if isinstance(item, list) and item and item[0] == 'layer':
            d = {'name': item[1]}
            for kv in item[2:]:
                if isinstance(kv, list) and len(kv) >= 2:
                    d[kv[0]] = kv[1]
            layers.append(d)
    z = 0.0
    cu = {}
    diel = []  # (z_top, z_bot, er)
    for d in layers:
        t = float(d.get('thickness', 0) or 0)
        typ = d.get('type', '')
        if typ == 'copper':
            cu[d['name']] = dict(z_top=z, z_mid=z + t / 2, z_bot=z + t, t=t)
        elif typ in ('core', 'prepreg') or d['name'].startswith('dielectric'):
            diel.append(dict(z_top=z, z_bot=z + t, t=t, er=float(d.get('epsilon_r', 4.5)),
                             name=d['name'], type=typ))
        if typ == 'copper' or typ in ('core', 'prepreg') or d['name'].startswith('dielectric'):
            z += t
    return dict(copper=cu, dielectrics=diel, total_mm=z, raw=layers)


# ---------------------------------------------------------------- pairing
POL_TOKENS = {'P': '+', 'N': '-', 'DP': '+', 'DM': '-', 'DN': '-'}


def pair_keys(name):
    """Return set of (key, polarity, rule) for a net name."""
    out = set()
    prefix, _, leaf = name.rpartition('/')
    toks = re.split(r'(_)', leaf)
    for idx, t in enumerate(toks):
        if t in POL_TOKENS:
            key = prefix + '/' + ''.join(toks[:idx] + ['<' + ('P' if t in ('P', 'N') else 'D') + '>'] + toks[idx + 1:])
            out.add((key, POL_TOKENS[t], 'token ' + ('_P/_N' if t in ('P', 'N') else 'DP/DM/DN')))
    last = toks[-1] if toks else ''
    if len(last) > 1 and last[-1] in 'PN' and last not in POL_TOKENS:
        key = prefix + '/' + ''.join(toks[:-1]) + last[:-1] + '<p>'
        out.add((key, '+' if last[-1] == 'P' else '-', 'trailing P/N'))
    if leaf.endswith(('+', '-')) and len(leaf) > 1:
        out.add((prefix + '/' + leaf[:-1] + '<+>', leaf[-1], 'trailing +/-'))
    if len(last) > 1 and last[-1] in 'HL' and 'CAN' in last.upper():
        key = prefix + '/' + ''.join(toks[:-1]) + last[:-1] + '<HL>'
        out.add((key, '+' if last[-1] == 'H' else '-', 'CAN H/L'))
    return out


def find_name_pairs(netnames):
    buckets = defaultdict(lambda: {'+': set(), '-': set(), 'rule': set()})
    for n in netnames:
        if n.startswith('unconnected-') or not n:
            continue
        for key, pol, rule in pair_keys(n):
            buckets[key][pol].add(n)
            buckets[key]['rule'].add(rule)
    pairs = {}
    for key, b in buckets.items():
        if len(b['+']) == 1 and len(b['-']) == 1:
            p, n = next(iter(b['+'])), next(iter(b['-']))
            pairs[(p, n)] = sorted(b['rule'])
    return pairs


def dru_diffpair_rules(dru_path):
    rules = []
    if not dru_path or not os.path.exists(dru_path):
        return rules
    txt = open(dru_path, errors='replace').read()
    for m in re.finditer(r'\(rule\s+"?([^"\n)]*)"?(.*?)\n\s*\)\s*$', txt, re.S | re.M):
        body = m.group(2)
        pats = re.findall(r"inDiffPair\('([^']+)'\)", body)
        cons = re.findall(r'\(constraint\s+([a-z_]+)([^)]*)\)', body)
        if pats or any(c[0] in ('skew', 'diff_pair_gap', 'diff_pair_uncoupled', 'length') for c in cons):
            rules.append(dict(rule=m.group(1), inDiffPair=pats, constraints=[' '.join(c).strip() for c in cons]))
    return rules


# ---------------------------------------------------------------- geometry
def seg_point_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def arc_points(trk, step):
    """Polyline for a PCB_ARC in mm."""
    s, m, e = trk.GetStart(), trk.GetMid(), trk.GetEnd()
    pts = [(pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)) for p in (s, m, e)]
    (x1, y1), (x2, y2), (x3, y3) = pts
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(d) < 1e-12:
        return [pts[0], pts[2]]
    ux = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / d
    uy = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / d
    r = math.hypot(x1 - ux, y1 - uy)
    a1 = math.atan2(y1 - uy, x1 - ux); a2 = math.atan2(y2 - uy, x2 - ux); a3 = math.atan2(y3 - uy, x3 - ux)

    def norm(a):
        return a % (2 * math.pi)
    ccw = norm(a2 - a1) < norm(a3 - a1)
    sweep = norm(a3 - a1) if ccw else -norm(a1 - a3)
    n = max(2, int(abs(sweep) * r / step) + 1)
    return [(ux + r * math.cos(a1 + sweep * k / n), uy + r * math.sin(a1 + sweep * k / n)) for k in range(n + 1)]


class NetGeom:
    def __init__(self, board, name, stackup, layer_names):
        self.name = name
        self.segs = []   # (layer, ax, ay, bx, by, w_mm)
        self.vias = []
        net = board.FindNet(name)
        self.netclass = net.GetNetClassName() if net else None
        for t in board.GetTracks():
            if t.GetNetname() != name:
                continue
            cls = t.GetClass()
            if cls == 'PCB_VIA':
                self.vias.append(t)
            elif cls == 'PCB_ARC':
                pts = arc_points(t, 0.02)
                lyr = layer_names(t.GetLayer())
                for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                    self.segs.append((lyr, ax, ay, bx, by, pcbnew.ToMM(t.GetWidth())))
            else:
                s, e = t.GetStart(), t.GetEnd()
                self.segs.append((layer_names(t.GetLayer()), pcbnew.ToMM(s.x), pcbnew.ToMM(s.y),
                                  pcbnew.ToMM(e.x), pcbnew.ToMM(e.y), pcbnew.ToMM(t.GetWidth())))
        self.pads = []
        for fp in board.GetFootprints():
            for p in fp.Pads():
                if p.GetNetname() == name:
                    self.pads.append(p)


def via_clusters(ng, layer_names):
    """Group this net's vias that overlap in x/y (stacked microvia + buried via chains)."""
    items = []
    for v in ng.vias:
        p = v.GetPosition()
        try:
            r = pcbnew.ToMM(v.GetWidth(v.TopLayer())) / 2
        except TypeError:
            r = pcbnew.ToMM(v.GetWidth()) / 2
        items.append([v, pcbnew.ToMM(p.x), pcbnew.ToMM(p.y), r])
    clusters = []
    for it in items:
        for c in clusters:
            if any(math.hypot(it[1] - o[1], it[2] - o[2]) <= min(it[3], o[3]) for o in c):
                c.append(it)
                break
        else:
            clusters.append([it])
    return clusters


def cluster_barrel(cluster, ng, stackup, layer_names):
    """Barrel length (mm) of a via stack between the outermost copper layers on which this net's
    tracks or pads touch it. Tracks count if their copper overlaps the via pad (end or pass-through)."""
    span_layers = set()
    for v, vx, vy, r in cluster:
        span_layers |= {layer_names(l) for l in v.GetLayerSet().CuStack()}
    hit = set()
    for lyr, ax, ay, bx, by, w in ng.segs:
        if lyr in span_layers and any(seg_point_dist(vx, vy, ax, ay, bx, by) <= r + w / 2 - 1e-4
                                      for v, vx, vy, r in cluster):
            hit.add(lyr)
    for p in ng.pads:
        for l in p.GetLayerSet().CuStack():
            ln = layer_names(l)
            if ln in span_layers and any(p.HitTest(v.GetPosition()) for v, vx, vy, r in cluster):
                hit.add(ln)
    cu = stackup['copper']
    zs = [cu[l]['z_mid'] for l in hit if l in cu]
    barrel = (max(zs) - min(zs)) if len(zs) >= 2 else 0.0
    spans = [f"{layer_names(v.TopLayer())}-{layer_names(v.BottomLayer())}" for v, *_ in cluster]
    return barrel, sorted(hit, key=lambda l: cu.get(l, {}).get('z_mid', 0)), spans, cluster[0][1], cluster[0][2]


def eps_eff_for(layer, width_mm, stackup):
    cu = stackup['copper']
    names = sorted(cu, key=lambda l: cu[l]['z_mid'])
    if layer not in cu:
        return None, None
    idx = names.index(layer)
    if idx in (0, len(names) - 1):
        # adjacent dielectric
        z = cu[layer]['z_bot'] if idx == 0 else cu[layer]['z_top']
        d = [x for x in stackup['dielectrics'] if (abs(x['z_top'] - z) < 1e-6 if idx == 0 else abs(x['z_bot'] - z) < 1e-6)]
        if not d:
            return None, None
        h, er = d[0]['t'], d[0]['er']
        ee = (er + 1) / 2 + (er - 1) / 2 * (1 + 12 * h / width_mm) ** -0.5
        return ee, dict(model='microstrip (Hammerstad)', h_mm=h, er=er)
    above = [x['er'] for x in stackup['dielectrics'] if abs(x['z_bot'] - cu[layer]['z_top']) < 1e-6]
    below = [x['er'] for x in stackup['dielectrics'] if abs(x['z_top'] - cu[layer]['z_bot']) < 1e-6]
    ers = above + below
    er = sum(ers) / len(ers) if ers else 4.2
    return er, dict(model='stripline (eps_eff = er)', er=er)


def net_metrics(board, ng, stackup, layer_names):
    by_layer = defaultdict(float)
    delay_ps = 0.0
    eps_used = {}
    for lyr, ax, ay, bx, by, w in ng.segs:
        L = math.hypot(bx - ax, by - ay)
        by_layer[lyr] += L
        ee, info = eps_eff_for(lyr, w, stackup)
        if ee is None:
            ee = 4.2
        eps_used[f'{lyr}@w={w:.3f}'] = dict(eps_eff=round(ee, 4), **(info or {}))
        delay_ps += L * math.sqrt(ee) / C_MM_PER_PS
    vias = []
    barrel_total = 0.0
    er_bulk = [d['er'] for d in stackup['dielectrics']]
    er_via = sum(er_bulk) / len(er_bulk) if er_bulk else 4.2
    for cl in via_clusters(ng, layer_names):
        bl, hit, spans, x, y = cluster_barrel(cl, ng, stackup, layer_names)
        barrel_total += bl
        delay_ps += bl * math.sqrt(er_via) / C_MM_PER_PS
        vias.append(dict(spans=spans, connected_layers=hit, barrel_mm=round(bl, 4),
                         x=round(x, 3), y=round(y, 3)))
    track = sum(by_layer.values())
    return dict(net=ng.name, netclass=ng.netclass, track_mm=track, via_count=len(ng.vias),
                via_barrel_mm=barrel_total, routed_mm=track + barrel_total, delay_ps=delay_ps,
                layers={k: round(v, 4) for k, v in sorted(by_layer.items())},
                vias=vias, eps_eff_used=eps_used, segments=len(ng.segs), pads=len(ng.pads),
                pad_to_pad=pad_paths(ng, stackup, layer_names))


def pad_paths(ng, stackup, layer_names):
    """Shortest copper path between every pair of this net's pads (Dijkstra over tracks + via stacks).
    Returns {"REF1.pad-REF2.pad": {"length_mm", "delay_ps", "refs": [REF1, REF2]}}."""
    import heapq
    cu = stackup['copper']
    er_bulk = [d['er'] for d in stackup['dielectrics']]
    er_via = sum(er_bulk) / len(er_bulk) if er_bulk else 4.2
    adj = defaultdict(list)

    def add(a, b, L, t):
        adj[a].append((b, L, t)); adj[b].append((a, L, t))
    ends = []  # (node, layer, x, y, w)
    for i, (lyr, ax, ay, bx, by, w) in enumerate(ng.segs):
        L = math.hypot(bx - ax, by - ay)
        ee, _ = eps_eff_for(lyr, w, stackup)
        t = L * math.sqrt(ee or 4.2) / C_MM_PER_PS
        na, nb = ('s', i, 0), ('s', i, 1)
        add(na, nb, L, t)
        ends += [(na, lyr, ax, ay, w), (nb, lyr, bx, by, w)]
    # coincident / overlapping endpoints on the same layer
    for i in range(len(ends)):
        for j in range(i + 1, len(ends)):
            a, b = ends[i], ends[j]
            if a[1] == b[1] and math.hypot(a[2] - b[2], a[3] - b[3]) <= min(a[4], b[4]) / 2:
                add(a[0], b[0], 0.0, 0.0)
    # T-junctions: endpoint lying on another segment's copper
    for node, lyr, x, y, w in ends:
        for i, (l2, ax, ay, bx, by, w2) in enumerate(ng.segs):
            if l2 != lyr or node[1] == i:
                continue
            L = math.hypot(bx - ax, by - ay)
            if L == 0 or seg_point_dist(x, y, ax, ay, bx, by) > w2 / 2:
                continue
            tt = max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (L * L)))
            if 0.0 < tt < 1.0:
                ee, _ = eps_eff_for(l2, w2, stackup)
                k = math.sqrt(ee or 4.2) / C_MM_PER_PS
                add(node, ('s', i, 0), tt * L, tt * L * k)
                add(node, ('s', i, 1), (1 - tt) * L, (1 - tt) * L * k)
    # via stacks
    for ci, cl in enumerate(via_clusters(ng, layer_names)):
        layers = set()
        for v, vx, vy, r in cl:
            layers |= {layer_names(l) for l in v.GetLayerSet().CuStack()}
        order = sorted((l for l in layers if l in cu), key=lambda l: cu[l]['z_mid'])
        for l1, l2 in zip(order, order[1:]):
            dz = cu[l2]['z_mid'] - cu[l1]['z_mid']
            add(('v', ci, l1), ('v', ci, l2), dz, dz * math.sqrt(er_via) / C_MM_PER_PS)
        for node, lyr, x, y, w in ends:
            if lyr in layers and any(math.hypot(x - vx, y - vy) <= r + w / 2 for v, vx, vy, r in cl):
                add(node, ('v', ci, lyr), 0.0, 0.0)
        # tracks that pass through the via pad without ending on it
        for i, (l2, ax, ay, bx, by, w2) in enumerate(ng.segs):
            if l2 not in layers:
                continue
            L = math.hypot(bx - ax, by - ay)
            for v, vx, vy, r in cl:
                if L == 0 or seg_point_dist(vx, vy, ax, ay, bx, by) > r + w2 / 2:
                    continue
                if min(math.hypot(ax - vx, ay - vy), math.hypot(bx - vx, by - vy)) <= r + w2 / 2:
                    continue
                tt = max(0.0, min(1.0, ((vx - ax) * (bx - ax) + (vy - ay) * (by - ay)) / (L * L)))
                ee, _ = eps_eff_for(l2, w2, stackup)
                k = math.sqrt(ee or 4.2) / C_MM_PER_PS
                add(('v', ci, l2), ('s', i, 0), tt * L, tt * L * k)
                add(('v', ci, l2), ('s', i, 1), (1 - tt) * L, (1 - tt) * L * k)
        for p in ng.pads:
            for l in p.GetLayerSet().CuStack():
                ln = layer_names(l)
                if ln in layers and any(p.HitTest(v.GetPosition()) for v, *_ in cl):
                    add(('p', p.GetParentFootprint().GetReference(), p.GetNumber()), ('v', ci, ln), 0.0, 0.0)
    pad_nodes = []
    for p in ng.pads:
        pn = ('p', p.GetParentFootprint().GetReference(), p.GetNumber())
        pad_nodes.append(pn)
        players = {layer_names(l) for l in p.GetLayerSet().CuStack()}
        for node, lyr, x, y, w in ends:
            if lyr in players and p.HitTest(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)), pcbnew.FromMM(w / 2)):
                add(pn, node, 0.0, 0.0)
    out = {}
    for i, src in enumerate(pad_nodes):
        dist = {src: (0.0, 0.0)}
        pq = [(0.0, 0.0, 0, src)]
        cnt = 1
        while pq:
            d, t, _, u = heapq.heappop(pq)
            if d > dist[u][0] + 1e-12:
                continue
            for v, L, tt in adj[u]:
                nd = d + L
                if v not in dist or nd < dist[v][0] - 1e-12:
                    dist[v] = (nd, t + tt)
                    heapq.heappush(pq, (nd, t + tt, cnt, v)); cnt += 1
        for dst in pad_nodes[i + 1:]:
            if dst[1] == src[1]:
                continue
            key = f"{src[1]}.{src[2]}-{dst[1]}.{dst[2]}"
            if dst in dist:
                out[key] = dict(length_mm=round(dist[dst][0], 4), delay_ps=round(dist[dst][1], 3),
                                refs=sorted([src[1], dst[1]]))
            else:
                out[key] = dict(length_mm=None, delay_ps=None, refs=sorted([src[1], dst[1]]),
                                note='no copper path')
    return out


def coupling(ga, gb, coupled_max, step, alt_max=1.0):
    """Sample ga's centreline; edge gap to nearest same-layer segment of gb."""
    by_layer = defaultdict(list)
    for s in gb.segs:
        by_layer[s[0]].append(s)
    coupled_len = unc_len = unc_alt = 0.0
    gaps = []
    for lyr, ax, ay, bx, by, w in ga.segs:
        L = math.hypot(bx - ax, by - ay)
        if L == 0:
            continue
        n = max(1, int(math.ceil(L / step)))
        dl = L / n
        others = by_layer.get(lyr, [])
        for k in range(n):
            t = (k + 0.5) / n
            px, py = ax + (bx - ax) * t, ay + (by - ay) * t
            best = None
            for o in others:
                d = seg_point_dist(px, py, o[1], o[2], o[3], o[4]) - (w + o[5]) / 2
                if best is None or d < best:
                    best = d
            if best is not None and best <= coupled_max:
                coupled_len += dl
            else:
                unc_len += dl
            if best is None or best > alt_max:
                unc_alt += dl
            else:
                gaps.append(best)
    gs = sorted(gaps)
    stats = None
    if gs:
        stats = dict(min=round(gs[0], 4), median=round(statistics.median(gs), 4),
                     p95=round(gs[min(len(gs) - 1, int(0.95 * len(gs)))], 4), max=round(gs[-1], 4))
    return dict(coupled_mm=round(coupled_len, 3), uncoupled_mm=round(unc_len, 3),
                uncoupled_at_1mm_mm=round(unc_alt, 3), gap_stats_mm=stats)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--md')
    ap.add_argument('--limits')
    ap.add_argument('--fine-step', type=float, default=0.05)
    a = ap.parse_args()

    bp = os.path.abspath(a.board)
    board = pcbnew.LoadBoard(bp)
    layer_names = board.GetLayerName
    stackup = read_stackup(bp)
    pro = os.path.splitext(bp)[0] + '.kicad_pro'
    dru = os.path.splitext(bp)[0] + '.kicad_dru'
    proj = json.load(open(pro)) if os.path.exists(pro) else {}
    classes = {c['name']: c for c in proj.get('net_settings', {}).get('classes', [])}

    netnames = sorted(str(n) for n in board.GetNetsByName().keys())
    name_pairs = find_name_pairs(netnames)

    # netclass-based
    nc_members = defaultdict(list)
    for n in netnames:
        net = board.FindNet(n)
        if net:
            nc_members[net.GetNetClassName()].append(n)
    nc_pairs = {}
    for nc, members in nc_members.items():
        if 'DIFF' in nc.upper():
            for (p, n), rules in find_name_pairs(members).items():
                nc_pairs[(p, n)] = nc
    dru_rules = dru_diffpair_rules(dru)

    groups = {}  # key -> dict(p=[..], n=[..], sources=[..], limit)
    for (p, n), rules in name_pairs.items():
        groups[(p,), (n,)] = dict(name=f'{p} / {n}', p=[p], n=[n], sources=['name: ' + ', '.join(rules)], limits=[])
    for (p, n), nc in nc_pairs.items():
        g = groups.setdefault(((p,), (n,)), dict(name=f'{p} / {n}', p=[p], n=[n], sources=[], limits=[]))
        g['sources'].append(f'netclass: {nc}')
    for r in dru_rules:
        for pat in r['inDiffPair']:
            for (pp, nn), g in groups.items():
                if any(re.fullmatch(pat.replace('*', '.*'), x) for x in g['p'] + g['n']):
                    g['sources'].append(f"dru: {r['rule']}")
                    g['limits'].append(dict(source='dru ' + r['rule'], constraints=r['constraints']))
    if a.limits:
        lim = json.load(open(a.limits))
        for e in lim['pairs']:
            key = (tuple(e['p']), tuple(e['n']))
            g = groups.setdefault(key, dict(name=e.get('name', ' / '.join(e['p'] + e['n'])), p=list(e['p']),
                                            n=list(e['n']), sources=[], limits=[]))
            g['sources'].append(f"limits file: {e.get('source', '')}")
            if e.get('name'):
                g['name'] = e['name']
            g['limits'].append(dict(source=e.get('source', ''), max_skew_mm=e['max_skew_mm'],
                                    require_equal_vias=e.get('require_equal_vias')))

    geoms = {}

    def geom(n):
        if n not in geoms:
            geoms[n] = NetGeom(board, n, stackup, layer_names)
        return geoms[n]

    results = []
    for key, g in sorted(groups.items(), key=lambda kv: kv[1]['name']):
        missing = [n for n in g['p'] + g['n'] if not board.FindNet(n)]
        if missing:
            results.append(dict(name=g['name'], p=g['p'], n=g['n'], sources=g['sources'], limits=g['limits'],
                                status='NET MISSING ON BOARD', missing=missing))
            continue
        mp = [net_metrics(board, geom(n), stackup, layer_names) for n in g['p']]
        mn = [net_metrics(board, geom(n), stackup, layer_names) for n in g['n']]

        def agg(ms, k):
            return sum(m[k] for m in ms)
        P = dict(track_mm=agg(mp, 'track_mm'), via_barrel_mm=agg(mp, 'via_barrel_mm'), routed_mm=agg(mp, 'routed_mm'),
                 delay_ps=agg(mp, 'delay_ps'), via_count=agg(mp, 'via_count'))
        N = dict(track_mm=agg(mn, 'track_mm'), via_barrel_mm=agg(mn, 'via_barrel_mm'), routed_mm=agg(mn, 'routed_mm'),
                 delay_ps=agg(mn, 'delay_ps'), via_count=agg(mn, 'via_count'))
        nc = mp[0]['netclass']
        dpg = float(classes.get(nc, classes.get('Default', {})).get('diff_pair_gap', 0.2) or 0.2)
        cmax = 2 * dpg
        cps, cns = [], []
        for x, y in zip(g['p'], g['n']):
            cps.append(coupling(geom(x), geom(y), cmax, a.fine_step))
            cns.append(coupling(geom(y), geom(x), cmax, a.fine_step))
        routed = P['track_mm'] > 0 and N['track_mm'] > 0
        skew_mm = abs(P['routed_mm'] - N['routed_mm'])
        skew_track_mm = abs(P['track_mm'] - N['track_mm'])
        skew_ps = abs(P['delay_ps'] - N['delay_ps'])
        path_skew = []
        if len(mp) == 1 and len(mn) == 1:
            for kp, vp in mp[0]['pad_to_pad'].items():
                for kn, vn in mn[0]['pad_to_pad'].items():
                    if vp['refs'] == vn['refs'] and vp['length_mm'] is not None and vn['length_mm'] is not None:
                        path_skew.append(dict(refs='-'.join(vp['refs']), p_path=kp, n_path=kn,
                                              p_mm=vp['length_mm'], n_mm=vn['length_mm'],
                                              skew_mm=round(abs(vp['length_mm'] - vn['length_mm']), 4),
                                              skew_ps=round(abs(vp['delay_ps'] - vn['delay_ps']), 3)))
            pp = [v for v in mp[0]['pad_to_pad'].values() if v['length_mm'] is not None]
            nn = [v for v in mn[0]['pad_to_pad'].values() if v['length_mm'] is not None]
            if not path_skew and len(pp) == 1 and len(nn) == 1:
                kp = next(k for k, v in mp[0]['pad_to_pad'].items() if v['length_mm'] is not None)
                kn = next(k for k, v in mn[0]['pad_to_pad'].items() if v['length_mm'] is not None)
                path_skew.append(dict(refs='single path each (refs differ)', p_path=kp, n_path=kn,
                                      p_mm=pp[0]['length_mm'], n_mm=nn[0]['length_mm'],
                                      skew_mm=round(abs(pp[0]['length_mm'] - nn[0]['length_mm']), 4),
                                      skew_ps=round(abs(pp[0]['delay_ps'] - nn[0]['delay_ps']), 3)))
        verdicts = []
        for lim in g['limits']:
            if 'max_skew_mm' in lim:
                ok_tool = routed and skew_track_mm <= lim['max_skew_mm']
                ok_full = routed and skew_mm <= lim['max_skew_mm']
                v = dict(source=lim['source'], max_skew_mm=lim['max_skew_mm'],
                         pass_track_only=ok_tool, pass_with_via_barrels=ok_full)
                if lim.get('require_equal_vias'):
                    v['equal_vias'] = P['via_count'] == N['via_count']
                verdicts.append(v)
        results.append(dict(
            name=g['name'], p=g['p'], n=g['n'], sources=sorted(set(g['sources'])), netclass=nc,
            status='ROUTED' if routed else 'NOT ROUTED (one or both nets have no tracks)',
            p_metrics=mp, n_metrics=mn,
            P={k: round(v, 4) for k, v in P.items()}, N={k: round(v, 4) for k, v in N.items()},
            skew_mm=round(skew_mm, 4), skew_track_only_mm=round(skew_track_mm, 4), skew_ps=round(skew_ps, 3),
            layers_p=sorted({l for m in mp for l in m['layers']}), layers_n=sorted({l for m in mn for l in m['layers']}),
            coupled_max_edge_gap_mm=cmax, coupled_max_basis=f'2 x netclass {nc!r} diff_pair_gap {dpg} mm',
            coupling_p=cps, coupling_n=cns,
            uncoupled_mm=dict(p=round(sum(c['uncoupled_mm'] for c in cps), 3), n=round(sum(c['uncoupled_mm'] for c in cns), 3)),
            uncoupled_at_1mm=dict(p=round(sum(c['uncoupled_at_1mm_mm'] for c in cps), 3),
                                  n=round(sum(c['uncoupled_at_1mm_mm'] for c in cns), 3)),
            pad_to_pad_skew=path_skew,
            limits=g['limits'], limit_verdicts=verdicts or 'no limit declared'))

    out = dict(
        board=os.path.basename(bp), board_sha256=hashlib.sha256(open(bp, 'rb').read()).hexdigest(),
        kicad=pcbnew.GetBuildVersion(),
        stackup=dict(copper={k: {kk: round(vv, 4) for kk, vv in v.items()} for k, v in stackup['copper'].items()},
                     dielectrics=stackup['dielectrics'], total_mm=round(stackup['total_mm'], 4)),
        assumptions=[
            'routed length = sum of track/arc lengths (all branches of the net) + via barrel between outermost connected copper layers (copper-centre to copper-centre)',
            'outer-layer delay: quasi-static microstrip eps_eff (Hammerstad) per segment width and adjacent dielectric; solder mask, odd/even-mode split and dispersion ignored',
            'inner-layer delay: eps_eff = er; via barrel delay: eps_eff = mean dielectric er',
            'uncoupled length reported at two thresholds: same-layer edge gap > 2 x netclass diff_pair_gap, and > 1.0 mm; gap stats over samples whose same-layer partner is within 1.0 mm edge gap',
            'series parts (resistors, ESD, chokes) are not part of any net length',
        ],
        dru_diffpair_rules=dru_rules,
        netclass_diff_pair_settings={k: dict(diff_pair_gap=v.get('diff_pair_gap'), diff_pair_width=v.get('diff_pair_width'))
                                     for k, v in classes.items()},
        pairs=results)
    json.dump(out, open(a.out, 'w'), indent=1)

    if a.md:
        L = [f"# Differential-pair skew — {out['board']} (sha256 {out['board_sha256'][:12]}…)", '',
             'Assumptions: ' + '; '.join(out['assumptions']) + '.', '',
             '| pair | source | layers P / N | track P / N (mm) | vias P / N | barrel P / N (mm) | skew (mm) | skew track-only (mm) | skew (ps) | pad-to-pad skew by matching refs (mm / ps) | uncoupled P / N (mm) @2×dp_gap ; @1.0 mm | gap median [min–max] where within 1 mm (mm) | limit → verdict |',
             '|---|---|---|---|---|---|---:|---:|---:|---|---|---|---|']
        for r in out['pairs']:
            if r.get('status', '').startswith('NET MISSING'):
                L.append(f"| {r['name']} | {'; '.join(r['sources'])} | missing {r['missing']} | | | | | | | | | | |")
                continue
            gs = [c['gap_stats_mm'] for c in r['coupling_p'] if c['gap_stats_mm']]
            gtxt = '; '.join(f"{s['median']} [{s['min']}–{s['max']}]" for s in gs) or '—'
            lv = r['limit_verdicts']
            if isinstance(lv, str):
                ltxt = lv
            else:
                ltxt = '; '.join(f"{v['max_skew_mm']} mm ({v['source']}): track-only {'PASS' if v['pass_track_only'] else 'FAIL'}, with barrels {'PASS' if v['pass_with_via_barrels'] else 'FAIL'}"
                                 + (f", vias equal {v['equal_vias']}" if 'equal_vias' in v else '') for v in lv)
            if not r['status'].startswith('ROUTED'):
                ltxt = r['status'] + ('; ' + ltxt if ltxt else '')
            L.append('| ' + ' | '.join([
                r['name'], '; '.join(r['sources']),
                f"{','.join(r['layers_p'])} / {','.join(r['layers_n'])}",
                f"{r['P']['track_mm']:.3f} / {r['N']['track_mm']:.3f}",
                f"{r['P']['via_count']} / {r['N']['via_count']}",
                f"{r['P']['via_barrel_mm']:.3f} / {r['N']['via_barrel_mm']:.3f}",
                f"{r['skew_mm']:.3f}", f"{r['skew_track_only_mm']:.3f}", f"{r['skew_ps']:.2f}",
                '; '.join(f"{x['refs']}: {x['p_mm']:.3f}/{x['n_mm']:.3f} → {x['skew_mm']:.3f} / {x['skew_ps']:.1f}" for x in r['pad_to_pad_skew']) or '—',
                f"{r['uncoupled_mm']['p']:.2f} / {r['uncoupled_mm']['n']:.2f} (≤{r['coupled_max_edge_gap_mm']:.2f}) ; {r['uncoupled_at_1mm']['p']:.2f} / {r['uncoupled_at_1mm']['n']:.2f}", gtxt, ltxt]) + ' |')
        open(a.md, 'w').write('\n'.join(L) + '\n')
    print(json.dumps([dict(name=r['name'], status=r.get('status'), skew_mm=r.get('skew_mm'),
                           skew_track=r.get('skew_track_only_mm'), skew_ps=r.get('skew_ps'),
                           unc=r.get('uncoupled_mm'), verdict=r.get('limit_verdicts')) for r in out['pairs']], indent=0))


if __name__ == '__main__':
    main()
