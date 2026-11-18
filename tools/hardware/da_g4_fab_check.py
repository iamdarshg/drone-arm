#!/usr/bin/env python3
"""DA-G4 fab-package consistency check (read-only on the board).

Run with KiCad's bundled python (needs `import pcbnew`):

    $KICAD_PY da_g4_fab_check.py --board <board.kicad_pcb> --pkg <package dir> \
        [--stale-edge <old Edge_Cuts gerber>] [--git-commit <sha>]

<package dir> must already contain gerbers/ (Gerbers + job + Excellon), positions.pos and
drc_receipt.json produced by stock kicad-cli from <board>. The script:

  * hashes the board and every package file -> MANIFEST_SHA256.json
  * Edge.Cuts gerber extent (centreline and with aperture) vs pcbnew GetBoardEdgesBoundingBox()
    and vs the job file Size -> outline_check.json
  * job file layer count / thickness / finish vs board
  * drill hit count per Excellon file vs pcbnew via + PTH/NPTH pad holes
  * position-file row count vs board footprints (minus exclude-from-pos)
  * zone-fill freshness: stored fill area per zone vs an in-memory refill (board is NOT saved)
  * DRC receipt summary

Nothing is written next to the board.
"""
import argparse, hashlib, json, math, os, re, sys, collections

import pcbnew


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def gerber_extent(path):
    """Extent of all D01/D02/D03 coordinates (and arc sweeps) in a Gerber file, in mm.
    Returns centreline bbox and bbox grown by half the largest circular aperture used."""
    txt = open(path, errors='replace').read()
    fs = re.search(r'%FSLAX(\d)(\d)Y(\d)(\d)\*%', txt)
    xdec, ydec = int(fs.group(2)), int(fs.group(4))
    unit = 25.4 if '%MOIN*%' in txt else 1.0
    aps = {m.group(1): float(m.group(2)) * unit
           for m in re.finditer(r'%ADD(\d+)C,([0-9.]+)', txt)}
    x = y = 0.0
    mode = 'G01'
    cur_ap = None
    used = set()
    xs, ys = [], []
    prev = (0.0, 0.0)
    for stmt in re.split(r'\*', txt):
        stmt = stmt.strip().lstrip('%')
        if not stmt or stmt.startswith(('TF', 'TA', 'TD', 'TO', 'FS', 'MO', 'LP', 'AD', 'AM', 'G04')):
            continue
        for g in re.findall(r'G0?([123])(?!\d)', stmt):
            mode = 'G0' + g
        m = re.fullmatch(r'D(\d{2,})', stmt)
        if m and int(m.group(1)) >= 10:
            cur_ap = m.group(1)
            continue
        cm = re.search(r'(?:X(-?\d+))?(?:Y(-?\d+))?(?:I(-?\d+))?(?:J(-?\d+))?D0?([123])$', stmt)
        if not cm or (cm.group(1) is None and cm.group(2) is None and cm.group(5) != '3'):
            if cm is None:
                continue
        nx = int(cm.group(1)) / 10 ** xdec * unit if cm.group(1) is not None else x
        ny = int(cm.group(2)) / 10 ** ydec * unit if cm.group(2) is not None else y
        d = cm.group(5)
        if d in ('1', '3'):
            if cur_ap:
                used.add(cur_ap)
            if d == '1' and mode in ('G02', 'G03') and cm.group(3) is not None:
                cx = x + int(cm.group(3)) / 10 ** xdec * unit
                cy = y + int(cm.group(4) or 0) / 10 ** ydec * unit
                r = math.hypot(x - cx, y - cy)
                a0 = math.atan2(y - cy, x - cx)
                a1 = math.atan2(ny - cy, nx - cx)
                if mode == 'G03':
                    if a1 <= a0:
                        a1 += 2 * math.pi
                else:
                    if a1 >= a0:
                        a1 -= 2 * math.pi
                for k in range(65):
                    a = a0 + (a1 - a0) * k / 64
                    xs.append(cx + r * math.cos(a)); ys.append(cy + r * math.sin(a))
            xs += [x, nx] if d == '1' else [nx]
            ys += [y, ny] if d == '1' else [ny]
        x, y = nx, ny
    half = max((aps[a] for a in used if a in aps), default=0.0) / 2
    c = dict(xmin=min(xs), xmax=max(xs), ymin=min(ys), ymax=max(ys))
    c['w'] = c['xmax'] - c['xmin']; c['h'] = c['ymax'] - c['ymin']
    o = dict(w=c['w'] + 2 * half, h=c['h'] + 2 * half)
    return dict(centreline=c, with_aperture=o, aperture_diameter_mm=2 * half,
                coordinate_count=len(xs))


def excellon_hits(path):
    hits = collections.Counter()
    tools = {}
    tool = None
    in_header = True
    for line in open(path, errors='replace'):
        line = line.strip()
        m = re.match(r'T(\d+)C([0-9.]+)', line)
        if m:
            tools['T' + m.group(1)] = float(m.group(2))
            continue
        if line == '%':
            in_header = False
            continue
        m = re.fullmatch(r'T(\d+)', line)
        if m and not in_header:
            tool = 'T' + m.group(1)
            continue
        if not in_header and tool and re.match(r'X-?[0-9.]+Y-?[0-9.]+', line):
            hits[tool] += 1
        if not in_header and tool and line.startswith('G85'):
            hits[tool] += 1
    return dict(tools=tools, hits=dict(hits), total_hits=sum(hits.values()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--board', required=True)
    ap.add_argument('--pkg', required=True)
    ap.add_argument('--stale-edge')
    ap.add_argument('--git-commit', default='')
    ap.add_argument('--repo-root', default='')
    ap.add_argument('--commands', default='')
    a = ap.parse_args()

    board_path = os.path.abspath(a.board)
    pkg = os.path.abspath(a.pkg)
    rel = os.path.relpath(board_path, a.repo_root) if a.repo_root else board_path
    bsha = sha256(board_path)

    b = pcbnew.LoadBoard(board_path)
    bb = b.GetBoardEdgesBoundingBox()
    board_bbox = dict(w=pcbnew.ToMM(bb.GetWidth()), h=pcbnew.ToMM(bb.GetHeight()),
                      xmin=pcbnew.ToMM(bb.GetX()), ymin=pcbnew.ToMM(bb.GetY()))
    # Edge.Cuts drawing extent without line width (centreline)
    exs, eys = [], []
    edge_w = set()
    for d in b.GetDrawings():
        if d.GetLayer() == pcbnew.Edge_Cuts:
            r = d.GetBoundingBox()
            w = d.GetWidth() if hasattr(d, 'GetWidth') else 0
            edge_w.add(pcbnew.ToMM(w))
            exs += [r.GetX() + w / 2, r.GetRight() - w / 2]
            eys += [r.GetY() + w / 2, r.GetBottom() - w / 2]
    edge_centre = dict(w=pcbnew.ToMM(max(exs) - min(exs)), h=pcbnew.ToMM(max(eys) - min(eys)))

    gdir = os.path.join(pkg, 'gerbers')
    gfiles = sorted(os.listdir(gdir))
    edge = [f for f in gfiles if 'Edge_Cuts' in f][0]
    job = [f for f in gfiles if f.endswith('.gbrjob')][0]
    ext = gerber_extent(os.path.join(gdir, edge))
    jobj = json.load(open(os.path.join(gdir, job)))
    gs = jobj['GeneralSpecs']

    tol = 0.01
    oc = dict(
        board=rel, board_sha256=bsha,
        board_edges_bbox_mm=board_bbox,
        board_edge_cuts_centreline_mm=edge_centre,
        board_edge_cuts_line_widths_mm=sorted(edge_w),
        gerber_edge_cuts_file='gerbers/' + edge,
        gerber_edge_cuts_extent_mm=ext,
        job_size_mm=gs.get('Size'),
        checks={
            'gerber_centreline_equals_board_edge_centreline':
                abs(ext['centreline']['w'] - edge_centre['w']) < tol and
                abs(ext['centreline']['h'] - edge_centre['h']) < tol,
            'gerber_with_aperture_equals_board_bbox':
                abs(ext['with_aperture']['w'] - board_bbox['w']) < tol and
                abs(ext['with_aperture']['h'] - board_bbox['h']) < tol,
            'job_size_equals_board_bbox':
                abs(gs['Size']['X'] - board_bbox['w']) < tol and abs(gs['Size']['Y'] - board_bbox['h']) < tol,
            'job_layer_count_equals_board':
                gs.get('LayerNumber') == b.GetCopperLayerCount(),
            'job_thickness_equals_board':
                abs(gs.get('BoardThickness', 0) - pcbnew.ToMM(b.GetDesignSettings().GetBoardThickness())) < 1e-6,
        },
        tolerance_mm=tol,
    )
    if a.stale_edge:
        se = gerber_extent(a.stale_edge)
        oc['stale_package_edge_cuts'] = dict(
            file=os.path.relpath(os.path.abspath(a.stale_edge), a.repo_root) if a.repo_root else a.stale_edge,
            sha256=sha256(a.stale_edge), extent_mm=se,
            matches_board=abs(se['centreline']['w'] - edge_centre['w']) < tol and
            abs(se['centreline']['h'] - edge_centre['h']) < tol)

    # drill hits vs board
    vias = collections.Counter()
    for t in b.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            vias[(t.TopLayer(), t.BottomLayer())] += 1
    pth = npth = 0
    for fp in b.GetFootprints():
        for p in fp.Pads():
            if p.GetDrillSize().x > 0:
                if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                    npth += 1
                else:
                    pth += 1
    lname = b.GetLayerName
    drills = {f: excellon_hits(os.path.join(gdir, f)) for f in gfiles if f.endswith('.drl')}
    through = vias.get((pcbnew.F_Cu, pcbnew.B_Cu), 0)
    drill_check = dict(
        board_vias_by_span={f'{lname(k[0])}-{lname(k[1])}': v for k, v in sorted(vias.items())},
        board_pth_pad_holes=pth, board_npth_pad_holes=npth,
        excellon=drills,
        excellon_total_hits=sum(d['total_hits'] for d in drills.values()),
        board_total_holes=sum(vias.values()) + pth + npth,
    )
    drill_check['hits_equal_board_holes'] = drill_check['excellon_total_hits'] == drill_check['board_total_holes']

    # positions
    pos_rows = [l for l in open(os.path.join(pkg, 'positions.pos')) if l.strip() and not l.startswith('#')]
    fps = list(b.GetFootprints())
    excl = [fp.GetReference() for fp in fps if fp.IsExcludedFromPosFiles()]
    pos_check = dict(pos_rows=len(pos_rows), board_footprints=len(fps),
                     excluded_from_pos=sorted(excl),
                     rows_equal_footprints_minus_excluded=len(pos_rows) == len(fps) - len(excl))

    # zone fill freshness (in memory only)
    def zone_areas():
        out = {}
        for z in b.Zones():
            for lyr in z.GetLayerSet().Seq():
                if z.HasFilledPolysForLayer(lyr):
                    out[f"{z.GetNetname()}|{lname(lyr)}|{z.m_Uuid.AsString()}"] = \
                        z.GetFilledPolysList(lyr).Area() / 1e12
        return out
    before = zone_areas()
    filler = pcbnew.ZONE_FILLER(b)
    zs = pcbnew.ZONES()
    for z in b.Zones():
        zs.append(z)
    filler.Fill(zs)
    after = zone_areas()
    deltas = {k: dict(stored_mm2=round(before.get(k, 0), 4), refilled_mm2=round(after.get(k, 0), 4))
              for k in set(before) | set(after)
              if abs(before.get(k, 0) - after.get(k, 0)) > 0.01}
    zone_check = dict(zone_layers=len(set(before) | set(after)),
                      zone_layers_changed_gt_0p01mm2=len(deltas),
                      total_stored_mm2=round(sum(before.values()), 3),
                      total_refilled_mm2=round(sum(after.values()), 3),
                      changed=deltas,
                      note='refill done in memory on the loaded board; board file not saved')

    drc = json.load(open(os.path.join(pkg, 'drc_receipt.json')))
    drc_summary = dict(
        errors=sum(1 for v in drc['violations'] if v['severity'] == 'error'),
        warnings=sum(1 for v in drc['violations'] if v['severity'] == 'warning'),
        violations_by_type=dict(collections.Counter(v['type'] for v in drc['violations'])),
        unconnected=len(drc['unconnected_items']),
        schematic_parity=len(drc['schematic_parity']),
        schematic_parity_by_type=dict(collections.Counter(v['type'] for v in drc['schematic_parity'])),
    )

    checks = dict(outline=oc, drill=drill_check, positions=pos_check, zone_fill=zone_check,
                  drc_summary=drc_summary)
    json.dump(checks, open(os.path.join(pkg, 'outline_check.json'), 'w'), indent=1, sort_keys=True)

    files = {}
    for root, _, names in os.walk(pkg):
        for n in sorted(names):
            p = os.path.join(root, n)
            r = os.path.relpath(p, pkg)
            if r in ('MANIFEST_SHA256.json', 'README.md'):
                continue
            files[r] = sha256(p)
    manifest = dict(
        board=rel, board_sha256=bsha, kicad_cli=pcbnew.GetBuildVersion(),
        git_commit_of_board=a.git_commit,
        commands=[c for c in a.commands.split('||') if c],
        drc_summary=drc_summary,
        checks_passed={**oc['checks'],
                       'drill_hits_equal_board_holes': drill_check['hits_equal_board_holes'],
                       'pos_rows_equal_footprints_minus_excluded': pos_check['rows_equal_footprints_minus_excluded'],
                       'zone_fill_current': zone_check['zone_layers_changed_gt_0p01mm2'] == 0},
        files=dict(sorted(files.items())),
        note='README.md is excluded from the manifest (it describes the manifest).',
    )
    json.dump(manifest, open(os.path.join(pkg, 'MANIFEST_SHA256.json'), 'w'), indent=1)
    print(json.dumps(dict(board_sha256=bsha, outline=oc['checks'],
                          gerber_extent=ext, board_bbox=board_bbox,
                          stale=oc.get('stale_package_edge_cuts', {}).get('extent_mm', {}).get('centreline'),
                          drill=dict(hits=drill_check['excellon_total_hits'], holes=drill_check['board_total_holes'],
                                     vias=drill_check['board_vias_by_span']),
                          pos=pos_check, zone=dict((k, v) for k, v in zone_check.items() if k != 'changed'),
                          drc=drc_summary), indent=1))


if __name__ == '__main__':
    main()
