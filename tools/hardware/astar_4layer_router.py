#!/usr/bin/env python3
"""Clearance-screened four-layer bidirectional A* for ESC signal routing."""

from __future__ import annotations

import argparse
import collections
import heapq
import hashlib
import json
import math
import os
import shutil
import subprocess
from pathlib import Path

import pcbnew


HERE = Path(__file__).resolve().parent
BASE: Path
WORK: Path
PROBE: Path
DRC: Path
KICAD_CLI = Path(r"C:\Program Files\KiCad\9.0\bin\kicad-cli.exe")

LAYERS = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
LAYER_NAMES = ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"]
FIDX = 0
GRID = 0.20
WIDTH = 0.25
CLEARANCE = 0.20
VIA_DIAMETER = 0.70
VIA_DRILL = 0.35
VIA_COST = 24.0
TURN_COST = 1.35
SOFT_ZONE_COST = 1.5
HARD_ZONE_COST = 6.0
ESCAPE_LENGTH = 0.80
MAX_EXPANSIONS = 1_200_000
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]

def mm(v): return pcbnew.ToMM(v)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest().upper()


def endpoint_index(board):
    return {
        pad.m_Uuid.AsString(): pad
        for footprint in board.GetFootprints()
        for pad in footprint.Pads()
        if pad.GetNumber()
    }


def validate_contract(board, board_path, contract):
    errors = []
    if contract.get("board_sha256") != sha256(board_path):
        errors.append({"type": "board_sha_mismatch", "contract": contract.get("board_sha256"), "actual": sha256(board_path)})
    if not contract.get("gate_pass"):
        errors.append({"type": "contract_gate_failed"})
    pads = endpoint_index(board)
    for row in contract.get("endpoints", []):
        pad = pads.get(row["pad_uuid"])
        if pad is None:
            errors.append({"type": "missing_pad_uuid", "endpoint": row})
            continue
        footprint = pad.GetParentFootprint()
        actual = (footprint.GetReference(), pad.GetNumber(), pad.GetNetname(), pad.GetNetCode())
        expected = (row["ref"], row["pad"], row["net"], row["net_code"])
        if actual != expected:
            errors.append({"type": "endpoint_identity_mismatch", "pad_uuid": row["pad_uuid"], "expected": expected, "actual": actual})
    if errors:
        raise RuntimeError(json.dumps(errors[:20], indent=2))
    return pads


def open_count(board):
    board.BuildConnectivity()
    return board.GetConnectivity().GetUnconnectedCount(False)


def two_pads(board, net_name):
    pads = [p for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetname() == net_name]
    if len(pads) != 2:
        raise RuntimeError(f"{net_name}: expected two pads, got {len(pads)}")
    return pads


def point_segment_distance(px, py, ax, ay, bx, by):
    vx, vy, wx, wy = bx - ax, by - ay, px - ax, py - ay
    vv = vx * vx + vy * vy
    if vv == 0: return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


class Geometry:
    def __init__(self, board, net_name, start, goal, margin):
        self.board, self.net_name = board, net_name
        self.net_code = board.FindNet(net_name).GetNetCode()
        edge = board.GetBoardEdgesBoundingBox()
        self.x0 = max(min(start[0], goal[0]) - margin, mm(edge.GetX()) + 0.5)
        self.y0 = max(min(start[1], goal[1]) - margin, mm(edge.GetY()) + 0.5)
        self.x1 = min(max(start[0], goal[0]) + margin, mm(edge.GetRight()) - 0.5)
        self.y1 = min(max(start[1], goal[1]) + margin, mm(edge.GetBottom()) - 0.5)
        self.start, self.goal = start, goal
        self.escape_rays = []
        for fp in board.GetFootprints():
            for p in fp.Pads():
                if p.GetNetCode() != self.net_code: continue
                q, c = p.GetPosition(), fp.GetPosition()
                px, py, cx, cy = mm(q.x), mm(q.y), mm(c.x), mm(c.y)
                dx, dy = px-cx, py-cy
                if abs(dx) >= abs(dy): ux, uy = (1.0 if dx >= 0 else -1.0), 0.0
                else: ux, uy = 0.0, (1.0 if dy >= 0 else -1.0)
                # Stay just beyond the package pad edge.  The previous fixed
                # 1.35 mm stub could cross an already-routed adjacent signal
                # before A* obstacle checking began (the DRV_SCLK/DRV_SDI
                # pair is the concrete failure).  A short ray still clears the
                # longest local pads while handing direction choice to A*.
                self.escape_rays.append((px, py, px+ux*ESCAPE_LENGTH, py+uy*ESCAPE_LENGTH))
        self.pads = [[] for _ in LAYERS]
        self.tracks = [[] for _ in LAYERS]
        self.zones_hard = [[] for _ in LAYERS]
        self.zones_soft = [[] for _ in LAYERS]
        trace_expand = WIDTH / 2 + CLEARANCE

        for fp in board.GetFootprints():
            for p in fp.Pads():
                if p.GetNetCode() == self.net_code: continue
                bb = p.GetBoundingBox()
                box = (mm(bb.GetX()) - trace_expand, mm(bb.GetY()) - trace_expand,
                       mm(bb.GetRight()) + trace_expand, mm(bb.GetBottom()) + trace_expand)
                if not self.box_near(box): continue
                for li, layer in enumerate(LAYERS):
                    if p.IsOnLayer(layer): self.pads[li].append((box, p))

        self.vias = []
        self.via_track_obstacles = []
        for t in board.GetTracks():
            if t.GetNetCode() == self.net_code: continue
            if isinstance(t, pcbnew.PCB_VIA):
                p = t.GetPosition(); r = mm(t.GetWidth(pcbnew.F_Cu)) / 2 + WIDTH / 2 + CLEARANCE
                self.vias.append((mm(p.x), mm(p.y), r))
            else:
                a, b = t.GetStart(), t.GetEnd()
                ax, ay, bx, by = mm(a.x), mm(a.y), mm(b.x), mm(b.y)
                tr = mm(t.GetWidth()) / 2 + WIDTH / 2 + CLEARANCE
                vr = mm(t.GetWidth()) / 2 + VIA_DIAMETER / 2 + CLEARANCE
                li = LAYERS.index(t.GetLayer()) if t.GetLayer() in LAYERS else None
                if li is not None: self.tracks[li].append((ax, ay, bx, by, tr))
                self.via_track_obstacles.append((ax, ay, bx, by, vr, t.GetLayer()))

        for z in board.Zones():
            if z.GetNetCode() == self.net_code: continue
            name = z.GetNetname()
            hard = any(token in name for token in ("BATP", "BATN", "PHASE_", "_SHA_I", "_SHB_I", "_SHC_I"))
            bb = z.GetBoundingBox()
            box = (mm(bb.GetX()), mm(bb.GetY()), mm(bb.GetRight()), mm(bb.GetBottom()))
            if not self.box_near(box): continue
            for li, layer in enumerate(LAYERS):
                if z.IsOnLayer(layer) and z.HasFilledPolysForLayer(layer):
                    (self.zones_hard if hard else self.zones_soft)[li].append((box, z))

        self.protected = []
        for row in range(3):
            for col in range(2):
                dx, dy = 85.0 * col, 82.5 * row
                self.protected += [
                    (dx + 3.0, dy + 108.55, dx + 77.65, dy + 125.75),
                    (dx + 44.45, dy + 87.50, dx + 54.45, dy + 106.90),
                ]
        self.block_cache, self.cost_cache, self.via_cache = {}, {}, {}

    def box_near(self, b):
        return not (b[2] < self.x0 or b[0] > self.x1 or b[3] < self.y0 or b[1] > self.y1)

    def to_xyz(self, node):
        return self.x0 + node[0] * GRID, self.y0 + node[1] * GRID, node[2]

    def to_node(self, p, li=FIDX):
        return round((p[0] - self.x0) / GRID), round((p[1] - self.y0) / GRID), li

    def in_bounds(self, x, y): return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1

    def zone_hit(self, entry, layer, x, y, accuracy=0):
        box, z = entry
        if not (box[0] - 0.4 <= x <= box[2] + 0.4 and box[1] - 0.4 <= y <= box[3] + 0.4): return False
        return z.HitTestFilledArea(layer, pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)), accuracy)

    def blocked(self, node):
        if node in self.block_cache: return self.block_cache[node]
        x, y, li = self.to_xyz(node)
        if not self.in_bounds(x, y): return True
        # Keep the known current corridors free on all routing layers.
        for x0, y0, x1, y1 in self.protected:
            if x0 - WIDTH/2 <= x <= x1 + WIDTH/2 and y0 - WIDTH/2 <= y <= y1 + WIDTH/2:
                self.block_cache[node] = True; return True
        for box, _ in self.pads[li]:
            if box[0] <= x <= box[2] and box[1] <= y <= box[3]:
                self.block_cache[node] = True; return True
        for vx, vy, r in self.vias:
            if (x-vx)**2 + (y-vy)**2 <= r*r:
                self.block_cache[node] = True; return True
        for ax, ay, bx, by, r in self.tracks[li]:
            if point_segment_distance(x, y, ax, ay, bx, by) <= r:
                self.block_cache[node] = True; return True
        self.block_cache[node] = False
        return False

    def direct_clear(self, start, end, li):
        """Check the complete pad-to-grid stub, not only its final node."""
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        samples = max(2, math.ceil(length / 0.04))
        for index in range(samples + 1):
            t = index / samples
            x = start[0] + (end[0] - start[0]) * t
            y = start[1] + (end[1] - start[1]) * t
            for x0, y0, x1, y1 in self.protected:
                if x0 - WIDTH/2 <= x <= x1 + WIDTH/2 and y0 - WIDTH/2 <= y <= y1 + WIDTH/2:
                    return False
            for box, _ in self.pads[li]:
                if box[0] <= x <= box[2] and box[1] <= y <= box[3]:
                    return False
            for vx, vy, r in self.vias:
                if (x-vx)**2 + (y-vy)**2 <= r*r:
                    return False
            for ax, ay, bx, by, r in self.tracks[li]:
                if point_segment_distance(x, y, ax, ay, bx, by) <= r:
                    return False
        return True

    def safe_escape(self, pad_point, nominal, li=FIDX):
        """Find a nearby grid node whose complete entry stub is clearance-safe."""
        base = self.to_node(nominal, li)
        choices = []
        for dx in range(-6, 7):
            for dy in range(-6, 7):
                node = (base[0] + dx, base[1] + dy, li)
                if self.blocked(node):
                    continue
                x, y, _ = self.to_xyz(node)
                distance_from_pad = math.hypot(x-pad_point[0], y-pad_point[1])
                if distance_from_pad < 0.55:
                    continue
                if not self.direct_clear(pad_point, (x, y), li):
                    continue
                score = math.hypot(x-nominal[0], y-nominal[1]) + 0.03 * distance_from_pad
                choices.append((score, node, (x, y)))
        if not choices:
            raise RuntimeError(f"no clearance-safe pad escape near {pad_point}")
        _, node, point = min(choices)
        return point, node

    def penalty(self, node):
        if node in self.cost_cache: return self.cost_cache[node]
        x, y, li = self.to_xyz(node); layer = LAYERS[li]
        val = 0.0
        for entry in self.zones_hard[li]:
            if self.zone_hit(entry, layer, x, y): val = HARD_ZONE_COST; break
        for entry in self.zones_soft[li]:
            if self.zone_hit(entry, layer, x, y): val = max(val, SOFT_ZONE_COST); break
        self.cost_cache[node] = val
        return val

    def via_clear(self, node):
        key = node[:2]
        if key in self.via_cache: return self.via_cache[key]
        x, y, _ = self.to_xyz(node)
        if min(math.hypot(x-self.start[0], y-self.start[1]), math.hypot(x-self.goal[0], y-self.goal[1])) < 1.0:
            self.via_cache[key] = False; return False
        vr = VIA_DIAMETER/2 + CLEARANCE
        for li in range(len(LAYERS)):
            for box, _ in self.pads[li]:
                # The trace-expanded pad box needs another via-vs-trace radius increment.
                extra = (VIA_DIAMETER-WIDTH)/2
                if box[0]-extra <= x <= box[2]+extra and box[1]-extra <= y <= box[3]+extra:
                    self.via_cache[key] = False; return False
        for vx, vy, r in self.vias:
            if math.hypot(x-vx, y-vy) <= r + (VIA_DIAMETER-WIDTH)/2:
                self.via_cache[key] = False; return False
        for ax, ay, bx, by, r, _ in self.via_track_obstacles:
            if point_segment_distance(x, y, ax, ay, bx, by) <= r:
                self.via_cache[key] = False; return False
        # Zones create antipads around the via; the full refill plus current-net audit
        # decides whether that local hole is acceptable.
        self.via_cache[key] = True; return True


def octile(a, b):
    dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
    return GRID*(max(dx,dy)+(math.sqrt(2)-1)*min(dx,dy)) + (0 if a[2] == b[2] else VIA_COST)


def route(geo, start, goal):
    roots = [(start[0],start[1],start[2],8),(goal[0],goal[1],goal[2],8)]
    targets=[goal,start]
    heaps=[[(octile(start,goal),0.0,roots[0])],[(octile(goal,start),0.0,roots[1])]]
    gs=[{roots[0]:0.0},{roots[1]:0.0}]; parents=[{roots[0]:None},{roots[1]:None}]
    best=[{start:roots[0]},{goal:roots[1]}]; expanded=0; meet=None
    while heaps[0] and heaps[1] and expanded < MAX_EXPANSIONS:
        side = 0 if heaps[0][0][0] <= heaps[1][0][0] else 1
        _,g,state=heapq.heappop(heaps[side])
        if g != gs[side].get(state): continue
        pos=state[:3]
        if pos in best[1-side]:
            meet=(state,best[1-side][pos]) if side==0 else (best[1-side][pos],state); break
        expanded += 1; prev=state[3]
        for nd,(dx,dy) in enumerate(DIRS):
            nxt=(pos[0]+dx,pos[1]+dy,pos[2])
            if nxt not in (start,goal) and geo.blocked(nxt): continue
            step=GRID*(math.sqrt(2) if dx and dy else 1.0)
            turn=0 if prev in (8,nd) else TURN_COST*min((prev-nd)%8,(nd-prev)%8)
            ng=g+step+turn+geo.penalty(nxt); ns=(*nxt,nd)
            if ng+1e-9 >= gs[side].get(ns,math.inf): continue
            gs[side][ns]=ng;parents[side][ns]=state
            old=best[side].get(nxt)
            if old is None or ng < gs[side].get(old,math.inf): best[side][nxt]=ns
            heapq.heappush(heaps[side],(ng+octile(nxt,targets[side]),ng,ns))
        if geo.via_clear(pos):
            for li in range(len(LAYERS)):
                if li == pos[2]: continue
                nxt=(pos[0],pos[1],li)
                if geo.blocked(nxt): continue
                ng=g+VIA_COST+geo.penalty(nxt);ns=(*nxt,8)
                if ng+1e-9 >= gs[side].get(ns,math.inf): continue
                gs[side][ns]=ng;parents[side][ns]=state
                old=best[side].get(nxt)
                if old is None or ng < gs[side].get(old,math.inf):best[side][nxt]=ns
                heapq.heappush(heaps[side],(ng+octile(nxt,targets[side]),ng,ns))
    if meet is None:return None,expanded
    sf,sb=meet;left=[];cur=sf
    while cur is not None:left.append(cur[:3]);cur=parents[0][cur]
    left.reverse();right=[];cur=parents[1][sb]
    while cur is not None:right.append(cur[:3]);cur=parents[1][cur]
    return left+right,expanded


def compress(points):
    if len(points)<=2:return points
    out=[points[0]];prev=None
    for i in range(1,len(points)):
        if points[i][2] != points[i-1][2]:
            out.append(points[i-1]);out.append(points[i]);prev=None;continue
        dx=points[i][0]-points[i-1][0];dy=points[i][1]-points[i-1][1]
        d=(0 if abs(dx)<1e-8 else int(math.copysign(1,dx)),0 if abs(dy)<1e-8 else int(math.copysign(1,dy)))
        if prev is not None and d != prev:out.append(points[i-1])
        prev=d
    if out[-1] != points[-1]:out.append(points[-1])
    # Consecutive duplicates can arise at a layer transition.
    clean=[out[0]]
    for p in out[1:]:
        if p != clean[-1]:clean.append(p)
    return clean


def add_path(board, net_name, pa, pb, escape_a, escape_b, geo, nodes):
    a=pa.GetPosition();b=pb.GetPosition()
    pts=[(mm(a.x),mm(a.y),FIDX),(*escape_a,FIDX)]+[geo.to_xyz(n) for n in nodes]+[(*escape_b,FIDX),(mm(b.x),mm(b.y),FIDX)]
    clean=[pts[0]]
    for p in pts[1:]:
        if p != clean[-1]:clean.append(p)
    pts=compress(clean);net=board.FindNet(net_name);vias=[];segments=0
    for p,q in zip(pts,pts[1:]):
        if p[2] != q[2]:
            if abs(p[0]-q[0])>1e-6 or abs(p[1]-q[1])>1e-6:raise RuntimeError("layer change moved in XY")
            key=(round(p[0],6),round(p[1],6))
            if key not in vias:
                v=pcbnew.PCB_VIA(board);v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(p[0]),pcbnew.FromMM(p[1])))
                v.SetWidth(pcbnew.FromMM(VIA_DIAMETER));v.SetDrill(pcbnew.FromMM(VIA_DRILL));v.SetLayerPair(pcbnew.F_Cu,pcbnew.B_Cu);v.SetNet(net);board.Add(v);vias.append(key)
        else:
            t=pcbnew.PCB_TRACK(board);t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(p[0]),pcbnew.FromMM(p[1])));t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(q[0]),pcbnew.FromMM(q[1])))
            t.SetLayer(LAYERS[p[2]]);t.SetWidth(pcbnew.FromMM(WIDTH));t.SetNet(net);board.Add(t);segments+=1
    return pts,segments,len(vias)


def drc_errors(path):
    cp=subprocess.run([str(KICAD_CLI),"pcb","drc","--format","json","--all-track-errors","--exit-code-violations","-o",str(DRC),str(path)],capture_output=True,text=True)
    d=json.loads(DRC.read_text(encoding="utf-8"));errors=[v for v in d["violations"] if v.get("severity")=="error"]
    return errors,cp.returncode


def pair_connected(board, pa, pb):
    connectivity = board.GetConnectivity()
    target = pb.m_Uuid.AsString()
    queue = list(connectivity.GetConnectedTracks(pa))
    seen = set()
    while queue:
        item = queue.pop()
        uuid = item.m_Uuid.AsString()
        if uuid in seen:
            continue
        seen.add(uuid)
        if any(pad.m_Uuid.AsString() == target for pad in connectivity.GetConnectedPads(item)):
            return True
        queue.extend(connectivity.GetConnectedTracks(item))
    return False


def duplicate_vias(board):
    positions = collections.Counter(
        (item.GetPosition().x, item.GetPosition().y)
        for item in board.GetTracks()
        if isinstance(item, pcbnew.PCB_VIA)
    )
    return sum(count - 1 for count in positions.values() if count > 1)


def configure_net_geometry(board, net_name):
    global WIDTH, CLEARANCE, VIA_DIAMETER, VIA_DRILL
    netclass = board.GetDesignSettings().m_NetSettings.GetEffectiveNetClass(net_name)
    WIDTH = max(0.20, mm(netclass.GetTrackWidth()))
    CLEARANCE = max(0.20, mm(netclass.GetClearance()))
    VIA_DIAMETER = max(0.60, mm(netclass.GetViaDiameter()))
    VIA_DRILL = max(0.30, mm(netclass.GetViaDrill()))
    return netclass.GetName()


def main():
    global BASE, WORK, PROBE, DRC, MAX_EXPANSIONS
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--project", required=True, type=Path)
    parser.add_argument("--contract", required=True, type=Path)
    parser.add_argument("--work-dir", required=True, type=Path)
    parser.add_argument("--accept", type=int, default=10)
    parser.add_argument("--max-attempts", type=int, default=240)
    parser.add_argument("--max-expansions", type=int, default=1_200_000)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    BASE = args.board.resolve()
    args.work_dir.mkdir(parents=True, exist_ok=True)
    WORK = args.work_dir / "working.kicad_pcb"
    PROBE = args.work_dir / "probe.kicad_pcb"
    DRC = args.work_dir / "probe_drc.json"
    results_path = args.work_dir / "route_results.json"
    state_path = args.work_dir / "route_state.json"
    MAX_EXPANSIONS = args.max_expansions

    contract = json.loads(args.contract.read_text(encoding="utf-8"))
    base_board = pcbnew.LoadBoard(str(BASE))
    validate_contract(base_board, BASE, contract)
    baseline_footprints = {
        fp.GetReference(): (
            fp.GetPosition().x,
            fp.GetPosition().y,
            fp.GetOrientation().AsTenthsOfADegree(),
            fp.GetLayer(),
        )
        for fp in base_board.GetFootprints()
    }
    if not (args.resume and WORK.exists()):
        shutil.copy2(BASE, WORK)
        results = []
    else:
        results = json.loads(results_path.read_text(encoding="utf-8")) if results_path.exists() else []
    shutil.copy2(args.project, WORK.with_suffix(".kicad_pro"))
    shutil.copy2(args.project, PROBE.with_suffix(".kicad_pro"))

    board = pcbnew.LoadBoard(str(WORK))
    board.BuildConnectivity()
    live_endpoints = endpoint_index(board)
    zone_nets = {zone.GetNetname() for zone in board.Zones()}
    candidates = []
    for row in contract["nets_in_density_priority_order"]:
        if row["endpoint_count"] != 2 or row["net"] in zone_nets:
            continue
        pads = [live_endpoints.get(uuid) for uuid in row["endpoint_uuids"]]
        if any(pad is None for pad in pads):
            raise RuntimeError(f"missing live endpoint for {row['net']}")
        netclass = board.GetDesignSettings().m_NetSettings.GetEffectiveNetClass(row["net"])
        class_name = str(netclass.GetName())
        # Preserve signal-first routing while still leaving every two-pad net in
        # the queue. Wide power classes naturally follow the signal candidates.
        class_priority = 1 if class_name in ("HIGH_CURRENT", "POWER") else 0
        priority = row["priority"]
        candidates.append((class_priority, -priority["density_sum"], priority["centrality_sum_mm"], row["net"], row["endpoint_uuids"]))
    candidates.sort()

    initial_opens = open_count(board)
    accepted_total = sum(bool(row.get("accepted")) for row in results)
    attempted = 0
    print(json.dumps({"initial_true_opens": initial_opens, "candidate_nets": len(candidates), "already_accepted": accepted_total}), flush=True)
    for _, density_key, centrality, net_name, endpoint_uuids in candidates:
        if accepted_total >= args.accept or attempted >= args.max_attempts:
            break
        board = pcbnew.LoadBoard(str(WORK))
        board.BuildConnectivity()
        pads = endpoint_index(board)
        pa, pb = (pads[endpoint_uuids[0]], pads[endpoint_uuids[1]])
        if pa.GetNetname() != net_name or pb.GetNetname() != net_name or pa.GetNetCode() != pb.GetNetCode():
            raise RuntimeError(f"endpoint identity changed for {net_name}")
        if pair_connected(board, pa, pb):
            continue
        attempted += 1
        before = open_count(board)
        class_name = str(configure_net_geometry(board, net_name))
        a = (mm(pa.GetPosition().x), mm(pa.GetPosition().y))
        b = (mm(pb.GetPosition().x), mm(pb.GetPosition().y))
        distance = math.hypot(a[0] - b[0], a[1] - b[1])
        try:
            geo = Geometry(board, net_name, a, b, max(10.0, min(42.0, distance)))
            def escape_for(point):
                ray = min(geo.escape_rays, key=lambda value: math.hypot(value[0] - point[0], value[1] - point[1]))
                return geo.safe_escape(point, (ray[2], ray[3]))
            (ea, start), (eb, goal) = escape_for(a), escape_for(b)
            nodes, expanded = route(geo, start, goal)
            if nodes is None:
                row = {"net": net_name, "accepted": False, "reason": "no_path", "expanded": expanded, "netclass": class_name}
                results.append(row)
                results_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
                state_path.write_text(json.dumps({"status": "running", "initial_true_opens": initial_opens, "candidate_nets": len(candidates), "attempted": attempted, "accepted_total": accepted_total, "true_opens": before, "last_net": net_name, "last_accepted": False}, indent=2) + "\n", encoding="utf-8")
                print(json.dumps(row), flush=True)
                continue
            points, segments, vias = add_path(board, net_name, pa, pb, ea, eb, geo, nodes)
            pcbnew.ZONE_FILLER(board).Fill(board.Zones())
            pcbnew.SaveBoard(str(PROBE), board)
            probe_board = pcbnew.LoadBoard(str(PROBE))
            after = open_count(probe_board)
            errors, drc_rc = drc_errors(PROBE)
            duplicates = duplicate_vias(probe_board)
            transforms = {
                fp.GetReference(): (
                    fp.GetPosition().x,
                    fp.GetPosition().y,
                    fp.GetOrientation().AsTenthsOfADegree(),
                    fp.GetLayer(),
                )
                for fp in probe_board.GetFootprints()
            }
            accepted = after < before and not errors and duplicates == 0 and transforms == baseline_footprints
            row = {
                "net": net_name,
                "endpoint_uuids": endpoint_uuids,
                "accepted": accepted,
                "before": before,
                "after": after,
                "expanded": expanded,
                "segments": segments,
                "vias": vias,
                "layers": sorted({LAYER_NAMES[point[2]] for point in points}),
                "width_mm": WIDTH,
                "clearance_mm": CLEARANCE,
                "netclass": class_name,
                "density_sum": -density_key,
                "centrality_sum_mm": centrality,
                "drc_errors": len(errors),
                "drc_types": sorted({error.get("type") for error in errors}),
                "drc_rc": drc_rc,
                "duplicate_vias": duplicates,
            }
            if accepted:
                shutil.copy2(PROBE, WORK)
                accepted_total += 1
            results.append(row)
            results_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
            state_path.write_text(json.dumps({"status": "running", "initial_true_opens": initial_opens, "candidate_nets": len(candidates), "attempted": attempted, "accepted_total": accepted_total, "true_opens": after if accepted else before, "last_net": net_name, "last_accepted": accepted}, indent=2) + "\n", encoding="utf-8")
            print(json.dumps(row), flush=True)
        except Exception as error:
            row = {"net": net_name, "accepted": False, "reason": "exception", "error": repr(error), "netclass": class_name}
            results.append(row)
            results_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
            state_path.write_text(json.dumps({"status": "running", "initial_true_opens": initial_opens, "candidate_nets": len(candidates), "attempted": attempted, "accepted_total": accepted_total, "true_opens": before, "last_net": net_name, "last_accepted": False}, indent=2) + "\n", encoding="utf-8")
            print(json.dumps(row), flush=True)

    final_board = pcbnew.LoadBoard(str(WORK))
    final_opens = open_count(final_board)
    status = "checkpoint" if accepted_total >= args.accept else "attempt_limit"
    state = {"status": status, "initial_true_opens": initial_opens, "candidate_nets": len(candidates), "attempted": attempted, "accepted_total": accepted_total, "true_opens": final_opens, "board_sha256": sha256(WORK)}
    state_path.write_text(json.dumps(state, indent=2) + "\n", encoding="utf-8")
    results_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(state, indent=2), flush=True)
    return 0 if accepted_total >= args.accept else 3


if __name__=="__main__":
    raise SystemExit(main())

