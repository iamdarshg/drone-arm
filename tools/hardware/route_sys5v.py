"""Route the SYS_5V trunk on the main board.

Chain-connects the 9 SYS_5V pads with L-shaped 0.5 mm runs. Tries a set of
F.Cu horizontal corridors for the long haul; falls back to B.Cu (with vias)
if no F.Cu corridor clears. Every segment is checked against other-net
F.Cu tracks and pads.

Usage: kicad-python route_sys5v.py <board.kicad_pcb>
"""
import math
import sys
import pcbnew

L_F, L_B = 0, 2
WIDTH = pcbnew.FromMM(0.5)
KEEPOUT = 0.25 + 0.125 + 0.05  # half-width + default clearance + margin (mm)


def find_net(board, netname):
    for code, net in board.GetNetsByNetcode().items():
        if net.GetNetname().split("/")[-1] == netname:
            return net
    raise SystemExit(f"net {netname} not found")


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    ll = dx * dx + dy * dy
    t = 0.0 if ll == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / ll))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def main():
    path = sys.argv[1]
    board = pcbnew.LoadBoard(path)
    net = find_net(board, "SYS_5V")
    ncode = net.GetNetCode()

    pads = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() == ncode:
                pads.append((p.GetPosition().x / 1e6, p.GetPosition().y / 1e6,
                             fp.GetReference(), p.GetNumber()))
    print("SYS_5V pads:", pads)

    # obstacles per layer: other-net tracks, pads, and ALL other-net vias
    obst_by_layer = {L_F: [], L_B: []}
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_TRACK) and not isinstance(t, pcbnew.PCB_VIA):
            if t.GetLayer() in obst_by_layer and t.GetNetCode() != ncode:
                obst_by_layer[t.GetLayer()].append(
                    (t.GetStartX() / 1e6, t.GetStartY() / 1e6,
                     t.GetEndX() / 1e6, t.GetEndY() / 1e6))
        elif isinstance(t, pcbnew.PCB_VIA) and t.GetNetCode() != ncode:
            vx, vy = t.GetPosition().x / 1e6, t.GetPosition().y / 1e6
            for lay in obst_by_layer:
                obst_by_layer[lay].append((vx, vy, vx, vy))
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() != ncode:
                for lay in (L_F, L_B):
                    if p.IsOnLayer(lay):
                        obst_by_layer[lay].append(
                            (p.GetPosition().x / 1e6, p.GetPosition().y / 1e6,
                             p.GetPosition().x / 1e6, p.GetPosition().y / 1e6))

    def clear(x1, y1, x2, y2, y3=None, layer=L_F):
        segs = [(x1, y1, x2, y2)]
        if y3 is not None:
            segs.append((x2, y2, x2, y3))
        for sx1, sy1, sx2, sy2 in segs:
            for ox1, oy1, ox2, oy2 in obst_by_layer[layer]:
                d = seg_dist(ox1, oy1, sx1, sy1, sx2, sy2)
                if ox1 == ox2 and oy1 == oy2:  # pad point
                    if d < KEEPOUT + 0.3:
                        return False
                elif d < KEEPOUT:
                    return False
        return True

    def add_track(x1, y1, x2, y2, layer):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(x1), pcbnew.FromMM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(x2), pcbnew.FromMM(y2)))
        t.SetWidth(WIDTH)
        t.SetLayer(layer)
        t.SetNetCode(ncode)
        board.Add(t)

    def add_via(x, y):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y)))
        v.SetNetCode(ncode)
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetWidth(pcbnew.FromMM(0.6))
        v.SetDrill(pcbnew.FromMM(0.3))
        v.SetLayerPair(L_F, L_B)
        board.Add(v)

    pads_sorted = sorted(pads, key=lambda p: (p[1], p[0]))
    # chain: D2 -> cluster A (bottom-left) -> cluster B (top-right)
    by_ref = {f"{r}:{n}": (x, y) for x, y, r, n in pads}
    chain = []
    for key in ("D2:2", "C1:1", "D1:2", "U2:1", "U2:4", "U4:1", "C7:1", "C3:1", "U3:1"):
        if key in by_ref:
            chain.append(by_ref[key])
    print("chain:", chain)

    edge = board.GetBoardEdgesBoundingBox()
    ytop = edge.GetY() / 1e6 + 3.0
    ybot = (edge.GetY() + edge.GetHeight()) / 1e6 - 3.0

    def via_ok(x, y):
        """through-via barrel crosses every copper layer - check all"""
        for lay in (L_F, L_B, 4, 6):
            for ox1, oy1, ox2, oy2 in obst_by_layer.get(lay, []):
                if ox1 == ox2 and oy1 == oy2:
                    if math.hypot(ox1 - x, oy1 - y) < 0.55:
                        return False
                elif seg_dist(x, y, ox1, oy1, ox2, oy2) < 0.55:
                    return False
        return True

    placed = []
    plan = []
    via_dropped_at = []
    for (ax, ay), (bx, by) in zip(chain, chain[1:]):
        done = False
        for layer in (L_F, L_B):
            cands = []
            cands.append([(ax, ay), (bx, ay), (bx, by)])
            cands.append([(ax, ay), (ax, by), (bx, by)])
            for off in (1.6, 2.5, 3.5, -1.6, -2.5, -3.5):
                cands.append([(ax, ay), (ax + off, ay), (ax + off, by), (bx, by)])
                cands.append([(ax, ay), (bx + off, ay), (bx + off, by), (bx, by)])
                cands.append([(ax, ay), (ax, ay + off), (bx, ay + off), (bx, by)])
                cands.append([(ax, ay), (ax, by + off), (bx, by + off), (bx, by)])
            for cy in (ybot - 2, ybot, ytop, ytop + 2, (ay + by) / 2):
                cands.append([(ax, ay), (ax, cy), (bx, cy), (bx, by)])
            need_vias = (layer == L_B) and via_ok(ax, ay) and via_ok(bx, by)
            if layer == L_B and not need_vias:
                continue
            for cand in cands:
                good = True
                for (wx1, wy1), (wx2, wy2) in zip(cand, cand[1:]):
                    if abs(wx1 - wx2) < 0.01 and abs(wy1 - wy2) < 0.01:
                        continue
                    if not clear(wx1, wy1, wx2, wy2, layer=layer):
                        good = False
                        break
                if good:
                    for (wx1, wy1), (wx2, wy2) in zip(cand, cand[1:]):
                        if abs(wx1 - wx2) > 0.01 or abs(wy1 - wy2) > 0.01:
                            add_track(wx1, wy1, wx2, wy2, layer)
                            obst_by_layer[layer].append((wx1, wy1, wx2, wy2))
                    if layer == L_B:
                        for px, py in ((ax, ay), (bx, by)):
                            if not any(abs(px - ex) < 0.3 and abs(py - ey) < 0.3
                                       for ex, ey in via_dropped_at):
                                add_via(px, py)
                                via_dropped_at.append((px, py))
                    plan.append((cand, layer))
                    done = True
                    break
            if done:
                break
        if not done:
            print(f"  no route {ax,ay}->{bx,by}")
    placed = plan
    print(f"routed {len(plan)} connections")

    pcbnew.SaveBoard(path, board)
    print("saved" if placed else "NOT ROUTED - no plan")


raise SystemExit(main())

