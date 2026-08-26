"""Main Rev-B power rebuild v2.

- De-duplicate the generator's twin B.Cu GND zones (keep one).
- Add F.Cu GND pour (pads are topside SMD; B.Cu pour alone can't reach them).
- Add In1.Cu 3V3 pour and In2.Cu 1V1 pour.
- Drop 0.6/0.3 vias at SMD pads of GND/3V3/1V1, skipping any pad whose
  barrel would pierce other-net copper (tracks on any layer, PTH pads).

Usage: kicad-python main_power_rebuild.py <board.kicad_pcb>
"""
import math
import sys
import pcbnew

# KiCad 9 layer ids: F_Cu=0, B_Cu=2, In1_Cu=4, In2_Cu=6
L_F, L_B, L_In1, L_In2 = 0, 2, 4, 6
VIA_D, VIA_DRILL = 0.6, 0.3


def find_net(board, netname):
    target = netname.lstrip("/")
    for code, net in board.GetNetsByNetcode().items():
        if net.GetNetname().split("/")[-1] == target:
            return net
    raise SystemExit(f"net not found: {netname}")


def rect_zone(board, netname, layer, pts, prio, conn):
    z = pcbnew.ZONE(board)
    z.SetNetCode(find_net(board, netname).GetNetCode())
    z.SetLayer(layer)
    z.SetAssignedPriority(prio)
    z.SetLocalClearance(pcbnew.FromMM(0.25))
    z.SetMinThickness(pcbnew.FromMM(0.25))
    z.SetThermalReliefGap(pcbnew.FromMM(0.4))
    z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.35))
    z.SetPadConnection(conn)
    poly = z.Outline()
    poly.NewOutline()
    for (x, y) in pts:
        poly.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
    board.Add(z)
    return z


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    ll = dx * dx + dy * dy
    t = 0.0 if ll == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / ll))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def main():
    path = sys.argv[1]
    board = pcbnew.LoadBoard(path)
    edge = board.GetBoardEdgesBoundingBox()
    x0, y0 = edge.GetX() / 1e6, edge.GetY() / 1e6
    x1, y1 = x0 + edge.GetWidth() / 1e6, y0 + edge.GetHeight() / 1e6
    rect = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

    # 1. de-dup B.Cu GND zones (generator left two identical ones)
    gnd_bc = [z for z in board.Zones()
              if z.GetNetname().split("/")[-1] == "GND" and z.GetLayer() == L_B]
    for z in gnd_bc[1:]:
        board.Remove(z)
    print(f"B.Cu GND zones: {len(gnd_bc)} -> {min(len(gnd_bc), 1)}")

    # 2. F.Cu GND pour
    rect_zone(board, "GND", L_F, rect, prio=0, conn=pcbnew.ZONE_CONNECTION_THERMAL)
    # 3. In1 3V3 pour, In2 1V1 pour (solid into planes)
    rect_zone(board, "3V3", L_In1, rect, prio=1, conn=pcbnew.ZONE_CONNECTION_FULL)
    vpad = [p.GetPosition() for p in board.GetPads()
            if p.GetNetname().split("/")[-1] == "1V1"]
    if vpad:
        xs = [p.x / 1e6 for p in vpad]
        ys = [p.y / 1e6 for p in vpad]
        m = 5.0
        rect_zone(board, "1V1", L_In2,
                  [(min(xs) - m, min(ys) - m), (max(xs) + m, min(ys) - m),
                   (max(xs) + m, max(ys) + m), (min(xs) - m, max(ys) + m)],
                  prio=1, conn=pcbnew.ZONE_CONNECTION_FULL)

    # 4. via drops, obstacle-checked
    tracks = [(t.GetStartX() / 1e6, t.GetStartY() / 1e6,
               t.GetEndX() / 1e6, t.GetEndY() / 1e6,
               t.GetNetCode(), t.GetLayer()) for t in board.GetTracks()
              if isinstance(t, pcbnew.PCB_TRACK) and not isinstance(t, pcbnew.PCB_VIA)]
    vias = [t.GetPosition() for t in board.GetTracks() if isinstance(t, pcbnew.PCB_VIA)]
    pads_all = [(p.GetPosition().x / 1e6, p.GetPosition().y / 1e6,
                 p.GetNetCode(), p.GetAttribute()) for p in board.GetPads()]

    keepout = VIA_D / 2 + 0.25 + 0.05   # barrel radius + clearance + margin
    dropped = skipped = 0
    for p in board.GetPads():
        net = p.GetNetname().split("/")[-1]
        if net not in ("GND", "3V3", "1V1"):
            continue
        if p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
            continue
        if not p.IsOnLayer(L_F):
            continue
        px, py = p.GetPosition().x / 1e6, p.GetPosition().y / 1e6
        ncode = p.GetNetCode()
        if any(abs(vx / 1e6 - px) < 0.45 and abs(vy / 1e6 - py) < 0.45 for vx, vy in vias):
            skipped += 1
            continue
        # other-net pads too close (incl. PTH in thermal pads)
        if any((ox != px or oy != py) and ncode != onc
               and abs(ox - px) < 0.55 and abs(oy - py) < 0.55
               for ox, oy, onc, _attr in pads_all):
            skipped += 1
            continue
        # other-net tracks on any copper layer crossing the barrel
        blocked = False
        for ax, ay, bx, by, tnc, _tl in tracks:
            if tnc == ncode:
                continue
            if seg_dist(px, py, ax, ay, bx, by) < keepout:
                blocked = True
                break
        if blocked:
            skipped += 1
            continue
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(p.GetPosition())
        v.SetNetCode(ncode)
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetWidth(pcbnew.FromMM(VIA_D))
        v.SetDrill(pcbnew.FromMM(VIA_DRILL))
        v.SetLayerPair(L_F, L_B)
        board.Add(v)
        vias.append(v.GetPosition())
        dropped += 1
    print(f"vias dropped {dropped}, skipped {skipped}")

    pcbnew.SaveBoard(path, board)
    print("saved")


raise SystemExit(main())

