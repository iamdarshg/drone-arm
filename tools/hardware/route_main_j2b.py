
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_can5v = board.FindNet("/CAN_5V")
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0

    # remove the colliding verticals at x=4.23 (both nets) and the GND riser
    # crossing the small GND track near x=21.6
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if t.GetNetname() == "/CAN_5V":
            if abs(sx - 4.23) < 0.05 and abs(ex - 4.23) < 0.05 and abs(sy - 16.38) < 0.1:
                doomed.append(t)
            if abs(sy - 27.0) < 0.05 and abs(ey - 27.0) < 0.05:
                doomed.append(t)
            if abs(sx - 9.68) < 0.05 and abs(ex - 9.68) < 0.05:
                doomed.append(t)
        if t.GetNetname() == "/CAN_GND":
            if abs(sx - 4.23) < 0.05 and abs(ex - 4.23) < 0.05 and abs(sy - 15.12) < 0.1:
                doomed.append(t)
            if abs(sy - 29.5) < 0.05 and abs(ey - 29.5) < 0.05:
                doomed.append(t)
            if abs(sx - 21.62) < 0.05 and abs(ex - 21.62) < 0.05 and abs(sy - 29.5) < 0.1:
                doomed.append(t)
    seen = set(); uniq = []
    for t in doomed:
        if id(t) not in seen:
            seen.add(id(t)); uniq.append(t)
    for t in uniq:
        board.Remove(t); removed += 1

    def seg(net, x1, y1, x2, y2, w, layer=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w))
        t.SetLayer(layer)
        t.SetNet(net)
        board.Add(t)

    def via(x, y, net):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        v.SetWidth(MM(0.8))
        v.SetDrill(MM(0.4))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(net)
        board.Add(v)

    # CAN_5V: J2 pad1 -> LEFT margin x=2.5? Edge at x=0; keep >=0.6: x=1.5.
    # Down x=1.5 to y=26.4, right to F1 pad x=9.68 at y=25.5: L with riser.
    seg(net_can5v, 4.23, 16.38, 1.5, 16.38, 0.45); added += 1
    seg(net_can5v, 1.5, 16.38, 1.5, 25.5, 0.45); added += 1
    seg(net_can5v, 1.5, 25.5, 9.68, 25.5, 0.45); added += 1

    # CAN_GND: J2 pad2 down x=4.23 to y=30.8 (below CAN_5V turn at 27),
    # right to x=21.62... but must not cross CAN_5V horizontal at y=25.5-27?
    # Our path: vertical x=4.23 from 15.12 to 30.8 crosses CAN_5V horiz
    # (y=27, x=4.23..9.68) AT x=4.23! Conflict. Use B.Cu hop instead:
    seg(net_cangnd, 4.23, 15.12, 5.6, 15.12, 0.45); added += 1
    via(5.6, 15.12, net_cangnd)
    seg(net_cangnd, 5.6, 15.12, 5.6, 29.3, 0.45, pcbnew.B_Cu); added += 1
    seg(net_cangnd, 5.6, 29.3, 21.62, 29.3, 0.45, pcbnew.B_Cu); added += 1
    via(21.62, 29.3, net_cangnd)
    seg(net_cangnd, 21.62, 29.3, 21.62, 25.5, 0.45); added += 1

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

