
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_can5v = board.FindNet("/CAN_5V")
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0

    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() not in ("/CAN_5V", "/CAN_GND"):
            continue
        if type(t).__name__ == "PCB_TRACK" and t.IsOnLayer(pcbnew.B_Cu):
            doomed.append(t)
        elif type(t).__name__ == "PCB_TRACK":
            s, e = t.GetStart(), t.GetEnd()
            sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
            # my F.Cu additions: the J2->R3 gnd run and F1 stub
            if abs(ey - 25.5) < 0.01 and abs(sy - 25.5) < 0.01 and (abs(ex - 21.62) < 0.05 or abs(ex - 10.8) < 0.05):
                doomed.append(t)
        elif type(t).__name__ == "PCB_VIA":
            p = t.GetPosition()
            x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            if abs(x - 10.8) < 0.05 and abs(y - 25.5) < 0.05:
                doomed.append(t)
            if abs(x - 80.54) < 0.05 and abs(y - 63.0) < 0.05:
                doomed.append(t)
    seen = set(); uniq = []
    for t in doomed:
        if id(t) not in seen:
            seen.add(id(t)); uniq.append(t)
    for t in uniq:
        board.Remove(t); removed += 1

    # scan for a clear B.Cu horizontal corridor: try y=60
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

    # check B.Cu obstacles at y band 59-61
    blocked_y60 = False
    for t in board.GetTracks():
        if not t.IsOnLayer(pcbnew.B_Cu) or t.GetNetname() == "/CAN_5V":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sy, ey = pcbnew.ToMM(s.y), pcbnew.ToMM(e.y)
        p = t.GetPosition()
        py = pcbnew.ToMM(p.y)
        if type(t).__name__ == "PCB_VIA":
            if 58.6 < py < 61.4:
                blocked_y60 = True
                print("blocker via", t.GetNetname(), round(py,2))
        else:
            if min(sy,ey) < 60.7 and max(sy,ey) > 59.3:
                blocked_y60 = True
    print("y=60 corridor blocked?", blocked_y60)

    if not blocked_y60:
        seg(net_can5v, 9.68, 25.5, 9.0, 25.5, 0.45); added += 1
        via(9.0, 25.5, net_can5v)
        seg(net_can5v, 9.0, 25.5, 9.0, 60.0, 0.45, pcbnew.B_Cu); added += 1
        seg(net_can5v, 9.0, 60.0, 80.54, 60.0, 0.45, pcbnew.B_Cu); added += 1
        via(80.54, 60.0, net_can5v)
        seg(net_can5v, 80.54, 60.0, 80.54, 72.5, 0.45); added += 1

    # CAN_GND: route R3 to In2 CAN track on In2 layer instead of F.Cu
    # via below-left of R3 at (19.5, 27.5) then In2 run left/up to join.
    seg(net_cangnd, 21.62, 25.5, 19.5, 27.62, 0.45); added += 1
    via(19.5, 27.62, net_cangnd)
    # In2: from (19.5,27.62) go left to x=4.23? J2 pad2 is SMD F.Cu though.
    # The existing In2 track connects J2 already; we need to reach it.
    # Run In2 left to x=4.23, then via up, F.Cu to J2 pad2 (4.23,15.12).
    seg(net_cangnd, 19.5, 27.62, 4.23, 27.62, 0.45, pcbnew.In2_Cu); added += 1
    seg(net_cangnd, 4.23, 27.62, 4.23, 15.12, 0.45, pcbnew.In2_Cu); added += 1
    via(4.23, 15.12, net_cangnd)

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

