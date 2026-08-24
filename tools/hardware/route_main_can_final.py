
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_can5v = board.FindNet("/CAN_5V")
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0

    # remove my previous B.Cu CAN_5V attempts and the F.Cu GND L-route that
    # crossed the GND plane track
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ == "PCB_TRACK" and t.GetNetname() == "/CAN_5V" and t.IsOnLayer(pcbnew.B_Cu):
            doomed.append(t)
        elif type(t).__name__ == "PCB_TRACK" and t.GetNetname() == "/CAN_GND":
            s, e = t.GetStart(), t.GetEnd()
            sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
            if abs(ey - 27.8) < 0.01 and abs(sy - 27.8) < 0.01:
                doomed.append(t)
            elif abs(sx - 21.62) < 0.05 and abs(ex - 21.62) < 0.05 and abs(ey - 25.5) < 0.1:
                doomed.append(t)
        elif type(t).__name__ == "PCB_VIA" and t.GetNetname() in ("/CAN_5V", "/CAN_GND"):
            p = t.GetPosition()
            x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            if (abs(x - 9.0) < 0.05 and abs(y - 62.5) < 0.05) or                (abs(x - 80.54) < 0.05 and abs(y - 62.5) < 0.05) or                (abs(x - 4.23) < 0.05 and abs(y - 15.12) < 0.05):
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

    # CAN_5V feed: corridor y=56 (gap 54.5..57.1). From F1: F.Cu left to x=9,
    # via at (9,56)? No - need vertical first. Vertical at x=9 from y=25.5 to
    # y=56 on B.Cu: vias near x=9? blocked list has none at x~9 except y=57.11
    # (USB_CC1 at 8.86,57.1!). So stop at y=55.5, jog right at y=55.5.
    seg(net_can5v, 9.68, 25.5, 8.3, 25.5, 0.45); added += 1
    via(8.3, 25.5, net_can5v)
    seg(net_can5v, 8.3, 25.5, 8.3, 55.6, 0.45, pcbnew.B_Cu); added += 1
    seg(net_can5v, 8.3, 55.6, 80.54, 55.6, 0.45, pcbnew.B_Cu); added += 1
    via(80.54, 55.6, net_can5v)
    seg(net_can5v, 80.54, 55.6, 80.54, 72.5, 0.45); added += 1

    # CAN_GND J2->R3: route around the GND-track obstacle by going BELOW it:
    # J2 pad2 (4.23,15.12) -> down to y=28.6 -> right to x=21.62 -> up to R3.
    seg(net_cangnd, 4.23, 15.12, 4.23, 28.6, 0.45); added += 1
    seg(net_cangnd, 4.23, 28.6, 21.62, 28.6, 0.45); added += 1
    seg(net_cangnd, 21.62, 28.6, 21.62, 25.5, 0.45); added += 1

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

