
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0
    # The GND track runs (21.09,26.63)->(23.41,26.63): crosses x=21.62 at y=26.63.
    # Our riser x=21.62 from 29.3 to 25.5 crosses it. Reroute: approach R3 pad1
    # from the RIGHT side: horizontal at y=29.3 to x=23.8, then up x=23.8 to
    # y=25.5, then left into R3 pad.
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() != "/CAN_GND" or type(t).__name__ != "PCB_TRACK":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(sx - 21.62) < 0.05 and abs(ex - 21.62) < 0.05 and abs(ey - 25.5) < 0.1:
            doomed.append(t)
    for t in doomed:
        board.Remove(t); removed += 1

    def seg(net, x1, y1, x2, y2, w):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w))
        t.SetLayer(pcbnew.F_Cu)
        t.SetNet(net)
        board.Add(t)

    seg(net_cangnd, 21.62, 29.3, 24.2, 29.3, 0.45); added += 1
    seg(net_cangnd, 24.2, 29.3, 24.2, 25.5, 0.45); added += 1
    seg(net_cangnd, 24.2, 25.5, 21.62, 25.5, 0.45); added += 1

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

