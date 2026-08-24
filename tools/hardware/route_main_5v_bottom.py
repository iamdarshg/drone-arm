
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    v5 = board.FindNet("/CAN_5V")
    g = board.FindNet("/CAN_GND")
    removed = 0

    # Remove the failed CAN_5V top-route (all its F.Cu tracks in x<17, y<27)
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() != "/CAN_5V":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if max(sx,ex) < 17 and max(sy,ey) < 34:
            doomed.append(t)
    for t in doomed:
        board.Remove(t); removed += 1

    def seg(net, x1, y1, x2, y2, w=0.4):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(pcbnew.F_Cu); t.SetNet(net); board.Add(t)

    # CAN_5V bottom route: J2 pin1 down-left margin then along bottom.
    seg(v5, 4.23, 16.38, 4.23, 33.0)
    seg(v5, 4.23, 33.0, 9.68, 33.0)
    seg(v5, 9.68, 33.0, 9.68, 25.5)

    print("removed", removed)
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

