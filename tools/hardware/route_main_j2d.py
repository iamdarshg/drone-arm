
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")

    def s(x1, y1, x2, y2, w=0.4, l=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w))
        t.SetLayer(l)
        t.SetNet(g)
        board.Add(t)

    # CAN_GND: jog left to x=3.4 first (clear of J2 pad1 at 4.23), then down.
    s(4.23, 15.12, 3.4, 15.12)
    s(3.4, 15.12, 3.4, 29.5)
    s(3.4, 29.5, 18.9, 29.5)
    s(18.9, 29.5, 18.9, 24.6)
    s(18.9, 24.6, 21.62, 24.6)
    s(21.62, 24.6, 21.62, 25.5)

    # CAN_5V reroute: right along y=16.38 then down x=8.0, over and up into F1
    v = board.FindNet("/CAN_5V")

    def s5(x1, y1, x2, y2, w=0.45):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w))
        t.SetLayer(pcbnew.F_Cu)
        t.SetNet(v)
        board.Add(t)

    s5(4.23, 16.38, 8.0, 16.38)
    s5(8.0, 16.38, 8.0, 24.4)
    s5(8.0, 24.4, 9.68, 24.4)
    s5(9.68, 24.4, 9.68, 25.5)

    print("routed")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

