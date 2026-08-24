
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_can5v = board.FindNet("/CAN_5V")
    net_cangnd = board.FindNet("/CAN_GND")
    added = 0

    def seg(net, x1, y1, x2, y2, w, layer=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w))
        t.SetLayer(layer)
        t.SetNet(net)
        board.Add(t)

    # 1) J2 pad1 (4.23,16.38) -> F1 pad1 (9.68,25.5): L route on B.Cu or F.Cu.
    #    Check obstacles: keep simple L on F.Cu: up to y=25.5 then right to F1.
    seg(net_can5v, 4.23, 16.38, 4.23, 25.5, 0.5)
    seg(net_can5v, 4.23, 25.5, 9.68, 25.5, 0.5)
    added += 2

    # 2) R3 pad1 (21.62,25.5) connects CAN_GND: route to the existing CAN_GND
    #    via at (22.3204,24.2308).
    seg(net_cangnd, 21.62, 25.5, 22.32, 24.79, 0.5)
    added += 1

    print("added", added)
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

