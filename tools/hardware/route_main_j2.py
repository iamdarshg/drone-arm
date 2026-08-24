
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

    # CAN_5V: J2 pad1 (4.23,16.38) -> F1 pad1 (9.68,25.5).
    # Obstacles: CANH/CANL tracks occupy x>9 upper area; GND track at y=21.5
    # spans x 12..16 (right of our path). Clear L-path: down x=4.23 to y=27,
    # right to x=9.68? passes below CANL vertical at x=10.14 y22-22.45 - fine
    # since we stop at x=9.68. Then up to F1 pad at y=25.5.
    seg(net_can5v, 4.23, 16.38, 4.23, 27.0, 0.45); added += 1
    seg(net_can5v, 4.23, 27.0, 9.68, 27.0, 0.45); added += 1
    seg(net_can5v, 9.68, 27.0, 9.68, 25.5, 0.45); added += 1

    # CAN_GND: J2 pad2 (4.23,15.12) -> R3 pad1 (21.62,25.5).
    # Route below everything: down x=4.23 to y=29.5, right to x=21.62, up.
    # Check obstacles near y=29.5: none listed in scan (scan stopped at 28).
    # The GND track near (19.7-21.3, 25.2-26.6) is above our y=29.5 run; the
    # final riser at x=21.62 from 29.5 up to 25.5 crosses it at ~y26.6?
    # That GND track is at (19.68..21.31). Our riser x=21.62 > 21.31 - clear.
    seg(net_cangnd, 4.23, 15.12, 4.23, 29.5, 0.45); added += 1
    seg(net_cangnd, 4.23, 29.5, 21.62, 29.5, 0.45); added += 1
    seg(net_cangnd, 21.62, 29.5, 21.62, 25.5, 0.45); added += 1

    print("added", added)
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

