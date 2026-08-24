
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_can5v = board.FindNet("/CAN_5V")
    net_cangnd = board.FindNet("/CAN_GND")
    added = removed = 0

    # 1) The J60-J65 5V chain: F1 pad1 is far away at (9.68,25.5) — the chain
    #    itself needs a feed. Route from F1 to the first header on B.Cu with
    #    vias, avoiding the long F.Cu haul.
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

    # F1 (9.68,25.5): drop via, run B.Cu right and down, pop up near J60's
    # 5V hop start (80.54,73.75).
    seg(net_can5v, 9.68, 25.5, 10.8, 25.5, 0.5); added += 1
    via(10.8, 25.5, net_can5v); added += 0
    seg(net_can5v, 10.8, 25.5, 10.8, 70.0, 0.5, pcbnew.B_Cu); added += 1
    seg(net_can5v, 10.8, 70.0, 80.54, 70.0, 0.5, pcbnew.B_Cu); added += 1
    via(80.54, 70.0, net_can5v)
    seg(net_can5v, 80.54, 70.0, 80.54, 72.5, 0.5); added += 1

    # 2) CAN_GND: J2 pad2 (4.23,15.12) -> In2 track end; R3 pad1 -> same net.
    #    Find existing In2.Cu CAN_GND track endpoint near R3/via.
    #    Simpler: add via near R3 pad1 and connect on In2.
    seg(net_cangnd, 21.62, 25.5, 22.32, 24.79, 0.5); added += 1
    via(22.32, 24.79, net_cangnd)

    print("added", added, "+ vias")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

