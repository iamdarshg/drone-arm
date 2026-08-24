
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")

    def seg(x1, y1, x2, y2, w=0.45, l=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(l); t.SetNet(g); board.Add(t)

    def via(x, y):
        vv = pcbnew.PCB_VIA(board)
        vv.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        vv.SetWidth(MM(0.8)); vv.SetDrill(MM(0.4))
        vv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        vv.SetNet(g); board.Add(vv)

    def add_keepout(x, y, w, h, layers):
        k = pcbnew.ZONE(board)
        k.SetIsKnockout(True)   # KiCad 9: knockout = cutout in other zones
        k.SetDoNotAllowCopperPour(True)
        k.SetDoNotAllowTracks(True)
        k.SetDoNotAllowVias(True)
        k.SetLayerSet(layers)
        outline = k.Outline()
        outline.NewOutline()
        outline.Append(MM(x - w/2), MM(y - h/2))
        outline.Append(MM(x + w/2), MM(y - h/2))
        outline.Append(MM(x + w/2), MM(y + h/2))
        outline.Append(MM(x - w/2), MM(y + h/2))
        board.Add(k)

    layers = pcbnew.LSET()
    layers.AddLayer(pcbnew.B_Cu)
    layers.AddLayer(pcbnew.In1_Cu)
    add_keepout(16.0, 27.6, 2.0, 2.0, layers)
    add_keepout(20.9, 27.6, 2.0, 2.0, layers)

    seg(4.23, 15.12, 2.6, 15.12)
    seg(2.6, 15.12, 2.6, 31.5)
    seg(2.6, 31.5, 16.0, 31.5)
    seg(16.0, 31.5, 16.0, 27.6)
    via(16.0, 27.6)
    seg(16.0, 27.6, 20.9, 27.6, 0.45, pcbnew.In2_Cu)
    via(20.9, 27.6)
    seg(20.9, 27.6, 20.9, 25.5)
    seg(20.9, 25.5, 21.62, 25.5)
    print("routed with knockouts")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

