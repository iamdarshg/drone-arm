
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # shrink strip: move right edge 27.5 -> 22.8; move via at x=25 -> x=21;
    # reroute its segment.
    net = board.FindNet("/M1_BATN")
    for z in board.Zones():
        if z.GetNetname() == "/M1_BATN" and z.GetLayerSet().Contains(pcbnew.F_Cu):
            o = z.Outline().Outline(0)
            xs = [pcbnew.ToMM(o.CPoint(i).x) for i in range(o.PointCount())]
            if max(xs) > 27 and min(xs) > 15:
                for i in range(o.PointCount()):
                    v = o.CPoint(i)
                    if abs(pcbnew.ToMM(v.x) - 27.5) < 0.01:
                        o.SetPoint(i, pcbnew.VECTOR2I(MM(22.6), v.y))
                print("strip right edge -> 22.6")
                break
    for t in board.GetTracks():
        if type(t).__name__ == "PCB_VIA" and t.GetNetname() == "/M1_BATN":
            p = t.GetPosition()
            if abs(pcbnew.ToMM(p.x) - 25.0) < 0.05 and abs(pcbnew.ToMM(p.y) - 96.5) < 0.05:
                t.SetPosition(pcbnew.VECTOR2I(MM(21.0), MM(96.5)))
                print("via 25->21")
        if type(t).__name__ == "PCB_TRACK" and t.GetNetname() == "/M1_BATN":
            s, e = t.GetStart(), t.GetEnd()
            if abs(pcbnew.ToMM(s.x) - 25.0) < 0.05 or abs(pcbnew.ToMM(e.x) - 25.0) < 0.05:
                t.SetStart(pcbnew.VECTOR2I(MM(21.0), MM(97.8)))
                t.SetEnd(pcbnew.VECTOR2I(MM(21.0), MM(96.5)))
                print("seg moved to x=21")
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

