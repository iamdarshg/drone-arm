
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Extend the B.Cu M1_BATN plane upward to cover the controller strip
    # (y 88 -> y 96) where U1105/C1176/C1130/C1103 BATN pads sit.
    n = 0
    for z in board.Zones():
        if z.GetNetname() != "/M1_BATN" or not z.GetLayerSet().Contains(pcbnew.B_Cu):
            continue
        o = z.Outline().Outline(0)
        if o.PointCount() == 4:
            ymin = min(pcbnew.ToMM(o.CPoint(i).y) for i in range(4))
            if abs(ymin - 88.0) < 0.5:
                for i in range(4):
                    v = o.CPoint(i)
                    if abs(pcbnew.ToMM(v.y) - ymin) < 0.01:
                        o.SetPoint(i, pcbnew.VECTOR2I(v.x, v.y - MM(8.0)))
                print("extended B.Cu M1_BATN up to y=80")
                n += 1
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved", n)
    return 0


raise SystemExit(main())

