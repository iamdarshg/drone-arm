
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Extend every F.Cu BATP island downward 6 mm (covers drain pads,
    # C1102/C1105/C1106/C1133/C1134 decoupling and the U1101/U1401 drain bus).
    n = 0
    for z in board.Zones():
        name = z.GetNetname()
        if not name.endswith("_BATP"):
            continue
        if not z.GetLayerSet().Contains(pcbnew.F_Cu):
            continue
        bb = z.GetBoundingBox()
        if abs(pcbnew.ToMM(bb.GetWidth()) - 12) < 1.6:
            o = z.Outline().Outline(0)
            ymax = max(pcbnew.ToMM(o.CPoint(i).y) for i in range(o.PointCount()))
            for i in range(o.PointCount()):
                v = o.CPoint(i)
                if abs(pcbnew.ToMM(v.y) - ymax) < 0.01:
                    o.SetPoint(i, pcbnew.VECTOR2I(v.x, v.y + MM(6.0)))
            n += 1
    print("extended", n, "F.Cu BATP islands downward")
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

