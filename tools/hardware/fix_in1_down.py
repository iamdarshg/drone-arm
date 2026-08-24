
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Extend each In1.BATP plane bottom edge down 6 mm to cover the
    # (x0+22, y0+64) 5x3 BATP_IN transition grid.
    n = 0
    for z in board.Zones():
        name = z.GetNetname()
        if not name.endswith("_BATP"):
            continue
        if not z.GetLayerSet().Contains(pcbnew.In1_Cu):
            continue
        o = z.Outline().Outline(0)
        ymax = max(pcbnew.ToMM(o.CPoint(i).y) for i in range(o.PointCount()))
        for i in range(o.PointCount()):
            v = o.CPoint(i)
            if abs(pcbnew.ToMM(v.y) - ymax) < 0.01:
                o.SetPoint(i, pcbnew.VECTOR2I(v.x, v.y + MM(6.0)))
        n += 1
    print("extended", n, "In1 BATP planes downward")
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

