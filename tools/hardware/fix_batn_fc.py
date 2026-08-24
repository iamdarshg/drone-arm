
import re
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    n = 0
    for z in board.Zones():
        if not re.fullmatch(r"/M\d+_BATN", z.GetNetname()):
            continue
        if not z.GetLayerSet().Contains(pcbnew.F_Cu):
            continue
        bb = z.GetBoundingBox()
        if abs(pcbnew.ToMM(bb.GetWidth()) - 82) < 2.5 and abs(pcbnew.ToMM(bb.GetHeight()) - 22) < 2:
            o = z.Outline().Outline(0)
            ys = [pcbnew.ToMM(o.CPoint(i).y) for i in range(o.PointCount())]
            xs = [pcbnew.ToMM(o.CPoint(i).x) for i in range(o.PointCount())]
            ymin, ymax, xmin = min(ys), max(ys), min(xs)
            for i in range(o.PointCount()):
                v = o.CPoint(i)
                x, y = pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)
                nx, ny = x, y
                if abs(y - ymin) < 0.01:
                    ny = y - 8.0
                elif abs(y - ymax) < 0.01:
                    ny = y + 4.0
                if abs(x - xmin) < 0.01:
                    nx = x - 3.0
                o.SetPoint(i, pcbnew.VECTOR2I(MM(nx), MM(ny)))
            n += 1
    print("extended", n, "F.Cu BATN rects")
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

