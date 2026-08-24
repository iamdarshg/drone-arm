
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    extended = 0
    for motor in [1, 2, 3]:
        dx = ((motor-1)%3)*157
        dy = ((motor-1)//3)*100
        cx0 = 3+dx; cy0 = 43+dy
        batn_net_name = f"/M{motor}_BATN"
        for z in board.Zones():
            if z.GetNetname() == batn_net_name and z.HasFilledPolysForLayer(pcbnew.F_Cu):
                bb = z.GetBoundingBox()
                zx, zy = pcbnew.ToMM(bb.GetX()), pcbnew.ToMM(bb.GetY())
                zw, zh = pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight())
                # Match the controller strip (small ~6.6x5.4 zone in this cell)
                if abs(zx-(cx0+13)) < 3 and abs(zy-(cy0+54.8)) < 3 and abs(zw-6.6) < 2:
                    poly = z.Outline()
                    o = poly.Outline(0)
                    ymin = min(pcbnew.ToMM(o.CPoint(i).y) for i in range(o.PointCount()))
                    for i in range(o.PointCount()):
                        v = o.CPoint(i)
                        if abs(pcbnew.ToMM(v.y) - ymin) < 0.01:
                            o.SetPoint(i, pcbnew.VECTOR2I(v.x, v.y - MM(2.0)))
                    extended += 1
                    print(f"M{motor}: extended strip top edge up 2mm")
    pcbnew.SaveBoard(sys.argv[1], board)
    print(f"extended {extended}")
    return 0


raise SystemExit(main())

