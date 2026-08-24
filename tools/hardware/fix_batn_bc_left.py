
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Extend the 6-corner B.Cu M1_BATN plane: left edge x=6 -> x=16,
    # top edge y=88 stays; this covers U1105 (x18.5) etc? No - extend to x=17.
    n = 0
    for z in board.Zones():
        if str(z.m_Uuid.AsString()) != "85731cf2-23dc-4ed6-9ad8-d4e7c7996810":
            continue
        o = z.Outline().Outline(0)
        for i in range(o.PointCount()):
            v = o.CPoint(i)
            if abs(pcbnew.ToMM(v.x) - 6.0) < 0.01 and pcbnew.ToMM(v.y) < 100:
                o.SetPoint(i, pcbnew.VECTOR2I(MM(17.0), v.y))
        n += 1
        print("extended left edge to x=17")
    # Also extend the other five cells' B.Cu planes similarly.
    for motor in range(2, 7):
        for z in board.Zones():
            if z.GetNetname() != f"/M{motor}_BATN" or not z.GetLayerSet().Contains(pcbnew.B_Cu):
                continue
            o = z.Outline().Outline(0)
            if o.PointCount() == 6:
                xs = [pcbnew.ToMM(o.CPoint(i).x) for i in range(6)]
                xmin = min(xs)
                for i in range(6):
                    v = o.CPoint(i)
                    if abs(pcbnew.ToMM(v.x) - xmin) < 0.01 and pcbnew.ToMM(v.y) < max(
                            [pcbnew.ToMM(o.CPoint(j).y) for j in range(6)]):
                        o.SetPoint(i, pcbnew.VECTOR2I(v.x - MM(11.0), v.y))
                        n += 1
                        print(f"extended M{motor} left edge by 11mm")
                        break
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved", n)
    return 0


raise SystemExit(main())

