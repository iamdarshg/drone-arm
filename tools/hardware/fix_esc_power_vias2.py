
import re
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])

    # 1) Move the 126 BATP vias back up 1.5 mm to their original spots.
    moved = 0
    for motor in range(1, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        for px, hy in ((60, 32), (80, 24), (100, 32)):
            cx = x0 + px
            cy_new = y0 + hy - 7.0
            for t in board.GetTracks():
                if type(t).__name__ != "PCB_VIA":
                    continue
                if t.GetNetname() != f"/M{motor}_BATP":
                    continue
                p = t.GetPosition()
                vx, vy = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
                if abs(vx - cx) < 5.0 and abs(vy - cy_new) < 0.7:
                    t.SetPosition(pcbnew.VECTOR2I(p.x, p.y - MM(1.5)))
                    moved += 1
    print("restored", moved, "BATP vias")

    # 2) Extend F.Cu BATP phase islands upward by 2 mm (top edge yh-9 -> yh-11).
    n = 0
    for z in board.Zones():
        m = re.fullmatch(r"/M\d+_BATP", z.GetNetname())
        if not m or not z.GetLayerSet().Contains(pcbnew.F_Cu):
            continue
        bb = z.GetBoundingBox()
        w = pcbnew.ToMM(bb.GetWidth())
        h = pcbnew.ToMM(bb.GetHeight())
        if abs(w - 12) < 1.5 and abs(h - 12) < 1.5:
            poly = z.Outline()
            o = poly.Outline(0)
            for i in range(o.PointCount()):
                v = o.CPoint(i)
                o.SetPoint(i, pcbnew.VECTOR2I(v.x, v.y - MM(2.0)))
            n += 1
    print("raised", n, "F.Cu BATP islands")

    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

