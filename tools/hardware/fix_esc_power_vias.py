
import re
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])

    # 1) Extend In1 BATP planes: top edge up 8 mm, right edge out 8 mm.
    for z in board.Zones():
        m = re.fullmatch(r"/M\d+_BATP", z.GetNetname())
        if not m or not z.GetLayerSet().Contains(pcbnew.In1_Cu):
            continue
        poly = z.Outline()
        o = poly.Outline(0)
        for i in range(o.PointCount()):
            v = o.CPoint(i)
            x, y = pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)
            nx, ny = x + 8.0, y - 8.0
            o.SetPoint(i, pcbnew.VECTOR2I(MM(nx), MM(ny)))
        print("extended In1 BATP plane at",
              round(pcbnew.ToMM(z.GetBoundingBox().GetX()),1),
              round(pcbnew.ToMM(z.GetBoundingBox().GetY()),1))

    # 2) Extend BATN F.Cu rect (76x22) right edge +6, and the 6-corner B.Cu BATN poly.
    for z in board.Zones():
        m = re.fullmatch(r"/M\d+_BATN", z.GetNetname())
        if not m:
            continue
        if z.GetLayerSet().Contains(pcbnew.F_Cu):
            bb = z.GetBoundingBox()
            w = pcbnew.ToMM(bb.GetWidth())
            h = pcbnew.ToMM(bb.GetHeight())
            if abs(w - 76) < 2 and abs(h - 22) < 2:
                poly = z.Outline()
                o = poly.Outline(0)
                for i in range(o.PointCount()):
                    v = o.CPoint(i)
                    if pcbnew.ToMM(v.x) > pcbnew.ToMM(bb.GetX()) + w / 2:
                        o.SetPoint(i, pcbnew.VECTOR2I(v.x + MM(6.0), v.y))
                print("extended F.Cu BATN rect")
        elif z.GetLayerSet().Contains(pcbnew.B_Cu):
            poly = z.Outline()
            if poly.Outline(0).PointCount() == 6:
                xs = []
                for i in range(poly.Outline(0).PointCount()):
                    xs.append(pcbnew.ToMM(poly.Outline(0).CPoint(i).x))
                xmax = max(xs)
                o = poly.Outline(0)
                for i in range(o.PointCount()):
                    v = o.CPoint(i)
                    if abs(pcbnew.ToMM(v.x) - xmax) < 0.01:
                        o.SetPoint(i, pcbnew.VECTOR2I(v.x + MM(6.0), v.y))
                print("extended B.Cu BATN poly")

    # 3) Move BATP via grids down 1.5 mm into their islands.
    moved = 0
    for motor in range(1, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        for px, hy in ((60, 32), (80, 24), (100, 32)):
            cx = x0 + px
            cy_old = y0 + hy - 8.5
            for t in board.GetTracks():
                if type(t).__name__ != "PCB_VIA":
                    continue
                if t.GetNetname() != f"/M{motor}_BATP":
                    continue
                p = t.GetPosition()
                vx, vy = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
                if abs(vx - cx) < 5.0 and abs(vy - cy_old) < 0.7:
                    t.SetPosition(pcbnew.VECTOR2I(p.x, p.y + MM(1.5)))
                    moved += 1
    print("moved", moved, "BATP vias")

    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

