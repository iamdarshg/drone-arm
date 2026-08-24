
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    doomed = []
    for z in board.Zones():
        if z.GetNetname().endswith("_BATP") and z.GetLayerSet().Contains(pcbnew.F_Cu):
            o = z.Outline().Outline(0)
            pts = [(round(pcbnew.ToMM(o.CPoint(i).x), 1), round(pcbnew.ToMM(o.CPoint(i).y), 1))
                   for i in range(o.PointCount())]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            w, h = max(xs) - min(xs), max(ys) - min(ys)
            if abs(w - 9.0) < 0.5 and abs(h - 10.5) < 0.5:
                doomed.append(z)
    for z in doomed:
        board.Remove(z)
    print("removed", len(doomed), "stub zones")
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

