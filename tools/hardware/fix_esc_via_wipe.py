
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    removed = 0
    for motor in range(2, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        phase_x = (60.0, 80.0, 100.0)
        high_y = (32.0, 24.0, 32.0)
        low_y = (52.0, 60.0, 52.0)
        shunt_y = (69.0, 79.0, 69.0)
        centers = []
        for k in range(3):
            px, hy, yl, sy = phase_x[k], high_y[k], low_y[k], shunt_y[k]
            centers += [(x0 + px, y0 + hy - 8.5, 5.5),
                        (x0 + px - 1.35, y0 + hy + 5.5, 3.5),
                        (x0 + px - 1.35, y0 + yl - 5.1, 3.5),
                        (x0 + px, y0 + sy + 6, 5.5)]
        centers.append((x0 + 22, y0 + 64, 4.0))
        doomed = []
        for t in board.GetTracks():
            if type(t).__name__ != "PCB_VIA":
                continue
            if not t.GetNetname().startswith(f"/M{motor}_"):
                continue
            p = t.GetPosition()
            vx, vy = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            for cx, cy, r in centers:
                if abs(vx - cx) < r and abs(vy - cy) < r:
                    doomed.append(t)
                    break
        for t in doomed:
            board.Remove(t)
            removed += 1
    print("removed", removed)
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

