
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    fixed = 0
    missing = 0
    for motor in range(1, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        for px, hy in ((60, 32), (80, 24), (100, 32)):
            cx = x0 + px
            cy = y0 + hy - 8.5
            targets = []
            for c in range(8):
                for r in range(3):
                    targets.append((cx + (c - 3.5) * 1.35, cy + (r - 1) * 1.35))
            vias = []
            for t in board.GetTracks():
                if type(t).__name__ != "PCB_VIA":
                    continue
                if t.GetNetname() != f"/M{motor}_BATP":
                    continue
                p = t.GetPosition()
                vx, vy = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
                if abs(vx - cx) < 6.5 and abs(vy - cy) < 3.5:
                    vias.append((vx, vy, t))
            if len(vias) < 24:
                print(f"M{motor} phase@{px}: only {len(vias)} vias found")
            used = [False] * len(vias)
            for tx, ty in targets:
                best = None
                best_d = 1e9
                for i, (vx, vy, t) in enumerate(vias):
                    if used[i]:
                        continue
                    d = abs(vx - tx) + abs(vy - ty)
                    if d < best_d:
                        best_d = d
                        best = i
                if best is None or best_d > 2.5:
                    missing += 1
                    continue
                vx, vy, t = vias[best]
                used[best] = True
                if abs(vx - tx) > 0.01 or abs(vy - ty) > 0.01:
                    t.SetPosition(pcbnew.VECTOR2I(MM(tx), MM(ty)))
                    fixed += 1
    print("normalized", fixed, "vias; unmatched targets:", missing)
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

