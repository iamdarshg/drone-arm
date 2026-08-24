
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    snapped = created = removed = 0

    def ensure_grid(net_name, cx, cy, cols, rows, pitch=1.35):
        nonlocal snapped, created, removed
        targets = [(cx + (c - (cols - 1) / 2) * pitch,
                    cy + (r - (rows - 1) / 2) * pitch)
                   for c in range(cols) for r in range(rows)]
        box_w, box_h = (cols - 1) * pitch / 2 + 1.0, (rows - 1) * pitch / 2 + 1.0
        vias = []
        for t in list(board.GetTracks()):
            if type(t).__name__ != "PCB_VIA" or t.GetNetname() != net_name:
                continue
            p = t.GetPosition()
            vx, vy = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            if abs(vx - cx) < box_w and abs(vy - cy) < box_h:
                vias.append((vx, vy, t))
        used = set()
        for tx, ty in targets:
            best, best_d = None, 1e9
            for i, (vx, vy, t) in enumerate(vias):
                if i in used:
                    continue
                d = abs(vx - tx) + abs(vy - ty)
                if d < best_d:
                    best_d, best = d, i
            if best is not None and best_d <= 2.5:
                vx, vy, t = vias[best]
                used.add(best)
                if abs(vx - tx) > 0.01 or abs(vy - ty) > 0.01:
                    t.SetPosition(pcbnew.VECTOR2I(MM(tx), MM(ty)))
                    snapped += 1
            else:
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(pcbnew.VECTOR2I(MM(tx), MM(ty)))
                via.SetWidth(MM(1.0))
                via.SetDrill(MM(0.5))
                via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                net = board.FindNet(net_name)
                via.SetNet(net)
                board.Add(via)
                created += 1
        for i, (vx, vy, t) in enumerate(vias):
            if i not in used:
                board.Remove(t)
                removed += 1

    for motor in range(1, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        phase_x = (60.0, 80.0, 100.0)
        high_y = (32.0, 24.0, 32.0)
        low_y = (52.0, 60.0, 52.0)
        shunt_y = (69.0, 79.0, 69.0)
        for k in range(3):
            px, hy, yl, sy = phase_x[k], high_y[k], low_y[k], shunt_y[k]
            ensure_grid(f"/M{motor}_BATP", x0 + px, y0 + hy - 8.5, 8, 3)
            ensure_grid(f"/M{motor}_PHASE_" + "ABC"[k], x0 + px - 1.35, y0 + hy + 5.5, 5, 2)
            ensure_grid(f"/M{motor}_PHASE_" + "ABC"[k], x0 + px - 1.35, y0 + yl - 5.1, 5, 2)
            ensure_grid(f"/M{motor}_BATN", x0 + px, y0 + sy + 6, 8, 3)
        ensure_grid(f"/M{motor}_BATP", x0 + 22, y0 + 64, 5, 3)

    print(f"snapped={snapped} created={created} removed_stray={removed}")
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

