
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    added_zones = added_vias = 0

    for motor in range(2, 7):
        batn_net_name = f"/M{motor}_BATN"
        net = board.FindNet(batn_net_name)
        if net is None:
            continue

        # Compute absolute coords from M1's known positions
        # M1 cell origin: x0=3, y0=43
        m1_x0, m1_y0 = 3, 43
        dx = ((motor - 1) % 3) * 157
        dy = ((motor - 1) // 3) * 100
        cx0 = m1_x0 + dx
        cy0 = m1_y0 + dy

        # Check if controller strip zone exists for this motor (matching M1's at abs 16-22.6, 97.8-103.2)
        strip_exists = False
        target_x0 = 16.0 + dx
        target_y0 = 97.8 + dy
        for z in board.Zones():
            if z.GetNetname() == batn_net_name and z.GetLayerSet().Contains(pcbnew.F_Cu):
                bb = z.GetBoundingBox()
                zx = pcbnew.ToMM(bb.GetX())
                zy = pcbnew.ToMM(bb.GetY())
                if abs(zx - target_x0) < 2 and abs(zy - target_y0) < 2:
                    strip_exists = True
                    break

        if not strip_exists:
            zone = pcbnew.ZONE(board)
            zone.SetNet(net)
            zone.SetLayer(pcbnew.F_Cu)
            zone.SetLocalClearance(MM(0.3))
            zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
            outline = zone.Outline()
            outline.NewOutline()
            pts = [
                (16.0 + dx, 97.8 + dy),
                (22.6 + dx, 97.8 + dy),
                (22.6 + dx, 103.2 + dy),
                (16.0 + dx, 103.2 + dy),
            ]
            for px, py in pts:
                outline.Append(MM(px), MM(py))
            board.Add(zone)
            added_zones += 1
            print(f"M{motor}: added controller strip zone")

    pcbnew.SaveBoard(sys.argv[1], board)
    print(f"added {added_zones} zones, {added_vias} vias")
    return 0


raise SystemExit(main())

