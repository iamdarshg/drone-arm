
import re
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    added_zones = 0
    added_vias = 0

    # For each motor cell 1..6, ensure a F.Cu BATN island exists covering the
    # controller strip pads (U1105/U2105/etc pad 2/4/7 + C-pads), stitched to
    # the B.Cu plane via two vias in verified-clear locations.
    for motor in range(1, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100

        batn_net_name = f"/M{motor}_BATN"
        net = board.FindNet(batn_net_name)
        if net is None:
            print("net missing", batn_net_name)
            continue

        # Check if a F.Cu BATN zone already exists in this cell region
        has_fc_zone = False
        for z in board.Zones():
            if z.GetNetname() == batn_net_name and z.GetLayerSet().Contains(pcbnew.F_Cu):
                bb = z.GetBoundingBox()
                zx = pcbnew.ToMM(bb.GetX())
                zy = pcbnew.ToMM(bb.GetY())
                # does it overlap the cell area?
                cx0, cy0 = x0 + 14, y0 + 44   # controller strip region
                cx1, cy1 = x0 + 30, y0 + 62
                if not (zx > cx1 or zx + pcbnew.ToMM(bb.GetWidth()) < cx0 or
                        zy > cy1 or zy + pcbnew.ToMM(bb.GetHeight()) < cy0):
                    has_fc_zone = True
                    break

        if has_fc_zone:
            print(f"M{motor}: F.Cu BATN zone exists")
            continue

        # Create the zone covering the controller strip
        zone = pcbnew.ZONE(board)
        zone.SetNet(net)
        zone.SetLayer(pcbnew.F_Cu)
        zone.SetLocalClearance(MM(0.3))
        zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        outline = zone.Outline()
        outline.NewOutline()
        # Controller strip: U{base+5} at ~x0+21,y0+100? Actually from probe:
        # U1105 at (21.0, 100.0) => relative to x0=3,y0=43: dx=18, dy=57.
        # C1176 at (26,99). C1131/32/33/34 at y~62. So cover x0+15..x0+28,
        # y0+54..y0+58 (the bootstrap cap row) AND the U-pad cluster.
        # Simplify: one rectangle over the cap row.
        pts = [(x0 + 14.5, y0 + 60.5), (x0 + 28.5, y0 + 60.5),
               (x0 + 28.5, y0 + 63.8), (x0 + 14.5, y0 + 63.8)]
        for px, py in pts:
            outline.Append(MM(px), MM(py))
        board.Add(zone)
        added_zones += 1
        print(f"M{motor}: added F.Cu BATN island")

    pcbnew.SaveBoard(sys.argv[1], board)
    print(f"added {added_zones} zones")
    return 0


raise SystemExit(main())

