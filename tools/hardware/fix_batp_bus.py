
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Add a wide F.Cu BATP bus strip per cell connecting the three islands
    # down through the driver area to the shunt/charging-pump row.
    for motor in range(1, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        net = board.FindNet(f"/M{motor}_BATP")
        if net is None:
            print("net missing", motor)
            continue
        zone = pcbnew.ZONE(board)
        zone.SetNet(net)
        zone.SetLayer(pcbnew.F_Cu)
        zone.SetLocalClearance(MM(0.3))
        zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        zone.SetAssignedPriority(3)
        outline = zone.Outline()
        outline.NewOutline()
        # horizontal strip at y = y0+86..y0+90, x from x0+55 to x0+111
        for x, y in [(x0 + 55.5, y0 + 85.5), (x0 + 110.5, y0 + 85.5),
                     (x0 + 110.5, y0 + 89.5), (x0 + 55.5, y0 + 89.5)]:
            outline.Append(MM(x), MM(y))
        board.Add(zone)
    pcbnew.SaveBoard(sys.argv[1], board)
    print("added BATP bus strips; saved")
    return 0


raise SystemExit(main())

