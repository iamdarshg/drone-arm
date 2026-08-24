
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Add vertical F.Cu BATP stubs per cell from each island bottom to the
    # drain-bus row (U1101 pads at y0+88.9) and down to the decoupling caps.
    added = 0
    for motor in range(1, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        net = board.FindNet(f"/M{motor}_BATP")
        if net is None:
            continue
        for px in (60.0, 80.0, 100.0):
            zone = pcbnew.ZONE(board)
            zone.SetNet(net)
            zone.SetLayer(pcbnew.F_Cu)
            zone.SetLocalClearance(MM(0.3))
            zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
            zone.SetAssignedPriority(3)
            outline = zone.Outline()
            outline.NewOutline()
            cx = x0 + px - 4.5
            cy_top = y0 + 82.0    # island bottoms after extension
            cy_bot = y0 + 92.5    # below U1101 drain row
            for x, y in [(cx + 9.0, cy_top), (cx + 9.0, cy_bot),
                         (cx, cy_bot), (cx, cy_top)]:
                outline.Append(MM(x), MM(y))
            board.Add(zone)
            added += 1
    print("added", added, "BATP stubs")
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

