
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Add small F.Cu BATP landing pads (zones) at each driver BATP pad pair
    # and stitch to the islands above via short wide stubs.
    added = 0
    for motor in range(1, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        net = board.FindNet(f"/M{motor}_BATP")
        if net is None:
            continue
        zone = pcbnew.ZONE(board)
        zone.SetNet(net)
        zone.SetLayer(pcbnew.F_Cu)
        zone.SetLocalClearance(MM(0.3))
        zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        zone.SetAssignedPriority(3)
        outline = zone.Outline()
        outline.NewOutline()
        # horizontal strip covering the three drivers' BATP pads:
        # x from x0+91.5 to x0+94.6, y from y0+88.6 to y0+89.2
        for x, y in [(x0 + 91.5, y0 + 88.55), (x0 + 92.5, y0 + 88.55),
                     (x0 + 92.5, y0 + 89.25), (x0 + 91.5, y0 + 89.25)]:
            outline.Append(MM(x), MM(y))
        board.Add(zone)
        added += 1
    print("added", added, "BATP pad strips")
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

