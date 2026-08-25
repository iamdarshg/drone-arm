
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")
    added = 0

    def seg(net, x1, y1, x2, y2, w=0.35):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(pcbnew.F_Cu); t.SetNet(net); board.Add(t)

    # CAN_5V: J2 pad1 (4.23,16.38). Route DOWN x=4.23 to y=32, RIGHT to
    # F1 pad1 x=9.68, UP into pad at y=25.5. Check obstacles:
    # - CANH horiz y=13.88 is above start point (16.38 > 13.88 means we go DOWN)
    # - CANL vert (10.14,21.99..22.45): our horizontal at y=32 is below it
    # - GND horiz y=21.5 spans x12-16: our path goes x4.23->x9.68 at y=32,
    #   well below y=21.5. Clear!
    # But wait: the vertical x=4.23 crosses GND horiz y=35.6? That's OUR OWN
    # CAN_GND route which was deleted in cleanup. Let me check what's actually there.
    # From DRC report: no clearance violations on this path, so just do it.
    seg(v5, 4.23, 16.38, 4.23, 32.0)
    seg(v5, 4.23, 32.0, 9.68, 32.0)
    seg(v5, 9.68, 32.0, 9.68, 25.5)
    added += 3

    # CAN_GND: R3 pad1 (21.62,25.5) and J2 pad2 (4.23,15.12) both need to reach
    # the pre-existing via at (22.32,24.23).
    # For R3 pad1: simple hop right+up: (21.62,25.5)->(22.32,24.95)->(22.32,24.23)
    # Check: diag track (19.68,25.21)->(21.09,26.63) ends before x=22.32. Clear!
    seg(g, 21.62, 25.5, 22.32, 24.95)
    seg(g, 22.32, 24.95, 22.32, 24.23)
    added += 2

    # For J2 pad2 (4.23,15.12): this is harder due to the fence of tracks.
    # The pre-existing via at (5.42,14.58) might help! Route from J2 pad2
    # left-up to that via, which should already connect to the In2 network.
    # Actually, let me just check if the other pre-existing via (5.42,14.575)
    # is close enough to J2 pad2:
    # Distance from (4.23,15.12) to (5.42,14.575) = sqrt(1.19^2+0.545^2)=1.31mm
    # We can route a short track between them!
    seg(g, 4.23, 15.12, 4.6, 15.12)
    seg(g, 4.6, 15.12, 4.6, 14.8)
    seg(g, 4.6, 14.8, 5.42, 14.575)
    added += 3

    print(f"added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

