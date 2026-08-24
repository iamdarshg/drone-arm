
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0

    # The GND planes cover the entire board on B.Cu and In1.Cu - any via
    # anywhere violates them unless the fill clears it. The fills DID clear
    # for other nets' vias (thousands exist). The problem: my vias were added
    # AFTER the last refill? No - we refilled. Unless... the zone filler
    # doesn't know about the new vias because they were added without
    # marking the zone dirty. Force full re-fill with Fill() on modified zones.
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones(), True)  # True = force refill all
    pcbnew.SaveBoard(sys.argv[1], board)
    print("forced full refill done")
    return 0


raise SystemExit(main())

