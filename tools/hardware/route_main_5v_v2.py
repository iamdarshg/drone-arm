
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    v5 = board.FindNet("/CAN_5V")
    g = board.FindNet("/CAN_GND")
    removed = added = 0

    # Remove all CAN_5V B.Cu tracks and vias (keep GND)
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() != "/CAN_5V":
            continue
        if type(t).__name__ == "PCB_VIA":
            doomed.append(t)
        elif t.IsOnLayer(pcbnew.B_Cu):
            doomed.append(t)
    for t in doomed:
        try: board.Remove(t); removed += 1
        except: pass

    def seg(net, x1, y1, x2, y2, w=0.4, l=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(l); t.SetNet(net); board.Add(t)

    xs_5v = [80.54, 89.54, 98.54, 107.54, 116.54, 125.54]

    # Rebuild 5V on F.Cu only: hops between adjacent pins going DOWN to y=73.9
    # with w=0.25 (edge clearance: track bottom edge 74.025 vs board edge 74.52
    # => gap 0.495mm > 0.2mm min). Between-pin corridor is clear.
    for i in range(5):
        x1, x2 = xs_5v[i], xs_5v[i+1]
        seg(v5, x1, 72.5, x1, 73.9, 0.2)
        seg(v5, x1, 73.9, x2, 73.9, 0.2)
        seg(v5, x2, 73.9, x2, 72.5, 0.2)
        added += 3

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

