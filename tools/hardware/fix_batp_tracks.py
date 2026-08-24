
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Draw explicit F.Cu tracks: U1101 pads 3/4 (BATP) -> up to the phase-A
    # island bottom (y0+82). Track from (91.75, 88.9) and (92.25, 88.9) to
    # a junction at (92.0, 84.5), then to (92.0, 82.0) inside the island.
    added = 0
    for motor in range(1, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        net = board.FindNet(f"/M{motor}_BATP")
        if net is None:
            continue

        def seg(x1, y1, x2, y2, w):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
            t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
            t.SetWidth(MM(w))
            t.SetLayer(pcbnew.F_Cu)
            t.SetNet(net)
            board.Add(t)

        # pad3/pad4 fan-in then vertical riser into the phase-A island area
        seg(x0 + 91.75, y0 + 88.9, x0 + 91.9, y0 + 86.6, 0.8)
        seg(x0 + 92.25, y0 + 88.9, x0 + 92.1, y0 + 86.6, 0.8)
        seg(x0 + 91.9, y0 + 86.6, x0 + 92.1, y0 + 86.6, 0.8)
        seg(x0 + 92.0, y0 + 86.6, x0 + 92.0, y0 + 80.5, 1.2)
        added += 4
    print("added", added, "segments")
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

