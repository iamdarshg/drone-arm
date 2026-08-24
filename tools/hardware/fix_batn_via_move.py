
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Move the two new BATN stitching vias from y=104.6 to y=96.5 (above the strip),
    # and re-route the stub segments upward instead of downward.
    net = board.FindNet("/M1_BATN")
    moved = 0
    for t in board.GetTracks():
        if type(t).__name__ == "PCB_VIA" and t.GetNetname() == "/M1_BATN":
            p = t.GetPosition()
            x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            if abs(y - 104.6) < 0.05 and (abs(x - 17.5) < 0.05 or abs(x - 25.0) < 0.05):
                t.SetPosition(pcbnew.VECTOR2I(p.x, MM(96.5)))
                moved += 1
    # fix stub segments: they run 103.2->104.6; make them 97.8->96.5
    fixed = 0
    for t in board.GetTracks():
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() != "/M1_BATN":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(sy - 103.2) < 0.01 and abs(ey - 104.6) < 0.01 and (abs(sx-17.5)<0.1 or abs(sx-25.0)<0.1):
            t.SetStart(pcbnew.VECTOR2I(MM(sx), MM(97.8)))
            t.SetEnd(pcbnew.VECTOR2I(MM(ex), MM(96.5)))
            fixed += 1
    print("moved vias:", moved, "fixed segs:", fixed)
    pcbnew.SaveBoard(sys.argv[1], board)
    print("saved")
    return 0


raise SystemExit(main())

