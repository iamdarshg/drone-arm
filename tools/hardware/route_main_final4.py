
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")
    removed = 0

    # Remove ALL leftover F.Cu CAN_GND/CAN_5V tracks in the J2 region and
    # rebuild ONE final time with fully separated corridors:
    #   CAN_GND: x=1.2 vertical (left margin), bottom y=35.6
    #   CAN_5V:  x=4.23->down x=4.23? No - crosses GND horiz at y=35.6 if
    #            both go right. Stagger: CAN_5V turns right at y=33.0 (ABOVE
    #            the GND horizontal), runs right to x=9.0. GND continues
    #            down past y=33 only at x=1.2 (left of CAN_5V's x=4.23
    #            vertical start? no - CAN_5V vertical IS at x=4.23 from 16.38
    #            down to 33.0; GND vertical at x=1.2 from 15.12 down to 35.6;
    #            they never touch. GND horizontal y=35.6 from x=1.2 to 25.6:
    #            passes UNDER CAN_5V's y=33 horizontal (different y, parallel).
    #            Riser x=25.6 up from 35.6 to 24.23: crosses CAN_5V horizontal
    #            y=33.0 spanning x=2.2..9.0? No - 25.6 > 9.0. Crosses CAN_5V's
    #            other segments? The (9.0,33)->(9.0,26.2) riser is left. Clear!
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() not in ("/CAN_GND","/CAN_5V"):
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if max(sx,ex) < 27 and max(sy,ey) < 36.5 and t.IsOnLayer(pcbnew.F_Cu):
            doomed.append(t)
    seen=set(); uniq=[]
    for t in doomed:
        if id(t) not in seen: seen.add(id(t)); uniq.append(t)
    for t in uniq:
        try: board.Remove(t); removed += 1
        except: pass

    def seg(net, pts, w=0.45):
        for i in range(len(pts)-1):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(MM(pts[i][0]), MM(pts[i][1])))
            t.SetEnd(pcbnew.VECTOR2I(MM(pts[i+1][0]), MM(pts[i+1][1])))
            t.SetWidth(MM(w)); t.SetLayer(pcbnew.F_Cu); t.SetNet(net); board.Add(t)

    # CAN_5V (upper corridor)
    seg(v5, [(4.23,16.38),(4.23,16.38)])
    board.Remove(board.GetTracks()[-1])  # remove zero-length
    seg(v5, [(4.23,16.38),(4.23,16.38)])
    board.Remove(board.GetTracks()[-1])
    seg(v5, [(4.23,16.38),(4.23,33.0)])
    seg(v5, [(4.23,33.0),(9.0,33.0)])
    seg(v5, [(9.0,33.0),(9.0,26.2)])
    seg(v5, [(9.0,26.2),(9.68,26.2)])
    seg(v5, [(9.68,26.2),(9.68,25.5)])

    # CAN_GND (lower corridor)
    seg(g, [(4.23,15.12),(1.2,15.12)])
    seg(g, [(1.2,15.12),(1.2,35.6)])
    seg(g, [(1.2,35.6),(25.6,35.6)])
    seg(g, [(25.6,35.6),(25.6,24.23)])
    seg(g, [(25.6,24.23),(22.32,24.23)])

    print("removed", removed)
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

