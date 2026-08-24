
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")
    removed = 0

    # The horizontal y=15.12 route hits the CAN transceiver U50 tracks.
    # Replace with a route that goes DOWN first (below all J2-area stuff)
    # then right along y=33.5 (below everything, above bottom edge 37? board
    # is 130x74.52 so plenty of room), then up at x=25.6 to y=24.23, left to
    # the pre-existing via. Vertical x=25.6 from 33.5 to 24.2: obstacles?
    # GND diag at (22.24,55.35)... no. SYS_5V ends ~x18.7. Likely clear.
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() != "/CAN_GND":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(ey - 15.12) < 0.05 and abs(sy - 15.12) < 0.05 and max(sx,ex) > 5.0:
            doomed.append(t)
        elif abs(sy - 24.23) < 0.05 and abs(ey - 24.23) < 0.05 and min(sx,ex) > 22.4:
            doomed.append(t)
        elif abs(sx - 25.6) < 0.05 and abs(ex - 25.6) < 0.05 and abs(max(sy,ey) - 15.12) < 11:
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

    seg(g, [(4.23,15.12),(2.6,15.12)])
    seg(g, [(2.6,15.12),(2.6,33.8)])
    seg(g, [(2.6,33.8),(25.6,33.8)])
    seg(g, [(25.6,33.8),(25.6,24.23)])
    seg(g, [(25.6,24.23),(22.32,24.23)])
    print("removed", removed)
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

