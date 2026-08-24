
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    removed = 0

    # 1) Fix the riser crossing the GND escape (24.54,25.5)->(27.27,25.5):
    # move riser x from 25.6 to 28.0 (right of the escape's end 27.27).
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() != "/CAN_GND":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(sx - 25.6) < 0.05 and abs(ex - 25.6) < 0.05 and max(sy,ey) > 30:
            doomed.append(t)
        elif abs(sy - 35.6) < 0.05 and abs(ey - 35.6) < 0.05:
            doomed.append(t)
        elif abs(sy - 24.23) < 0.05 and abs(ey - 24.23) < 0.05 and min(sx,ex) >= 22.32:
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

    # bottom y=35.6 extends to x=28.0; riser up at x=28.0; top horizontal
    # y=24.23 from 28.0 left to 22.32: crosses R3 pad2 escape horizontal
    # (24.54->27.27 @ y=25.5)? Our y=24.23 is ABOVE (smaller y) - parallel,
    # no cross. Crosses diag? diag is below too. OK!
    seg(g, [(1.2, 35.6), (28.0, 35.6)])
    seg(g, [(28.0, 35.6), (28.0, 24.23)])
    seg(g, [(28.0, 24.23), (22.32, 24.23)])
    print("removed", removed)
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

