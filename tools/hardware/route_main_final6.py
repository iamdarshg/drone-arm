
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    removed = added = 0
    # GND escape continues beyond 27.27! It goes (24.54,25.5)->(27.27,25.5)
    # and apparently further up-right. Move riser to x=30 and check what's
    # there... just try: remove riser+top horiz, rebuild at x=30.
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() != "/CAN_GND":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(sx - 28.0) < 0.05 and abs(ex - 28.0) < 0.05:
            doomed.append(t)
        elif abs(sy - 35.6) < 0.05 and abs(ey - 35.6) < 0.05 and max(sx,ex) > 25:
            doomed.append(t)
        elif abs(sy - 24.23) < 0.05 and abs(ey - 24.23) < 0.05 and min(sx,ex) >= 22.3:
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

    seg(g, [(1.2, 35.6), (31.0, 35.6)])
    seg(g, [(31.0, 35.6), (31.0, 22.0)])
    seg(g, [(31.0, 22.0), (22.32, 22.0)])
    seg(g, [(22.32, 22.0), (22.32, 24.23)])
    added += 4
    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

