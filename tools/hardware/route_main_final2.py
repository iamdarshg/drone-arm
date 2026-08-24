
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")
    removed = added = 0

    # 1) Fix the J2 pin1/pin2 clearance: my GND left-jog at (3.2,15.12) is too
    # close to pad1. Remove the jog segment; route GND straight DOWN from
    # pad2 at x=4.23? It would cross CAN_5V horizontal at y=33/35 region?
    # CAN_5V bottom run: (4.23,16.38)->(4.23,33)->(9.68,33)... vertical at
    # SAME x=4.23! Conflict with anything going down from pad2.
    # Instead: shift the CAN_5V bottom vertical LEFT to x=2.2 and keep CAN_GND
    # vertical at x=4.23. Check CAN_5V x=2.2 vs edge 0 => fine; vs GND vert
    # 4.23: gap 1.8mm ok. Bottom horizontals: CAN_5V at y=33.0 from x=2.2;
    # CAN_GND at y=35.2 from x=3.2 - parallel different y - fine. But wait,
    # earlier removal deleted bottom runs below y=33.9 for both nets; CAN_5V's
    # own y=33.0 run survived (min y = 16.38..33). Rebuild cleanly:

    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() not in ("/CAN_GND", "/CAN_5V"):
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        # remove everything in the J2/R3 local area for a clean rebuild
        if max(sx, ex) < 26 and max(sy, ey) < 34:
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

    def via(x, y, net):
        vv = pcbnew.PCB_VIA(board)
        vv.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        vv.SetWidth(MM(0.8))
        vv.SetDrill(MM(0.4))
        vv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        vv.SetNet(net)
        board.Add(vv)

    # CAN_5V: J2 pad1 down at x=2.2 (jog first), bottom y=33, up riser x=9.0,
    # jog to F1 pad.
    seg(v5, [(4.23,16.38),(2.2,16.38)])
    seg(v5, [(2.2,16.38),(2.2,33.0)])
    seg(v5, [(2.2,33.0),(9.0,33.0)])
    seg(v5, [(9.0,33.0),(9.0,26.2)])
    seg(v5, [(9.0,26.2),(9.68,26.2)])
    seg(v5, [(9.68,26.2),(9.68,25.5)])
    added += 6

    # CAN_GND: J2 pad2 straight down x=4.23 to y=35.2 (crosses nothing now:
    # CAN_5V moved to x=2.2), right to x=21.62, up into R3 pad1. The R3-area
    # GND escape tracks: diag (19.68,25.21)->(21.09,26.63), horiz
    # (21.09..23.41 @26.63): our riser x=21.62 from 35.2 UP to 25.5 crosses
    # horiz@26.63 at x=21.62 in range 21.09..23.41 -> CONFLICT!
    # Approach pad1 from ABOVE-LEFT instead: continue past... we can't cross.
    # Use the pre-existing via (22.3204,24.2308) which has proper fill
    # clearance: reach it from ABOVE: horizontal at y=24.0? From where? We're
    # below. Hmm: via is at y=24.23, ABOVE pad1(25.5). Our bus is BELOW.
    # Vertical must pass y=26.63 line somewhere. Only clear columns: x<19.68
    # or x>23.41(+margin) or between 24.04..24.29?? none good.
    # x>24.54: right of R3 entirely: vertical x=25.6 from 35.2 up to 26.63+
    # margin... then LEFT at y=27.0 to... still below the 26.63 line. The
    # line spans only x 21.09..23.41. Right of 23.41: no horiz track! But
    # diag (23.41,26.63)->(24.54,25.5) occupies that zone partially. At x=25.6
    # > 24.54: fully clear of all R3 tracks. Then go UP x=25.6 to y=24.0
    # (above everything), LEFT along y=24.0 to x=22.32 (the pre-existing via),
    # crossing over... at y=24.0 horizontal from 25.6 to 22.32: obstacles?
    # diag max y-end 26.63 no; SYS_5V stuff x<=18.7 no. Clear!
    # Via at (22.32,24.0)? Not the pre-existing one (that's at y=24.23 with
    # verified fill). Place OUR via exactly AT the pre-existing location
    # (22.32,24.23) - dedupe: if a via already exists there reuse it.
    existing = None
    for t in board.GetTracks():
        if type(t).__name__ == "PCB_VIA" and t.GetNetname() == "/CAN_GND":
            p = t.GetPosition()
            if abs(pcbnew.ToMM(p.x)-22.32)<0.05 and abs(pcbnew.ToMM(p.y)-24.23)<0.05:
                existing = t; break
    seg(g, [(4.23,15.12),(25.6,15.12)])   # NO! crosses J2 pads 3/4 (CANH/CANL
    # at y=13.88/12.62 - above 15.12, so horizontal y=15.12 passes just below
    # pad2.. wait pad2 IS at 15.12. Going RIGHT at y=15.12 from pad2: J2 pins
    # are vertically stacked at x=4.23; moving right immediately leaves the
    # connector. CANH/CANL pads also at x=4.23. So horizontal is fine!
    # But does it hit other things between x=4.23 and 25.6 at y=15.12?
    # CANH track (4.23,13.88)->(9.36,13.88) is at y=13.88 - 1.24mm above,
    # fine. OK.
    added += 1
    seg(g, [(25.6,15.12),(25.6,24.23)])
    added += 1
    seg(g, [(25.6,24.23),(22.32,24.23)])
    added += 1
    if existing is None:
        via(22.32, 24.23, g)
    else:
        print("reusing pre-existing via at (22.32,24.23)")
    # From that via, the pre-existing network continues (In2 etc).

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

