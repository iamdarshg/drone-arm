
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    removed = added = 0

    # GND escape chain: (24.54,25.5)->(27.27,25.5)->(29.56,23.21)->(32.95,23.21)
    # The fence around R3: diag+horiz below-left, escape above-right.
    # Gap between escape start (29.56,23.21) and SYS_5V/D2 cluster (ends x~18.7)?
    # Big open area at x 19..29, y 20..22? U51 oscillator stuff sits x28-33,
    # y 20-24. So corridor: riser at x=26.8 (between 24.54-escape and U51),
    # from bottom y=35.6 up to y=22.4, then LEFT along y=22.4 to the
    # pre-existing via column... but via is at (22.32,24.23); horizontal
    # y=22.4 from 26.8 to 22.32: crosses SYS_5V F.Cu diag (17.8,23.24)->
    # (18.68,25.55)? x<=18.68 - our run stops at 22.32 - clear! Crosses GND
    # small vertical (19.68,25.12->25.21)? No (y range differs). CANL vert
    # (10.14,...) no. Clear!
    # But riser x=26.8 crosses GND horiz (24.54->27.27 @ y=25.5): x=26.8 IS in
    # [24.54,27.27] -> CONFLICT. Riser must be right of 27.27: x=28.5. Then
    # top horizontal y=22.4 from 28.5 to 22.32 crosses U51_OSC1 track
    # (31.56,21.93)... no wait that's x>=29.9; our y=22.4 from 28.5 leftward -
    # U51 osc pads at (28.6,22.5),(29.56,22.5): pad edges y~22.0-23.0, x
    # ~28.2-29.9. Our horizontal passes THROUGH them! Move corridor up:
    # y=21.3? OSC track (29.16..31.56 @21.93): still close. This area is a minefield.
    #
    # FINAL DECISION: approach from BELOW-LEFT through the gap between the
    # GND diag (ends x=21.09) and the CAN_5V riser (x=9.0): i.e., riser at
    # x=20.5 from bottom y=35.2 up to y=24.23, then RIGHT to via (22.32).
    # Check riser x=20.5 crossings: GND diag (19.68,25.21)->(21.09,26.63):
    # at x=20.5, diag-y = 25.21+(20.5-19.68)/1.41*1.42 = 26.03 -> CROSSES.
    # Riser at x=19.3 (left of diag start 19.68): crosses nothing on diag;
    # SYS_5V diag (17.8,23.24)->(18.68,25.55): max x=18.68 <19.3 - clear!
    # GND tiny stubs at x=19.68 - clear. Top horizontal y=24.23 from 19.3 to
    # 22.32: crosses diag? diag spans y 25.21..26.63 - we're at 24.23 above it
    # - clear! Crosses SYS_5V? max y 25.55 - no. Clear!!
    # Bottom: horizontal y=35.2 from x=3.2 to 19.3, riser x=19.3 up to 24.23.
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() != "/CAN_GND":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(sy - 35.6) < 0.05 and abs(ey - 35.6) < 0.05 and max(sx,ex) > 25:
            doomed.append(t)
        elif abs(sx - 31.0) < 0.05 and abs(ex - 31.0) < 0.05:
            doomed.append(t)
        elif abs(sy - 22.0) < 0.05 and abs(ey - 22.0) < 0.05:
            doomed.append(t)
        elif abs(sx - 22.32) < 0.05 and abs(ex - 22.32) < 0.05 and min(sy,ey) > 22.05 and max(sy,ey) < 24.25:
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

    seg(g, [(3.2, 35.2), (19.3, 35.2)])
    seg(g, [(19.3, 35.2), (19.3, 24.23)])
    seg(g, [(19.3, 24.23), (22.32, 24.23)])
    added += 3
    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

