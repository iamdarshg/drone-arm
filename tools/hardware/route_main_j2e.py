
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Clean slate for CAN_GND/CAN_5V routing near J2/F1/R3: remove ALL
    # F.Cu tracks on these two nets in the region x<25, then route with a
    # proper obstacle-aware plan.
    removed = added = 0
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or not t.IsOnLayer(pcbnew.F_Cu):
            continue
        if t.GetNetname() not in ("/CAN_5V", "/CAN_GND"):
            continue
        s, e = t.GetStart(), t.GetEnd()
        if max(pcbnew.ToMM(s.x), pcbnew.ToMM(e.x)) < 26:
            doomed.append(t)
    seen = set(); uniq = []
    for t in doomed:
        if id(t) not in seen:
            seen.add(id(t)); uniq.append(t)
    for t in uniq:
        board.Remove(t); removed += 1

    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")

    def seg(net, x1, y1, x2, y2, w=0.4):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w))
        t.SetLayer(pcbnew.F_Cu)
        t.SetNet(net)
        board.Add(t)

    # Known obstacles (F.Cu, from scans):
    # - J2 pads: pin1 CAN_5V (4.23,16.38), pin2 GND (4.23,15.12),
    #   MP pads (7.43,18.23),(7.43,10.78)
    # - CANH: (9.36,13.88)->(11.11,15.62)->(13.26,15.62); (13.26,17.44)->(10.14,20.55);
    #   (4.23,13.88)->(9.36,13.88)
    # - CANL: (11.16,14.43)->(14.13,14.43); (14.13,18)->(10.14,21.99); (10.14,21.99)->(10.14,22.45);
    #   (11.16,14.43)->(9.36,12.62)
    # - GND: y=21.5 x12-16; diag (19.68,25.21)->(21.09,26.63); horiz (21.09..23.41, 26.63)
    # - SYS_5V near D2 (18.73,25.5); track (17.8,23.24) diag len2.66; (17.73..? ,25.55)
    # - F1 pad1 (9.68,25.5), pad2 (12.48,25.5) with fused-out track to x=15.43
    #
    # Plan: CAN_5V J2->F1: go UP and over the top: J2 pad1 up x=4.23 to y=11.5
    # (clear of CANL start at (9.36,12.62)? our horizontal y=11.5 crosses
    # nothing: CANL vertical (11.16,14.43)-(9.36,12.62) tops at y=12.62 > 11.5 ok),
    # right along y=11.5 to x=9.68... wait CANH horizontal at y=13.88 x4.23-9.36
    # is below y=13.88; we're above at 11.5. Then down x=9.68 from y=11.5 to
    # pad F1? F1 is at y=25.5 - vertical x=9.68 crosses CANH diag (9.36,13.88)->
    # (11.11,15.62): crosses x=9.68 at y~14.3! Conflict. Route right instead at
    # top to x=17.0, down x=17.0 to y=24.4 (right of CANL/GND cluster, left of
    # SYS_5V stuff at x>=17.7?), left along y=24.4 to F1? SYS_5V track at
    # (17.73,25.55) area and diag (17.8,23.24). x=17.0 vertical from 11.5 to 24.4
    # might cross things but scan region x~17 wasn't fully listed.
    # SAFER: use B.Cu hop with vias placed in verified-clear spots.
    # Verified clear B.Cu gaps: y band (49.4,50.7) etc are far away. For short
    # hops: via A at (6.5,17.5)? Check B.Cu obstacles there: unknown. Let's just
    # do it and let DRC verify; iterate if needed.
    def via(x, y, net):
        vv = pcbnew.PCB_VIA(board)
        vv.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        vv.SetWidth(MM(0.8)); vv.SetDrill(MM(0.4))
        vv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        vv.SetNet(net); board.Add(vv)

    def segB(net, x1, y1, x2, y2, w=0.4):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(pcbnew.B_Cu); t.SetNet(net); board.Add(t)

    # CAN_GND: J2 pad2 -> left x=3.2 -> down to y=30.9 -> right to x=20.9 ->
    # up to y=27.6 -> right to x=24.0? no that's GND pad zone...
    # R3 pad1 approach from below-left: final riser at x=20.9 from 27.6 to 25.5,
    # then right 20.9->21.62 at y=25.5. Check crossing: GND diag (19.68,25.21)->
    # (21.09,26.63): riser x=20.9 crosses diag at y~26.4! Still conflict.
    # Riser must be LEFT of x=19.68 or RIGHT of x=23.41+margin. Right side has
    # R3 pad2 (GND net) at 24.54 and its tracks. So approach from LEFT: riser
    # x=19.2 from 29.5 up to 25.5, then right 19.2->21.62 at y=25.5. Crossings:
    # diag starts at (19.68,25.21): our horizontal y=25.5 from x=19.2 to 21.62
    # passes under diag start? diag at x=19.68 has y=25.21, going down-right to
    # (21.09,26.63). Our horizontal y=25.5 crosses the diagonal where diag-y=25.5
    # => x ~ 19.96. CONFLICT again. The diagonal blocks all left approaches at
    # y between 25.2-26.6. Approach at y=24.6 (above diag start 25.21): riser
    # x=19.2 from 29.5 to 24.6 crosses diag? diag min y=25.21 > 24.6? The diag
    # occupies y 25.21..26.63; our vertical spans 24.6..29.5 => crosses at
    # y=25.21-ish when diag-x=19.68 vs our x=19.2: diag at x=19.2? diag starts
    # AT x=19.68, so x=19.2 never touches it. Horizontal y=24.6 from 19.2 to
    # 21.62: above the diag (diag max y at x=19.68..21.09 is >=25.21>24.6).
    # Clear! But what about GND vertical (19.68, 25.12->25.21)? tiny, below 24.6.
    # And SYS_5V diag (17.8,23.24) length 2.66 -> endpoint ~(19.0?,25.5?)?
    # "Track [/SYS_5V] (17.8,23.2427) len 2.6555" direction unknown; earlier
    # shorting showed SYS_5V track near (18.676,25.55) & D2 pad2 (18.73,25.5).
    # So SYS_5V diag likely runs (17.8,23.24)->(18.68,25.55)? That crosses our
    # horizontal y=24.6 at x~18.35! We go from x=19.2 rightward - starts AFTER
    # 18.68. Riser x=19.2 from 29.5 to 24.6: does SYS_5V reach x=19.2? Its end
    # ~x=18.68 < 19.2. Clear!
    seg(g, 4.23, 15.12, 3.2, 15.12)
    seg(g, 3.2, 15.12, 3.2, 30.9)
    seg(g, 3.2, 30.9, 19.2, 30.9)
    seg(g, 19.2, 30.9, 19.2, 24.6)
    seg(g, 19.2, 24.6, 21.62, 24.6)
    seg(g, 21.62, 24.6, 21.62, 25.5)
    added += 6

    # CAN_5V: J2 pad1 -> right y=16.38 to x=8.0 -> down x=8.0 to y=27.3 ->
    # right to x=9.68? F1 pad at (9.68,25.5): come up: (8.0,27.3)->(9.68,27.3)?
    # simpler: down x=8.0 to 25.5 then right into F1 pad. Crossing check:
    # vertical x=8.0 y16.38->25.5: CANH horiz y=13.88 no; CANL vertical x=10.14
    # no; MP pad (7.43,18.23) radius ~0.7 => edge x~8.1? clearance tight!
    # shift to x=8.6? CANL diag (10.14,21.99)... x=8.6 clear of MP pad (edge
    # ~8.13). Use x=8.6.
    seg(v5, 4.23, 16.38, 8.6, 16.38)
    seg(v5, 8.6, 16.38, 8.6, 25.5)
    seg(v5, 8.6, 25.5, 9.68, 25.5)
    added += 3

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

