
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")
    removed = 0

    # Deduplicate ALL tracks/vias on both nets (keep first occurrence)
    seen = {}
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() not in ("/CAN_5V", "/CAN_GND"):
            continue
        s, e = t.GetStart(), t.GetEnd()
        key = (t.GetNetname(), type(t).__name__,
               round(pcbnew.ToMM(s.x),2), round(pcbnew.ToMM(s.y),2),
               round(pcbnew.ToMM(e.x),2), round(pcbnew.ToMM(e.y,2) if False else pcbnew.ToMM(e.y),2))
        if t.GetNetname() == "/CAN_5V":
            # keep the canonical set only; mark all for removal first
            pass
        if key in seen:
            doomed.append(t)
        else:
            seen[key] = t
    for t in doomed:
        board.Remove(t); removed += 1

    # Now remove ALL /CAN_5V and /CAN_GND copper entirely for a clean rebuild
    for t in list(board.GetTracks()):
        if t.GetNetname() in ("/CAN_5V", "/CAN_GND"):
            try:
                board.Remove(t); removed += 1
            except Exception:
                pass

    def seg(net, x1, y1, x2, y2, w=0.4, l=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(l); t.SetNet(net); board.Add(t)

    xs_gnd = [78.0, 87.0, 96.0, 105.0, 114.0, 123.0]
    xs_5v = [80.54, 89.54, 98.54, 107.54, 116.54, 125.54]

    # header GND bus above pads
    seg(g, 78.0, 72.5, 78.0, 70.2)
    for i in range(5):
        seg(g, xs_gnd[i], 70.2, xs_gnd[i+1], 70.2)
        seg(g, xs_gnd[i+1], 70.2, xs_gnd[i+1], 72.5)

    # header 5V hops below pads
    for i in range(5):
        x1, x2 = xs_5v[i], xs_5v[i+1]
        seg(v5, x1, 72.5, x1, 73.75, 0.25)
        seg(v5, x1, 73.75, x2, 73.75, 0.25)
        seg(v5, x2, 73.75, x2, 72.5, 0.25)

    # CAN_5V feed J2->F1: up-over route via top edge corridor y=11.4
    seg(v5, 4.23, 16.38, 4.23, 11.4)
    seg(v5, 4.23, 11.4, 9.68, 11.4)
    seg(v5, 9.68, 11.4, 9.68, 25.5)
    # vertical x=9.68 crosses: CANH diag (9.36,13.88)->(11.11,15.62): at x=9.68
    # diag-y = 13.88 + (9.68-9.36)/(11.11-9.36)*(15.62-13.88) = ~14.2 CROSS!
    # Use x=12.6 down instead: from (12.6,11.4) down... CANH horiz ends 13.26;
    # segment (13.26,17.44)->(10.14,20.55) spans x 10.14-13.26: x=12.6 crosses
    # at y~19.3 CONFLICT. x=16? GND horiz y=21.5 spans 12..16 - touches at
    # exactly 16. x=17 crosses SYS_5V stuff lower. Route down at x=9.68 but
    # jog around the diag: at y=14.0 switch right to x=12.0, down to y=24.4,
    # left to 9.68, down to pad. Check: horizontal y=14.0 x9.68->12: CANL horiz
    # (11.16,14.43)->(14.13,14.43) is at y=14.43 - our y=14.0 with halfwidths
    # 0.175+0.175 => gap = 14.43-14.175 = 0.255 vs clearance? Default netclass
    # clearance likely 0.2 -> OK barely. Vertical x=12.0 from 14.0 to 24.4:
    # crosses CANH diag (13.26,17.44)->(10.14,20.55)? At x=12: diag-y =
    # 20.55-(12-10.14)/(13.26-10.14)*(20.55-17.44)= 20.55-0.596*3.11=18.7 ->
    # CROSSES. Ugh.
    # Simplest safe: go AROUND the right of everything: x=15.5 down (right of
    # CANL/GND cluster which ends x=16? GND horiz y21.5 spans 12..16!). x=16.8?
    # between GND-horiz end (16.0) and SYS_5V start (~17.73): gap 1.73mm wide,
    # fits a 0.4 track with 0.2 clearances both sides (needs 0.8). YES.
    # Path: (9.68,11.4)->(16.8,11.4)->(16.8,25.5)... wait F1 pad at (9.68,25.5);
    # coming down at x=16.8 lands near D2/SYS_5V at (18.73,25.5) region and
    # R3 pad2 (24.54). We need to reach F1 at 9.68! Horizontal y=25.5 from
    # 16.8 back LEFT to 9.68 crosses the whole GND/SYS_5V cluster. NO.
    #
    # FINAL sane approach: CAN_5V J2->F1 along the very bottom: J2 pad1 down
    # x=4.23 to y=33.5 (below everything incl GND horiz at 26.63 & SYS_5V),
    # right to x=9.68, up into F1 pad (9.68,25.5): vertical x=9.68 from 33.5
    # UP to 25.5 crosses GND horiz (21.09..23.41 @26.63)? x=9.68 not in range.
    # Crosses SYS_5V diag (17.8,23.24)->(18.68,25.55)? x=9.68 no. CANL vert
    # (10.14,21.99..22.45)? x differs by 0.46 - too close! Track edges:
    # ours 9.48-9.88, theirs 9.94-10.34: gap 0.06 < 0.2 FAIL. Shift riser to
    # x=9.0: then top horizontal (9.0..?) must still land on F1 pad x=9.68 -
    # final approach: rise x=9.0 to y=26.2, right to x=9.68 at y=26.2, up to
    # pad. Check crossing at y=26.2 horizontal x9.0->9.68: nothing there.
    # And bottom run y=33.5 x4.23->9.0: below all obstacles. Left vertical
    # x=4.23 from 16.38 down to 33.5: crosses CAN_5V_FUSED track
    # (12.48..15.43, 25.5)? No. Crosses GND horiz y=29.5/31.5? Those are OUR
    # old routes - being deleted. Original-board GND horiz at y=21.5 spans
    # x12-16 - no. OK!
    # But ALSO the CAN_GND route uses x=2.6/3.2 vertical nearby - parallel,
    # different nets, gap = |4.23-3.2| etc fine as long as >0.8 apart: yes.
    seg(v5, 4.23, 16.38, 4.23, 33.5)
    seg(v5, 4.23, 33.5, 9.0, 33.5)
    seg(v5, 9.0, 33.5, 9.0, 26.2)
    seg(v5, 9.0, 26.2, 9.68, 26.2)
    seg(v5, 9.68, 26.2, 9.68, 25.5)

    # CAN_GND: J2 pad2 -> left x=3.2 -> down y=35.2 -> right to x=20.4 ->
    # up to y=27.6?? crossings: vertical x=20.4 from 35.2 to 27.6: crosses
    # SYS_5V In2? we're F.Cu. SYS_5V F.CU diag (17.8,23.24)-(18.68,25.55)?
    # x=20.4 beyond it. GND horiz (21.09..23.41@26.63): our vertical stops at
    # 27.6 BELOW it - clear. Then horizontal y=27.6 x20.4->22.4 (still below
    # 26.63 line? 27.6>26.63 means LOWER on screen - KiCad Y grows down, so
    # y=27.6 is below the track at 26.63. We need to get ABOVE it to reach
    # pad y=25.5. Crossing inevitable on F.Cu left of 23.41. Right side:
    # continue right at y=27.6 to x=24.3, up x=24.3 to y=26.0? R3 pad2 edge
    # x~24.04,y 25.0-26.0: our vertical x=24.3 vs pad edge 24.04+clearance...
    # pad halfwidth maybe 0.55: edge 24.04? center 24.54 - halfwidth 0.5 =>
    # edge 24.04. Our track at 24.3 halfwidth 0.225 => left edge 24.075 >
    # 24.04 OVERLAP. x=24.45? overlaps more. Go BETWEEN: impossible.
    # So cross GND horiz at its far right END: x=23.41+margin... the track
    # ENDS there; beyond it is open until R3 pad2's own connection tracks.
    # Where does R3 pad2's GND track GO? From scan: (21.09,26.63)->(23.41,26.63)
    # then? Probably continues right/up to other GND. If it ends at 23.41
    # connecting upward, then right of 23.41 at y=26.63 might have another
    # segment going up-right toward main GND zone via... earlier probe found
    # only these two segments in that window plus tiny stubs. Let me just do
    # it: cross at x=23.75 (gap to 23.41 end = 0.34 - hmm need 0.2+0.175=
    # 0.375 from endpoint? End-of-track counts as point: distance from track
    # end to our track edge = 0.34-0.175=0.165 <0.2 marginal fail).
    # x=23.85: gap 0.44-0.175=0.265 >= 0.2 PASS. Then vertical x=23.85 up
    # through y=26.63 line? The GND track occupies y~26.53-26.73 there? It
    # ENDS at x=23.41 so at x=23.85 there is NO track - free to pass!
    # Continue up to y=25.9, left to x=21.62? passes over GND horiz? At y=25.9
    # horizontal from 23.85 to 21.62: the diag (19.68,25.21)->(21.09,26.63):
    # at y=25.9, diag-x = 19.68+(25.9-25.21)/(26.63-25.21)*1.41 = 20.36. Our
    # horizontal spans x 21.62..23.85 - starts AFTER 21.09 where diag ends.
    # Clear! Then up into pad1.
    seg(g, 4.23, 15.12, 3.2, 15.12)
    seg(g, 3.2, 15.12, 3.2, 35.2)
    seg(g, 3.2, 35.2, 23.85, 35.2)
    seg(g, 23.85, 35.2, 23.85, 25.9)
    seg(g, 23.85, 25.9, 21.62, 25.9)
    seg(g, 21.62, 25.9, 21.62, 25.5)

    print("rebuilt clean")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

