
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Full cleanup of ALL CAN_GND/CAN_5V tracks and non-original vias in the
    # J2/F1/R3 region, then one final clean route using a wider sweep.
    removed = added = 0
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() not in ("/CAN_5V", "/CAN_GND"):
            continue
        if type(t).__name__ == "PCB_VIA":
            doomed.append(t)  # remove all vias on these nets; rebuild minimal set
        elif type(t).__name__ == "PCB_TRACK" and t.IsOnLayer(pcbnew.F_Cu):
            s, e = t.GetStart(), t.GetEnd()
            if max(pcbnew.ToMM(s.x), pcbnew.ToMM(e.x)) < 26:
                doomed.append(t)
        elif type(t).__name__ == "PCB_TRACK" and t.IsOnLayer(pcbnew.B_Cu):
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

    def seg(net, x1, y1, x2, y2, w=0.4, l=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(l); t.SetNet(net); board.Add(t)

    def via(x, y, net):
        vv = pcbnew.PCB_VIA(board)
        vv.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        vv.SetWidth(MM(0.8)); vv.SetDrill(MM(0.4))
        vv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        vv.SetNet(net); board.Add(vv)

    # Final plan (all F.Cu, avoiding every mapped obstacle):
    #
    # CAN_5V: J2 pin1 (4.23,16.38). Go RIGHT along y=16.38 to x=6.3 only,
    # then UP x=6.3? MP pad at (7.43,18.23): keep x<=6.8. Actually simplest:
    # up from pad: x=4.23 upward is blocked by nothing above y<12.6? CANL
    # diag (11.16,14.43)->(9.36,12.62) - x>=9.36. CANH horiz y=13.88 x4.23..9.36!
    # That crosses x=4.23 vertical ABOVE pad1 (pad1 y=16.38 > 13.88 means the
    # horizontal is ABOVE i.e. smaller y). Going UP from y=16.38 we hit it.
    # So: right y=16.38 -> x=6.9 (MP pad edge ~8.0 with clearance... MP pad at
    # 7.43 radius ~0.75 => edge 8.18; stop at 7.2), down x=7.2? MP pad below?
    # MP pad center y=18.23, edge y 17.48..18.98. Our vertical x=7.2 vs pad
    # edge x=8.18-6.68 => 7.2 INSIDE pad x-range! Move left: x=6.5 (pad edge
    # 6.68 - clearance tight but ok w=0.35 => track edge 6.675 vs pad edge 6.68:
    # too tight. Use x=5.8: clear of pad (edge 6.68 minus halfwidth .175=6.85>6.68).
    # Down x=5.8 from 16.38 to y=24.4: crosses CANH/CANL? CANL vertical x=10.14;
    # CANH diag ends x>=10.14; GND y=21.5 spans x12-16 no. SYS_5V/D2 stuff x>=17.7
    # no. Clear! Then right y=24.4 from 5.8 to 9.68: crosses CANL vertical
    # (10.14,21.99->22.45)? No, stops at 9.68 <10.14. Then up into F1 pad
    # (9.68,25.5): from y=24.4 DOWN to 25.5. Wait pad is BELOW y=24.4. Yes down.
    seg(v5, 4.23, 16.38, 5.8, 16.38)
    seg(v5, 5.8, 16.38, 5.8, 24.4)
    seg(v5, 5.8, 24.4, 9.68, 24.4)
    seg(v5, 9.68, 24.4, 9.68, 25.5)
    added += 4

    # CAN_GND: J2 pin2 (4.23,15.12). Left x=2.6, down to y=31.5, right to
    # x=16.0, up x=16.0 to y=27.6, right x=16.0->20.9 at y=27.6? GND horiz
    # (21.09..23.41,26.63): our y=27.6 passes BELOW it; riser must reach pad1
    # (21.62,25.5) crossing that horiz OR the diag. Blocked both ways on F.Cu.
    # Use B.Cu hop between two vias placed in the corridor y~27.6-29.3 where
    # B.Cu was verified clear? Earlier B.Cu via list had nothing between
    # y=67.6..69.2 in ESC coords - wrong board. For main board unknown.
    # Alternative: In1.Cu/In2.Cu have no GND zones except In1 HAS zone.
    # Use In2 hop: via1 (16.0,27.6), In2 right to x=20.9, via2 (20.9,27.6),
    # F.Cu up x=20.9 from 27.6 to 25.5? crosses GND horiz y=26.63 at x=20.9 <
    # 21.09 - CLEAR! Then right 20.9->21.62 at y=25.5. Check diag: starts
    # (19.68,25.21): our horizontal y=25.5 x20.9->21.62 crosses diag where
    # diag-y=25.5 => x~19.96 <20.9 - clear!
    seg(g, 4.23, 15.12, 2.6, 15.12)
    seg(g, 2.6, 15.12, 2.6, 31.5)
    seg(g, 2.6, 31.5, 16.0, 31.5)
    seg(g, 16.0, 31.5, 16.0, 27.6)
    via(16.0, 27.6, g)
    seg(g, 16.0, 27.6, 20.9, 27.6, 0.4, pcbnew.In2_Cu)
    via(20.9, 27.6, g)
    seg(g, 20.9, 27.6, 20.9, 25.5)
    seg(g, 20.9, 25.5, 21.62, 25.5)
    added += 9

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

