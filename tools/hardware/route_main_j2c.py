
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # The GND zone fill covers my CAN_GND vias even after refill. Why do other
    # vias work? Because the zone filler only clears around items it knows.
    # My vias ARE in the board... unless their net assignment makes them
    # "not connected" so the filler treats them as plain holes and should
    # clear MORE. Actual 0.0000 gap means fill covers via pad entirely -
    # impossible if filler saw it. Test: check zone's filled polys timestamp
    # vs board. Alternative: these violations might be against the zone
    # OUTLINE (not fill) - i.e. my vias sit ON the zone boundary? No...
    # Try: remove my 2 problem vias and route CAN_GND entirely on F.Cu,
    # avoiding both the GND track cluster (x 19.7-23.4, y 25.1-26.6) and
    # CAN_5V routes, by going around the RIGHT side of R3: down from J2
    # x=4.23 is blocked by CAN_5V horiz at y=27 (x spans 4.23-9.68).
    # But wait: CAN_5V L-route: vertical at x=1.5! Horizontal y=25.5 from
    # x=1.5 to 9.68. So x=4.23 below y=16.38 is clear until y=25.5 where the
    # horizontal crosses. J2 GND vertical must cross it or go around left at
    # x=0.7? Edge clearance min ~0.5 => x=0.75 with w=0.45 ok? tight.
    # Better: swap approach - route CAN_GND FIRST on the direct path
    # (x=4.23 down to y=29.5 etc), and reroute CAN_5V to not cross:
    # CAN_5V from J2 pad1 (4.23,16.38) can go RIGHT then UP-OVER? F1 pad1
    # at (9.68,25.5). Path: right from J2 pad1 along y=16.38 to x=8.0,
    # down x=8.0 to y=24.4, right to 9.68? The CANL track vertical at x=10.14
    # y=21.99..22.45 - our x=8.0 clear; CANH diag from (9.36,13.88)->(11.11,15.62)
    # is above y=16.38? It starts at y=13.88-15.62, above our start point.
    # Our horizontal y=16.38 from x=4.23 to 8.0: CANH tracks are at y<=17.44?
    # (13.26,17.44)->(10.14,20.55) starts x>=10.14 - clear of x<=8.
    # So CAN_5V: (4.23,16.38)->(8.0,16.38)->(8.0,24.4)->(9.68,24.4)->(9.68,25.5).
    # Then CAN_GND: (4.23,15.12) down x=4.23 to y=29.5 crosses CAN_5V horiz?
    # CAN_5V now has NO horizontal at y=25.5-27 near x=4.23 (it goes right at
    # y=16.38). Vertical x=4.23 from 15.12 down to 29.5: crosses CAN_5V
    # horizontal y=16.38 segment x 4.23..8.0 AT ITS START POINT (4.23,16.38)
    # which is J2 pad1 itself! Both nets meet at adjacent pads of J2 - the
    # vertical passes within pad radius. Shift GND vertical to x=3.4 first:
    seg(net_cangnd := board.FindNet("/CAN_GND"), 4.23, 15.12, 3.4, 15.12, 0.4)
    seg(board.FindNet("/CAN_GND"), 3.4, 15.12, 3.4, 29.5, 0.4)
    seg(board.FindNet("/CAN_GND"), 3.4, 29.5, 21.62, 29.5, 0.4)
    # riser to R3: cross GND diag at x~20.6,y~26.1? diag spans x19.68->21.09;
    # riser at x=21.62 > 21.09 - clear. But earlier crossing was with the
    # HORIZONTAL GND track (21.09,26.63)->(23.41,26.63). Riser x=21.62 crosses
    # it at y=26.63! That was the original problem. Approach R3 pad1 from
    # ABOVE instead: continue x=3.4? No - go right at y=24.6 (above the diag
    # start y=25.12): (3.4,24.6)? we're at bottom... simpler: from y=29.5 run
    # right only to x=18.9, up x=18.9 to y=24.6 (left of diag x-start 19.68),
    # right at y=24.6 to x=21.62, then up into pad (21.62,25.5).
    g = board.FindNet("/CAN_GND")
    def s(x1,y1,x2,y2,w=0.45,l=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(l); t.SetNet(g); board.Add(t)
    s(3.4, 29.5, 18.9, 29.5)
    s(18.9, 29.5, 18.9, 24.6)
    s(18.9, 24.6, 21.62, 24.6)
    s(21.62, 24.6, 21.62, 25.5)
    print("done")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

