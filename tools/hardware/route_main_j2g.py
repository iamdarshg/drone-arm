
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # The GND zone fill genuinely covers (16,27.6) on both B.Cu and In1.Cu.
    # Any through via there will always violate. So route CAN_GND entirely on
    # F.Cu + In2.Cu using BLIND/BURIED-style layer pairs? KiCad through vias
    # span F-B always. Check the board's allowed via pairs: if it's a 4-layer
    # F/In1/In2/B stack, a via from F to In2 isn't possible with simple vias.
    # Options: (a) add keepout/cutout in GND zones around my vias,
    # (b) find an F.Cu-only path.
    #
    # F.Cu path avoiding ALL obstacles for CAN_GND J2->R3:
    # Obstacles: CANH/CANL cluster right of x=9.3 above y<22.5; GND horiz
    # y=21.5 x12-16; GND diag (19.68,25.21)->(21.09,26.63); GND horiz
    # (21.09..23.41,26.63); SYS_5V near x17.7-18.7 y23.2-25.6; D2 pad (18.73,25.5).
    # R3 pad2 is GND at (24.54,25.5)! R3 pad1 CAN_GND at (21.62,25.5).
    # The GND tracks belong to R3 pad2's connection - they're between our
    # target and everything else at that height.
    # Path: go BELOW everything: y=31.5 horizontal, then approach R3 pad1
    # from below at x=21.62: vertical from 31.5 up to 25.5 crosses GND horiz
    # (21.09..23.41 @26.63) AT x=21.62! Unless... we stop the vertical BELOW
    # the horiz and jog LEFT then UP then RIGHT? The diag blocks left-up.
    # Gap analysis around the GND track pair: diag ends (21.09,26.63), horiz
    # starts (21.09,26.63). There's a wedge opening upward-right between
    # diag and horiz? No, they form a V pointing left; open side faces RIGHT
    # toward x>23.41. But R3 pad2 (GND) at 24.54 sits in that opening.
    # Between diag and SYS_5V/D2 stuff on the left at x~19-20: the diag's
    # upper-left region y>26.63,x<21.09 connects down-left. Our corridor at
    # y=29.3-31.5 reaches x up to ~24 freely (SYS_5V ends by y~25.6). From
    # (24.0,29.3) go UP to y=27.8, LEFT to x=22.4?? crosses nothing? GND horiz
    # at y=26.63 - we stay below it until... we need to reach pad at y=25.5
    # which is ABOVE the horiz. Crossing unavoidable on F.Cu left of x=23.41.
    # RIGHT of x=23.41: vertical x=24.0 from 29.3 up to 25.9, then LEFT at
    # y=25.9 to pad1 x=21.62: crosses R3 pad2 (24.54,25.5)? Pad edge x~24.04!
    # Our vertical at x=24.0 vs pad edge 24.04: clearance fail. x=23.9?
    # GND horiz track ends at 23.41; pad2 edge starts 24.04. Corridor
    # x 23.5..23.95 exists! Use x=23.7: vertical from y=29.3 up to y=25.5
    # (crosses NOTHING: horiz GND at y=26.63 spans x<=23.41 < 23.7-margin(0.2+
    # 0.175)=clear by 0.115mm... too tight? DRC clearance POWER class default
    # likely 0.2: gap = 23.7-0.175-23.41 = 0.115 < 0.2 FAIL.
    # Widen: vertical at x=23.75 => gap 0.165 still <0.2.
    # Hmm. Move the horizontal crossing point instead: cross ABOVE the horiz:
    # impossible without touching.
    #
    # SOLUTION: reroute the GND net itself! The offending GND tracks connect
    # R3 pad2 to somewhere. If I instead move MY route to In2 and accept vias,
    # I must add zone cutouts. KiCad keeps out via "keepout" zones or by
    # drawing rule areas. Simplest robust fix: add a small KEEPOUT zone
    # (no copper fill) rectangle over my two via locations on B.Cu and In1.Cu
    # GND zones -> forces fill to clear. Implement via ZONE with
    # SetIsKeepout(true) and fill-disable flags.
    g = board.FindNet("/CAN_GND")

    def seg(x1, y1, x2, y2, w=0.45, l=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(l); t.SetNet(g); board.Add(t)

    def via(x, y):
        vv = pcbnew.PCB_VIA(board)
        vv.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        vv.SetWidth(MM(0.8)); vv.SetDrill(MM(0.4))
        vv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        vv.SetNet(g); board.Add(vv)

    # Clean previous attempts in region
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() != "/CAN_GND" or type(t).__name__ == "PCB_VIA":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if max(sy, ey) > 26.8 or max(sx, ex) < 17:
            doomed.append(t)
    seen = set(); uniq = []
    for t in doomed:
        if id(t) not in seen:
            seen.add(id(t)); uniq.append(t)
    for t in uniq:
        board.Remove(t)
    print("cleaned", len(uniq))

    # Keepout helper
    def add_keepout(x, y, w, h, layers):
        k = pcbnew.ZONE(board)
        k.SetIsKeepout(True)
        k.SetDoNotAllowCopper(True)
        k.SetLayerSet(layers)
        outline = k.Outline()
        outline.NewOutline()
        outline.Append(MM(x - w/2), MM(y - h/2))
        outline.Append(MM(x + w/2), MM(y - h/2))
        outline.Append(MM(x + w/2), MM(y + h/2))
        outline.Append(MM(x - w/2), MM(y + h/2))
        board.Add(k)

    layers = pcbnew.LSET()
    layers.AddLayer(pcbnew.B_Cu)
    layers.AddLayer(pcbnew.In1_Cu)
    add_keepout(16.0, 27.6, 2.0, 2.0, layers)
    add_keepout(20.9, 27.6, 2.0, 2.0, layers)

    # Route: J2 pin2 down/left/below to (16,27.6) via hop to (20.9,27.6),
    # then up to R3 pad1.
    seg(4.23, 15.12, 2.6, 15.12)
    seg(2.6, 15.12, 2.6, 31.5)
    seg(2.6, 31.5, 16.0, 31.5)
    seg(16.0, 31.5, 16.0, 27.6)
    via(16.0, 27.6)
    seg(16.0, 27.6, 20.9, 27.6, 0.45, pcbnew.In2_Cu)
    via(20.9, 27.6)
    seg(20.9, 27.6, 20.9, 25.5)
    seg(20.9, 25.5, 21.62, 25.5)
    print("routed with keepouts")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

