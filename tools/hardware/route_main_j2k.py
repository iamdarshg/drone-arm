
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    v5 = board.FindNet("/CAN_5V")
    g = board.FindNet("/CAN_GND")
    removed = 0
    # 1) remove the mystery long horizontal CAN_5V at y=16.38 (len 8.02 => to
    # x~12.25) - it's a duplicate from an earlier run that my "exists" check
    # didn't catch because its start x was 4.23 and end >5.9.
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ != "PCB_TRACK" or t.GetNetname() != "/CAN_5V":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(sy - 16.38) < 0.05 and abs(ey - 16.38) < 0.05:
            if abs(max(sx,ex) - min(sx,ex)) > 1.8:  # my clean one is 4.23->5.8 = 1.57
                doomed.append(t)
    for t in doomed:
        board.Remove(t); removed += 1

    # 2) remove the In2 CAN_GND runs crossing SYS_5V: the two segments at
    # y=27.6 (x 20.9->23.9) and the vertical/horiz at x/y 23.9/24.0/27.6,
    # plus via at (21.62,20.5). Then re-route R3 connection on B.Cu? No -
    # use F.Cu approach from BELOW with the corridor at x=24.2..24.3?
    # R3 pad2 GND at (24.54,25.5): its own track leaves right/up.
    # Actually check what's between x 23.41 (GND horiz end) and 24.04 (pad2
    # edge) at y=25.5-26.63: gap width ~0.63mm at y>25.5. Our track w=0.4 +
    # clearance 0.2 each side needs 0.8 - doesn't fit between them.
    # BUT we can cross GND horiz at y=26.63 ABOVE x=23.41+margin if we route
    # at x>=23.75... same tightness issue horizontally.
    # Give up on F.Cu; instead move the In2 crossing AWAY from SYS_5V:
    # SYS_5V In2 track is at y=25.55 constant spanning whole board. ANY
    # vertical crossing on In2 hits it. So do the hop on B.Cu but place vias
    # where the GND fill already has openings - i.e., co-locate with existing
    # via clusters of other nets. From earlier ESC work, filler clears fine
    # for normal vias; the problem here is mysterious, but note: the two
    # flagged vias are MINE; other vias don't get flagged. Difference: mine
    # have SetNet called AFTER Add? No, before Add in latest scripts...
    # Actually in route_main_j2j.py I did SetNet then Add - correct order.
    # Try instead: assign net by code: t.SetNetCode(g.GetNetCode()).
    # Also try adding the via FIRST then setting net after Add.
    def seg(net, pts, w=0.45, l=pcbnew.F_Cu):
        for i in range(len(pts)-1):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(MM(pts[i][0]), MM(pts[i][1])))
            t.SetEnd(pcbnew.VECTOR2I(MM(pts[i+1][0]), MM(pts[i+1][1])))
            t.SetWidth(MM(w)); t.SetLayer(l); t.SetNet(net); board.Add(t)

    def via(x, y, net):
        vv = pcbnew.PCB_VIA(board)
        vv.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        vv.SetWidth(MM(0.8)); vv.SetDrill(MM(0.4))
        vv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        vv.SetNet(net)
        board.Add(vv)
        vv.SetNet(net)   # set again AFTER Add
        return vv

    # remove old In2 runs + via at (21.62,20.5)
    for t in list(board.GetTracks()):
        if t.GetNetname() != "/CAN_GND":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if type(t).__name__ == "PCB_VIA":
            if abs(sx - 21.62) < 0.05 and abs(sy - 20.5) < 0.05:
                board.Remove(t); removed += 1
            continue
        if t.IsOnLayer(pcbnew.In2_Cu):
            board.Remove(t); removed += 1
        elif abs(sx - 21.62) < 0.05 and abs(ex - 21.62) < 0.05 and max(sy,ey) < 25.6 and min(sy,ey) > 19:
            board.Remove(t); removed += 1

    # New plan: CAN_GND stays on In2 but crosses SYS_5V at x=90 (far away)?
    # No - keep local. Instead route on In1.Cu! In1 has GND zone TOO.
    # OK final: use B.Cu hop with vias at (16.0,31.5) & (23.9,31.5) - far
    # below all obstacles; B.Cu obstacles there unknown but likely clear
    # (board bottom edge region). SYS_5V In2 crossing avoided since hop is
    # on B.Cu. Vias still pass through In1/B zones - the crux.
    # Test whether SetNet-after-Add fixes zone clearing.
    seg(g, [(2.6, 31.5), (16.0, 31.5)])
    via(16.0, 31.5, g)
    seg(g, [(16.0, 31.5), (23.9, 31.5)], 0.45, pcbnew.B_Cu)
    via(23.9, 31.5, g)
    seg(g, [(23.9, 31.5), (23.9, 25.5)])
    seg(g, [(23.9, 25.5), (21.62, 25.5)])
    added += 6

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

