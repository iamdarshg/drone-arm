
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")
    removed = added = 0

    # 1) Fix CAN_5V MP-pad clearance: shift vertical from x=5.8 to x=6.0?
    # No - violation is pad MP (7.43,18.23) vs horizontal y=16.38. Pad edge
    # top ~17.48; horizontal at 16.38 with halfwidth .175 => bottom edge
    # 16.555; gap = 17.48-16.555=0.925 > 0.2 fine. The crossing item is the
    # VERTICAL x=5.8? Distance from pad center x7.43 to track x5.8 = 1.63 -
    # clear. Wait, the violation pairs pad MP with the HORIZONTAL track
    # (length 8.02 = full run 4.23->12.? hmm length 8.02 means it spans to
    # x=12.25!). My horizontal was only to 5.8... unless an old duplicate
    # remains: (4.23,16.38)->(8.6,16.38) len 4.37 and (4.23,16.38)->(12.25)?
    # Remove any CAN_5V F.Cu track crossing x>7 near y~16-19.
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() != "/CAN_5V" or type(t).__name__ != "PCB_TRACK":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(sy - 16.38) < 0.05 and abs(ey - 16.38) < 0.05:
            if max(sx, ex) > 7.0 or abs(max(sx,ex) - min(sx,ex)) > 2.0:
                doomed.append(t)
    for t in doomed:
        board.Remove(t); removed += 1

    def seg(net, pts, w=0.45, l=pcbnew.F_Cu):
        for i in range(len(pts)-1):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(MM(pts[i][0]), MM(pts[i][1])))
            t.SetEnd(pcbnew.VECTOR2I(MM(pts[i+1][0]), MM(pts[i+1][1])))
            t.SetWidth(MM(w)); t.SetLayer(l); t.SetNet(net); board.Add(t)

    def via(x, y, net):
        vv = pcbnew.PCB_VIA(board)
        vv.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        vv.SetWidth(MM(0.8))
        vv.SetDrill(MM(0.4))
        vv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        vv.SetNet(net)
        board.Add(vv)

    # re-add clean short horizontal + new vertical path avoiding MP pad:
    # down at x=5.8 is clear of MP (edge 6.68). Route: (4.23,16.38)->(5.8,
    # 16.38)->(5.8,24.4)->(9.68,24.4)->(9.68,25.5).
    exists_h = False; exists_v = False; exists_b = False; exists_c = False
    for t in list(board.GetTracks()):
        if t.GetNetname() != "/CAN_5V" or type(t).__name__ != "PCB_TRACK":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(sy-16.38)<0.05 and abs(ey-16.38)<0.05 and max(sx,ex)<=5.9:
            exists_h = True
        if abs(sx-5.8)<0.05 and abs(ex-5.8)<0.05:
            exists_v = True
        if abs(sy-24.4)<0.05 and abs(ey-24.4)<0.05:
            exists_b = True
        if abs(sx-9.68)<0.05 and abs(ex-9.68)<0.05 and max(sy,ey)>24:
            exists_c = True
    seg(v5, [(4.23,16.38),(5.8,16.38)])
    seg(v5, [(5.8,16.38),(5.8,24.4)])
    seg(v5, [(5.8,24.4),(9.68,24.4)])
    seg(v5, [(9.68,24.4),(9.68,25.5)])
    added += 4

    # 2) CAN_GND In2 crossing SYS_5V at y=27.6 x=23.9: reroute In2 segment to
    # cross SYS_5V perpendicularly anywhere - crossing In2 vs In2? SYS_5V on
    # In2 IS a different net: real conflict! Move the In2 run lower: y=30.5?
    # SYS_5V spans which y? It's a long track (83mm) at y=25.55 constant?
    # pos (100.77,25.55) suggests horizontal line y=25.55 across the board.
    # Then my In2 verticals CROSS it inevitably. Use B.Cu for the hop instead
    # but vias violate GND zones... UNLESS zone filler clears them properly.
    # Earlier via-zone violations showed fill covering via completely - that
    # indicates fills are STALE relative to new vias. force_refill ran BEFORE
    # these last routes. Refill now happened too (refill_zones). Yet still
    # violating? These 2 remaining violations are at (21.62,24) only - the NEW
    # via added AFTER the refill in same script run. The refill we just ran
    # SHOULD have fixed it. Check: maybe HitTest area uses stale data because
    # SaveBoard wrote but DRC reads fresh... The DRC after refill STILL flags
    # it. So the filler refuses to cut into its own GND fill for a foreign-net
    # via?? No - other nets' vias exist everywhere.
    # Actually wait: maybe the filler DID clear it and DRC compares against
    # the zone OUTLINE+fill correctly but clearance 0.35 comes from netclass
    # HIGH_CURRENT etc. actual=0.35 exactly equals required -> borderline!
    # "actual 0.3500 mm"? earlier messages said actual 0.0000. Let me just
    # move this via pair onto the existing corridor used by other nets: shift
    # the whole In2 detour up by 3mm so via lands at (21.62,21.0)? That region
    # may have its own obstacles.
    # Simplest robust fix: delete the via-hop entirely; connect R3 pad1 on
    # In2 directly under R3: R3 pads are SMD on F.Cu though...
    # Alternative: accept In2 route but move crossing point far from 'pos'
    # reported - no, crossing is inherent.
    # FINAL approach: put the whole CAN_GND J2->R3 link on In2 EXCEPT tiny
    # F.Cu ends, with vias placed at (2.0,15.12) and (23.9,15.12)-ish along
    # the TOP of the board where GND planes also exist... same issue.
    #
    # Pragmatic fix: the two zone-clearance violations at via (21.62,24):
    # nudge via to sit exactly ON an area where fill already has an opening -
    # e.g., near other nets' vias. From ESC experience, refills DO clear.
    # Hypothesis: my refill script Fill(board.Zones()) doesn't include
    # keepout/knockout zones but does include GND zones - should work.
    # Try moving via to (22.6,24.0) and re-refill with force flag.
    doomed2 = []
    for t in list(board.GetTracks()):
        if type(t).__name__ == "PCB_VIA" and t.GetNetname() == "/CAN_GND":
            p = t.GetPosition()
            x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            if abs(x - 21.62) < 0.05 and abs(y - 24.0) < 0.05:
                doomed2.append(t)
        if type(t).__name__ == "PCB_TRACK" and t.GetNetname() == "/CAN_GND":
            s, e = t.GetStart(), t.GetEnd()
            sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
            if abs(ey - 24.0) < 0.01 and abs(sy - 24.0) < 0.01 and abs(ex - 21.62) < 0.01 and abs(sx - 23.9) < 0.01:
                doomed2.append(t)
            elif abs(sy - 24.0) < 0.01 and abs(ey - 24.0) < 0.01 and abs(sx - 21.62) < 0.01 and abs(ex - 23.9) < 0.01:
                doomed2.append(t)
    for t in doomed2:
        board.Remove(t); removed += 1
    seg(g, [(23.9, 27.6), (23.9, 20.5)], 0.45, pcbnew.In2_Cu)
    seg(g, [(23.9, 20.5), (21.62, 20.5)], 0.45, pcbnew.In2_Cu)
    via(21.62, 20.5, g)
    seg(g, [(21.62, 20.5), (21.62, 25.5)])

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

