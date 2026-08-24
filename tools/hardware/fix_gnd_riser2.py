
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0
    # R3 pad2 (GND) at 24.54 - approach R3 pad1 from BELOW-LEFT instead:
    # horizontal y=29.3 to x=20.2, then up x=20.2 to y=26.9, right to x=21.62
    # pad? Pad1 is at (21.62,25.5) with pad size ~1.0; approach from left:
    # up x=20.2 to y=25.5 then right to 21.62 - crosses GND track diagonal?
    # GND diag: (19.68,25.21)->(21.09,26.63). Our riser x=20.2 crosses it at
    # y~25.7. Conflict again. Use via hop to B.Cu... but B.Cu has GND zone.
    # In2.Cu has no zones! Use In2 with through vias - the earlier zone
    # violations on vias were because fill was stale? They persisted after
    # refill. Actually those violations were hole_clearance vs ZONE FILL -
    # meaning the fill didn't shrink around via. That happens when zone has
    # "solid" connection and via is same net... but CAN_GND is different net.
    # The GND plane must clear 0.35 around a 0.8mm via = needs 0.75mm gap.
    # DRC said actual 0.0000 => fill covered it entirely. Suspicious. Maybe
    # those specific coordinates sit inside a filled region that predates.
    # Try: remove riser, end horizontal at x=19.0 (left of GND tracks), via
    # to In2, In2 right/up, via back up near R3 from below-left at (20.4,24.0),
    # then short F.Cu hop right into pad.
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() != "/CAN_GND":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        if abs(ey - 25.5) < 0.01 and abs(sy - 25.5) < 0.01 and abs(ex - 21.62) < 0.05:
            doomed.append(t)
        elif abs(sx - 24.2) < 0.05 and abs(ex - 24.2) < 0.05:
            doomed.append(t)
        elif abs(sy - 29.3) < 0.05 and abs(ey - 29.3) < 0.05 and abs(ex - 24.2) < 0.05:
            doomed.append(t)
    seen = set(); uniq = []
    for t in doomed:
        if id(t) not in seen:
            seen.add(id(t)); uniq.append(t)
    for t in uniq:
        board.Remove(t); removed += 1

    def seg(net, x1, y1, x2, y2, w, layer=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w))
        t.SetLayer(layer)
        t.SetNet(net)
        board.Add(t)

    def via(x, y, net):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        v.SetWidth(MM(0.8))
        v.SetDrill(MM(0.4))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(net)
        board.Add(v)

    # horizontal already ends at 21.62->? we removed pieces; re-run from
    # existing (5.6..?) Let's just add: from wherever horiz ended (x~21.62?)
    # Simplest: add new segments assuming horiz y=29.3 spans 5.6..21.62 still.
    # Extend: 21.62->19.0 at y=29.3? No wait we removed only riser + maybe more.
    seg(net_cangnd, 21.62, 29.3, 18.6, 29.3, 0.45); added += 1
    via(18.6, 29.3, net_cangnd)
    seg(net_cangnd, 18.6, 29.3, 18.6, 23.8, 0.45, pcbnew.In2_Cu); added += 1
    seg(net_cangnd, 18.6, 23.8, 20.4, 23.8, 0.45, pcbnew.In2_Cu); added += 1
    via(20.4, 23.8, net_cangnd)
    seg(net_cangnd, 20.4, 23.8, 20.4, 25.5, 0.45); added += 1
    seg(net_cangnd, 20.4, 25.5, 21.62, 25.5, 0.45); added += 1

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

