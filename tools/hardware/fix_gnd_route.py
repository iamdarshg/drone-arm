
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0

    # Move the CAN_GND B.Cu run to In2.Cu (no GND zone there).
    doomed = []
    for t in list(board.GetTracks()):
        if type(t).__name__ == "PCB_TRACK" and t.GetNetname() == "/CAN_GND" and t.IsOnLayer(pcbnew.B_Cu):
            doomed.append(t)
        if type(t).__name__ == "PCB_VIA" and t.GetNetname() == "/CAN_GND":
            p = t.GetPosition()
            x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            if abs(x - 5.6) < 0.05 or abs(x - 21.62) < 0.05 and abs(y - 29.3) < 0.05:
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

    def via(x, y, net, pair=(pcbnew.F_Cu, pcbnew.B_Cu)):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        v.SetWidth(MM(0.8))
        v.SetDrill(MM(0.4))
        v.SetLayerPair(pair[0], pair[1])
        v.SetNet(net)
        board.Add(v)

    # Rebuild on In2.Cu: F.Cu stub from J2 pad2 -> via at (5.6,15.12) using
    # F.Cu<->In2.Cu layer pair? KiCad vias are through-hole (F-B). A through
    # via still crosses B.Cu GND zone mechanically - hole clearance applies.
    # Instead keep the whole run on F.Cu but avoid zone issue: zones are only
    # B.Cu/In1 so F.Cu is fine! The earlier F.Cu problem was crossing the
    # small GND track near (21,26.6). Route around it: go right at y=30.8.
    seg(net_cangnd, 4.23, 15.12, 4.23, 15.12, 0.45)  # zero-len guard removed below
    board.Remove(board.GetTracks()[-1])
    added -= 0

    # F.Cu: from J2 pad2 down... x=4.23 vertical crossed CAN_5V horizontal at
    # y=27? CAN_5V horiz is at y=25.5..27 spanning x 1.5..9.68. Crossing at
    # x=4.23,y~25.5-27 unavoidable on same layer. So use In2 with through-vias;
    # the via holes at (5.6,15.12)/(21.62,29.3) violate only because they pass
    # THROUGH B.Cu/In1 GND planes. That's normal for any via! The violation
    # says zone clearance 0.35 vs actual 0: means the zone fill didn't clear
    # around the via - stale fill. Refill should fix. But refill ran...
    # Actually these violations existed pre-refill; check after this save+refill.

    print(f"removed {removed}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

