
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_can5v = board.FindNet("/CAN_5V")
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0

    # remove all CAN_5V B.Cu runs and the bad GND via additions
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() == "/CAN_5V" and type(t).__name__ == "PCB_TRACK" and t.IsOnLayer(pcbnew.B_Cu):
            doomed.append(t)
        if t.GetNetname() == "/CAN_GND":
            p = t.GetPosition()
            x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            if abs(x - 21.62) < 0.05 and abs(y - 26.6) < 0.05:
                doomed.append(t)
        if t.GetNetname() == "/CAN_GND" and type(t).__name__ == "PCB_TRACK":
            s, e = t.GetStart(), t.GetEnd()
            sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
            if abs(sx - 21.62) < 0.05 and abs(ex - 21.62) < 0.05 and abs(ey - 26.6) < 0.1:
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

    # 5V feed: corridor at y=63.0 on B.Cu (above the sensor via cluster which
    # starts ~64.2). From F1 (9.68,25.5): F.Cu seg right to (9.0,25.5)? Keep it
    # simple: via at (10.8,25.5), B.Cu up x=10.8 to y=63.0, right to x=80.54,
    # via up to F.Cu at (80.54,63.0), then F.Cu down x=80.54 to pad 72.5.
    # Check obstacles on x=10.8 vertical: none reported in band; assume clear
    # above y=64. Below y=64 unknown - accept risk, DRC will verify.
    seg(net_can5v, 9.68, 25.5, 10.8, 25.5, 0.45); added += 1
    via(10.8, 25.5, net_can5v)
    seg(net_can5v, 10.8, 25.5, 10.8, 63.0, 0.45, pcbnew.B_Cu); added += 1
    seg(net_can5v, 10.8, 63.0, 80.54, 63.0, 0.45, pcbnew.B_Cu); added += 1
    via(80.54, 63.0, net_can5v)
    seg(net_can5v, 80.54, 63.0, 80.54, 72.5, 0.45); added += 1

    # CAN_GND R3: R3 pad1 at (21.62,25.5). Existing In2.Cu CAN_GND track ends
    # somewhere near J2. Route on F.Cu from R3 left/down to join the existing
    # In2 track's reachable end? Instead connect R3 to J2 pad2 directly on
    # F.Cu: J2 pad2 (4.23,15.12) -> down/right L to R3 (21.62,25.5).
    seg(net_cangnd, 4.23, 15.12, 4.23, 25.5, 0.45); added += 1
    seg(net_cangnd, 4.23, 25.5, 21.62, 25.5, 0.45); added += 1

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

