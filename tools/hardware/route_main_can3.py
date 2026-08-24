
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_can5v = board.FindNet("/CAN_5V")
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0

    # remove the problematic additions: the 80.54,70 via + F.Cu stub and
    # the 22.32 via (it hit SYS_5V on In2).
    doomed = []
    for t in list(board.GetTracks()):
        p = t.GetPosition()
        x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
        is_via = type(t).__name__ == "PCB_VIA"
        if t.GetNetname() == "/CAN_5V":
            if abs(x - 80.54) < 0.05 and (abs(y - 70.0) < 0.05 or abs(y - 71.25) < 0.6):
                doomed.append(t)
        if t.GetNetname() == "/CAN_GND" and abs(x - 22.32) < 0.05 and abs(y - 24.79) < 0.05:
            doomed.append(t)
    seen = set()
    uniq = []
    for t in doomed:
        k = id(t)
        if k not in seen:
            seen.add(k)
            uniq.append(t)
    for t in uniq:
        board.Remove(t)
        removed += 1

    def seg(net, x1, y1, x2, y2, w, layer=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w))
        t.SetLayer(layer)
        t.SetNet(net)
        board.Add(t)

    # Re-route 5V feed: from J60 pad up to y=69.0 (above GND bus at 70.2),
    # then LEFT along y=69.0 on B.Cu? GND bus is F.Cu; crossing on B.Cu is fine.
    # Keep B.Cu run at different y so no same-layer conflict.
    def via(x, y, net):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        v.SetWidth(MM(0.8))
        v.SetDrill(MM(0.4))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(net)
        board.Add(v)

    seg(net_can5v, 80.54, 72.5, 80.54, 68.8, 0.45); added += 1
    via(80.54, 68.8, net_can5v)
    seg(net_can5v, 10.8, 68.8, 80.54, 68.8, 0.45, pcbnew.B_Cu); added += 1
    seg(net_can5v, 10.8, 25.5, 10.8, 68.8, 0.45, pcbnew.B_Cu); added += 1
    seg(net_can5v, 9.68, 25.5, 10.8, 25.5, 0.45); added += 1
    via(10.8, 25.5, net_can5v)

    # CAN_GND R3 connection: route on In2 via a via placed away from SYS_5V.
    # Check where SYS_5V runs: it crossed at (22.32,24.79). Try via at (21.62,26.4)
    # below R3 instead.
    seg(net_cangnd, 21.62, 25.5, 21.62, 26.6, 0.45); added += 1
    via(21.62, 26.6, net_cangnd)

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

