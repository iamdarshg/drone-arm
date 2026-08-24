
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    net_can5v = board.FindNet("/CAN_5V")
    net_cangnd = board.FindNet("/CAN_GND")
    removed = added = 0

    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() not in ("/CAN_5V", "/CAN_GND"):
            continue
        if type(t).__name__ == "PCB_VIA":
            p = t.GetPosition()
            x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            # keep only the pre-existing dangling CAN_GND vias? They were flagged
            # dangling before my work; keep them to preserve original state.
            if abs(x - 22.3204) < 0.05 and abs(y - 24.2308) < 0.05:
                continue
            if abs(x - 5.4192) < 0.05 and abs(y - 14.575) < 0.05:
                continue
            doomed.append(t)
        elif type(t).__name__ == "PCB_TRACK":
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

    xs_gnd = [78.0, 87.0, 96.0, 105.0, 114.0, 123.0]
    xs_5v = [80.54, 89.54, 98.54, 107.54, 116.54, 125.54]

    # GND bus above pads at y=70.2 (F.Cu)
    gnd_y = 70.2
    seg(net_cangnd, xs_gnd[0], 72.5, xs_gnd[0], gnd_y, 0.45); added += 1
    for i in range(len(xs_gnd) - 1):
        seg(net_cangnd, xs_gnd[i], gnd_y, xs_gnd[i+1], gnd_y, 0.45)
        seg(net_cangnd, xs_gnd[i+1], gnd_y, xs_gnd[i+1], 72.5, 0.45)
        added += 2

    # 5V hops below pads y=73.75 w=0.25
    for i in range(len(xs_5v) - 1):
        x1, x2 = xs_5v[i], xs_5v[i+1]
        seg(net_can5v, x1, 72.5, x1, 73.75, 0.25)
        seg(net_can5v, x1, 73.75, x2, 73.75, 0.25)
        seg(net_can5v, x2, 73.75, x2, 72.5, 0.25)
        added += 3

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

