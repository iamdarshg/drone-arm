
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")
    added = removed = 0

    # Clean slate: remove ALL tracks/vias on these two nets
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() in ("/CAN_5V", "/CAN_GND"):
            if type(t).__name__ == "PCB_VIA":
                p = t.GetPosition()
                x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
                # keep only pre-existing dangling vias
                if (abs(x-22.3204)<0.05 and abs(y-24.2308)<0.05) or \
                   (abs(x-5.4192)<0.05 and abs(y-14.575)<0.05):
                    continue
            doomed.append(t)
    seen=set(); uniq=[]
    for t in doomed:
        if id(t) not in seen: seen.add(id(t)); uniq.append(t)
    for t in uniq:
        try: board.Remove(t); removed += 1
        except: pass

    def seg(net, x1, y1, x2, y2, w=0.4, l=pcbnew.F_Cu):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
        t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
        t.SetWidth(MM(w)); t.SetLayer(l); t.SetNet(net); board.Add(t)

    xs_gnd = [78.0, 87.0, 96.0, 105.0, 114.0, 123.0]
    xs_5v = [80.54, 89.54, 98.54, 107.54, 116.54, 125.54]

    # Header GND bus ABOVE pads at y=70.2 (F.Cu)
    for i in range(6):
        seg(g, xs_gnd[i], 72.5, xs_gnd[i], 70.2)
        added += 1
    for i in range(5):
        seg(g, xs_gnd[i], 70.2, xs_gnd[i+1], 70.2)
        added += 1

    # Header 5V chain BELOW pads using B.Cu (no GND zone conflict there)
    def viaB(x, y, net):
        vv = pcbnew.PCB_VIA(board)
        vv.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        vv.SetWidth(MM(0.7)); vv.SetDrill(MM(0.35))
        vv.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        vv.SetNet(net); board.Add(vv)

    # Drop each 5V pin to B.Cu, run bus on B.Cu at y=73.8, come back up
    for i in range(6):
        viaB(xs_5v[i], 72.5, v5)
        viaB(xs_5v[i], 74.8, v5)
        added += 2
    for i in range(5):
        seg(v5, xs_5v[i], 74.8, xs_5v[i+1], 74.8, 0.4, pcbnew.B_Cu)
        added += 1

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

