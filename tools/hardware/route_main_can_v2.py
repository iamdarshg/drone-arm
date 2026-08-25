
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = board.FindNet("/CAN_GND")
    v5 = board.FindNet("/CAN_5V")
    removed = added = 0

    # Remove ALL existing CAN_GND/CAN_5V tracks and non-original vias
    doomed = []
    for t in list(board.GetTracks()):
        if t.GetNetname() not in ("/CAN_5V", "/CAN_GND"):
            continue
        if type(t).__name__ == "PCB_VIA":
            p = t.GetPosition()
            x, y = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            # keep only the two original vias
            if not ((abs(x-22.3204)<0.05 and abs(y-24.2308)<0.05) or \
                    (abs(x-5.4192)<0.05 and abs(y-14.575)<0.05)):
                doomed.append(t)
        else:
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

    # GND bus: horizontal at y=70.2 connecting all GND pins via vertical stubs
    for x in xs_gnd:
        seg(g, x, 72.5, x, 70.2)
        added += 1
    for i in range(5):
        seg(g, xs_gnd[i], 70.2, xs_gnd[i+1], 70.2)
        added += 1

    # 5V chain: use B.Cu with vias at each pin, bus at y=74.0 on B.Cu
    def via(x, y, net):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(MM(x), MM(y)))
        v.SetWidth(MM(0.7)); v.SetDrill(MM(0.35))
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(net); board.Add(v)

    for x in xs_5v:
        seg(v5, x, 72.5, x, 73.8, 0.3)     # F.Cu stub down
        via(x, 74.0, v5)                     # via to B.Cu
        added += 2
    for i in range(5):
        seg(v5, xs_5v[i], 74.3, xs_5v[i+1], 74.3, 0.4, pcbnew.B_Cu)
        added += 1
    # connect B.Cu segments to vias
    for x in xs_5v:
        seg(v5, x, 74.0, x, 74.3, 0.4, pcbnew.B_Cu)
        added += 1

    print(f"removed {removed}, added {added}")
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

