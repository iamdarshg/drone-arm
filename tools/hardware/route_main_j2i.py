
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    # Duplicates accumulated across attempts. Full dedupe of CAN_GND/CAN_5V
    # copper in the region, then final canonical route.
    removed = 0
    doomed = []
    seen_seg = set()
    for t in list(board.GetTracks()):
        if t.GetNetname() not in ("/CAN_5V", "/CAN_GND"):
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        key = (t.GetNetname(), round(sx,2), round(sy,2), round(ex,2), round(ey,2), type(t).__name__)
        rkey = (t.GetNetname(), round(ex,2), round(ey,2), round(sx,2), round(sy,2), type(t).__name__)
        if type(t).__name__ == "PCB_VIA":
            key = (t.GetNetname(), round(sx,2), round(sy,2), "VIA")
            rkey = key
        if key in seen_seg or rkey in seen_seg:
            doomed.append(t)
            continue
        seen_seg.add(key)
    for t in doomed:
        board.Remove(t); removed += 1
    print("deduped", removed)
    g = board.FindNet("/CAN_GND")

    # Also remove the old leftover In2 run at (18.6,29.3) and F.Cu stub at
    # (20.9,25.5->27.6) that crosses the GND diag/horiz.
    doomed2 = []
    for t in list(board.GetTracks()):
        if t.GetNetname() != "/CAN_GND":
            continue
        s, e = t.GetStart(), t.GetEnd()
        sx, sy, ex, ey = pcbnew.ToMM(s.x), pcbnew.ToMM(s.y), pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)
        # old In2 horizontal near y=29.3
        if abs(sy - 29.3) < 0.05 and abs(ey - 29.3) < 0.05:
            doomed2.append(t)
        elif abs(sx - 18.6) < 0.05 and abs(ex - 18.6) < 0.05:
            doomed2.append(t)
        # F.Cu vertical x=20.9 crossing GND tracks (from 27.6 up to 25.5)
        elif abs(sx - 20.9) < 0.05 and abs(ex - 20.9) < 0.05 and min(sy,ey) < 26.7:
            doomed2.append(t)
        elif abs(sy - 27.6) < 0.05 and abs(ey - 27.6) < 0.05 and max(sx,ex) <= 20.9 and min(sx,ex) >= 16:
            doomed2.append(t)
        elif abs(sy - 25.5) < 0.05 and abs(ey - 25.5) < 0.05 and min(sx,ex) >= 20.4 and max(sx,ex) <= 21.62:
            doomed2.append(t)
        # vias at 16/20.9 y=27.6 duplicates handled above; keep ONE pair
    for t in doomed2:
        try:
            board.Remove(t); removed += 1
        except Exception:
            pass

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
        vv.SetNet(net); board.Add(vv)

    # Final CAN_GND: via hop at (16.0,27.6)->In2->(22.6,27.6) [RIGHT of the
    # GND horiz end 23.41? no: 22.6<23.41 still under it... use 23.9 which is
    # right of 23.41 with clearance], then up on In2 to y=25.0, via up at
    # (23.9,25.0)? R3 pad2 (GND) at (24.54,25.5): pad edge ~24.04; via at
    # 23.9 + halfwidth 0.4 = 24.3 overlaps pad! Use approach from ABOVE:
    # In2 to (21.62, 24.0), via, then F.Cu down into pad1 from above.
    seg(g, [(16.0, 27.6), (23.9, 27.6)], 0.45, pcbnew.In2_Cu)
    seg(g, [(23.9, 27.6), (23.9, 24.0)], 0.45, pcbnew.In2_Cu)
    seg(g, [(23.9, 24.0), (21.62, 24.0)], 0.45, pcbnew.In2_Cu)
    via(21.62, 24.0, g)
    seg(g, [(21.62, 24.0), (21.62, 25.5)])

    print("final route done; total removed", removed)
    pcbnew.SaveBoard(sys.argv[1], board)
    return 0


raise SystemExit(main())

