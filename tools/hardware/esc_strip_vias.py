
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    added = removed = 0

    # Remove old dangling vias near the strip (rel 18,53.5)
    for motor in [1, 2, 3]:
        dx = ((motor-1)%3)*157
        dy = ((motor-1)//3)*100
        cx0 = 3+dx; cy0 = 43+dy
        batn_net_name = f"/M{motor}_BATN"
        net = board.FindNet(batn_net_name)

        # Remove via at rel(18,53.5) if it exists
        for t in list(board.GetTracks()):
            if type(t).__name__ != "PCB_VIA" or t.GetNetname() != batn_net_name:
                continue
            p = t.GetPosition()
            vx, vy = pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)
            rx, ry = vx-cx0, vy-cy0
            if abs(rx - 18) < 0.05 and abs(ry - 53.5) < 0.05:
                board.Remove(t)
                removed += 1

        # Add two vias INSIDE the strip fill (center area, clear of pads)
        # Strip is at rel(13-19.6, 52.8-60.2) after extension
        for vx_rel, vy_rel in [(15.0, 56.0), (18.5, 59.0)]:
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(MM(cx0+vx_rel), MM(cy0+vy_rel)))
            v.SetWidth(MM(0.8))
            v.SetDrill(MM(0.4))
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            v.SetNet(net)
            board.Add(v)
            added += 1
            print(f"M{motor}: added via at rel({vx_rel},{vy_rel})")

    pcbnew.SaveBoard(sys.argv[1], board)
    print(f"removed {removed}, added {added} vias")
    return 0


raise SystemExit(main())

