
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    added = 0
    for motor in range(2, 7):
        x0 = 3 + ((motor - 1) % 3) * 157
        y0 = 43 + ((motor - 1) // 3) * 100
        net = board.FindNet(f"/M{motor}_BATN")
        if net is None:
            continue
        # Match M1's extra vias at rel positions (18,53.5) and (56,19)
        for rx, ry in [(18.0, 53.5), (56.0, 19.0)]:
            # check no existing via here
            exists = False
            for t in board.GetTracks():
                if type(t).__name__ != "PCB_VIA" or t.GetNetname() != net.GetNetname():
                    continue
                p = t.GetPosition()
                if abs(pcbnew.ToMM(p.x) - (x0+rx)) < 0.05 and abs(pcbnew.ToMM(p.y) - (y0+ry)) < 0.05:
                    exists = True
                    break
            if not exists:
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(MM(x0+rx), MM(y0+ry)))
                v.SetWidth(MM(1.0))
                v.SetDrill(MM(0.5))
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetNet(net)
                board.Add(v)
                added += 1
                print(f"M{motor}: added via at rel({rx},{ry})")
    pcbnew.SaveBoard(sys.argv[1], board)
    print(f"total added: {added}")
    return 0


raise SystemExit(main())

