
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    for z in board.Zones():
        if z.GetNetname() == "/M1_BATN" and z.GetLayerSet().Contains(pcbnew.F_Cu):
            print("F.Cu M1_BATN already present:", str(z.m_Uuid.AsString())[:8])
            return 0
    net = board.FindNet("/M1_BATN")
    if net is None:
        print("net not found")
        return 1
    zone = pcbnew.ZONE(board)
    zone.SetNet(net)
    zone.SetLayer(pcbnew.F_Cu)
    zone.SetLocalClearance(MM(0.5))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    outline = zone.Outline()
    outline.NewOutline()
    for x, y in [(24.0, 107.0), (109.0, 107.0), (109.0, 141.0), (24.0, 141.0)]:
        outline.Append(MM(x), MM(y))
    board.Add(zone)
    pcbnew.SaveBoard(sys.argv[1], board)
    print("created M1_BATN F.Cu island (24,107)-(109,141); saved")
    return 0


raise SystemExit(main())

