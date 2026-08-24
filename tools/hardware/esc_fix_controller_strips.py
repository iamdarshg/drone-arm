
import sys

import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    removed_zones = added_zones = 0

    # M1 reference: controller strip at abs(16-22.6, 97.8-103.2) with cell
    # origin (3,43) => rel x 13-19.6, rel y 54.8-60.2
    # M1 cell origins: M1=(3,43), M2=(160,43), M3=(317,43),
    #                  M4=(3,143), M5=(160,143), M6=(317,143)
    
    for motor in [1, 2, 3]:
        dx = ((motor - 1) % 3) * 157
        dy = ((motor - 1) // 3) * 100
        cx0 = 3 + dx
        cy0 = 43 + dy
        batn_net_name = f"/M{motor}_BATN"
        net = board.FindNet(batn_net_name)
        if net is None:
            continue

        # Remove any F.Cu BATN zones in this motor that DON'T match the correct
        # controller strip position (rel 13-19.6, 54.8-60.2 => abs cx0+16..cx0+19.6? no)
        # Actually: abs positions are:
        #   M1: (16,97.8)-(22.6,103.2), origin(3,43) => rel(13,54.8)-(19.6,60.2)
        #   M4: (16,197.8)-(22.6,203.2), origin(3,143) => same rel!
        # So target abs for M{motor}: (cx0+13, cy0+54.8) to (cx0+19.6, cy0+60.2)
        target_x0 = cx0 + 13.0
        target_y0 = cy0 + 54.8
        target_x1 = cx0 + 19.6
        target_y1 = cy0 + 60.2

        # Check if a correctly-positioned zone exists
        has_correct = False
        to_remove = []
        for z in board.Zones():
            if z.GetNetname() == batn_net_name and z.GetLayerSet().Contains(pcbnew.F_Cu):
                bb = z.GetBoundingBox()
                zx = pcbnew.ToMM(bb.GetX())
                zy = pcbnew.ToMM(bb.GetY())
                zw = pcbnew.ToMM(bb.GetWidth())
                zh = pcbnew.ToMM(bb.GetHeight())
                if abs(zx - target_x0) < 2 and abs(zy - target_y0) < 2 and abs(zw - 6.6) < 2 and abs(zh - 5.4) < 2:
                    has_correct = True
                elif zx >= cx0 and zx < cx0 + 157:  # in this cell
                    if zh > 10 or zx > cx0 + 30:  # big rect or cap row zone
                        pass  # keep these
                    else:
                        to_remove.append(z)

        for z in to_remove:
            board.Remove(z)
            removed_zones += 1

        if not has_correct:
            zone = pcbnew.ZONE(board)
            zone.SetNet(net)
            zone.SetLayer(pcbnew.F_Cu)
            zone.SetLocalClearance(MM(0.3))
            zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
            outline = zone.Outline()
            outline.NewOutline()
            outline.Append(MM(target_x0), MM(target_y0))
            outline.Append(MM(target_x1), MM(target_y0))
            outline.Append(MM(target_x1), MM(target_y1))
            outline.Append(MM(target_x0), MM(target_y1))
            board.Add(zone)
            added_zones += 1
            print(f"M{motor}: added controller strip at ({target_x0},{target_y0})")

    pcbnew.SaveBoard(sys.argv[1], board)
    print(f"removed {removed_zones}, added {added_zones}")
    return 0


raise SystemExit(main())

