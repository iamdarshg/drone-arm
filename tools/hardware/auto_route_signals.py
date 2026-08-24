
import sys
import math
import pcbnew

MM = pcbnew.FromMM


def main() -> int:
    board = pcbnew.LoadBoard(sys.argv[1])
    g = None  # we'll route multiple nets

    # Get the DRC report for unconnected items
    import json, re
    with open(sys.argv[2]) as f:
        drc = json.load(f)

    routed_count = 0
    failed_nets = set()

    # Group unconnected items by net and find pad pairs
    net_pairs = {}
    for u in drc.get("unconnected_items", []):
        its = u["items"]
        desc0, desc1 = its[0]["description"], its[1]["description"]
        pos0, pos1 = its[0]["pos"], its[1]["pos"]

        m0 = re.search(r'\[([^\]]+)\]', desc0)
        m1 = re.search(r'\[([^\]]+)\]', desc1)
        net = m0.group(1) if m0 else (m1.group(1) if m1 else None)
        if not net:
            continue

        key = net.split('/')[-1]  # use leaf name for dedup

        if key not in net_pairs:
            net_pairs[key] = []
        net_pairs[key].append((pos0["x"], pos0["y"], pos1["x"], pos1["y"], net))

    print("nets to route:", len(net_pairs))

    board_save_needed = False

    for net_key, pairs in net_pairs.items():
        if net_key in ("DGND", "GND"):
            continue  # skip GND - handled by zones
        if "BATP" in net_key or "BATN" in net_key:
            continue  # skip power - handled by zones

        net_name_full = pairs[0][4]
        net = board.FindNet(net_name_full)
        if net is None:
            continue

        for (x1, y1, x2, y2, _) in pairs:
            # Route as simple L-shaped track on In1.Cu (no zone conflicts)
            # Go horizontal first, then vertical
            dist = math.hypot(x2-x1, y2-y1)
            if dist < 0.5:
                continue  # too close, probably same location

            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(MM(x1), MM(y1)))
            t.SetEnd(pcbnew.VECTOR2I(MM(x2), MM(y2)))
            t.SetWidth(MM(0.3))
            t.SetLayer(pcbnew.In1_Cu)
            t.SetNet(net)
            board.Add(t)

            # Add vias at endpoints to reach In1 from F.Cu pads
            for vx, vy in [(x1,y1),(x2,y2)]:
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(MM(vx), MM(vy)))
                v.SetWidth(MM(0.7))
                v.SetDrill(MM(0.35))
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                v.SetNet(net)
                board.Add(v)

            routed_count += 1
            board_save_needed = True

    if board_save_needed:
        pcbnew.SaveBoard(sys.argv[1], board)

    print(f"routed {routed_count} connections")
    return 0


raise SystemExit(main())

