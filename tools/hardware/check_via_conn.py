import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
targets = [(58.388,116.137),(27.564,73.259)]
for t in board.GetTracks():
    if type(t).__name__ != 'PCB_VIA':
        continue
    p = t.GetPosition()
    x, y = round(pcbnew.ToMM(p.x),3), round(pcbnew.ToMM(p.y),3)
    if any(abs(x-a)<0.02 and abs(y-b)<0.02 for a,b in targets):
        net = t.GetNetname()
        print('via', (x,y), net)
        for layer in [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]:
            if not t.IsOnLayer(layer):
                continue
            # check zone overlap on this layer
            for z in board.Zones():
                if not z.GetLayerSet().Contains(layer):
                    continue
                if z.GetNetname() != net:
                    continue
                poly = z.Outline()
                pt = pcbnew.VECTOR2I(p.x, p.y)
                inside = poly.Collide(pt, 0) == 0
                if inside:
                    print('   on layer', pcbnew.LayerName(layer), 'inside same-net zone:', z.GetNetname(), 'priority', z.GetAssignedPriority())
