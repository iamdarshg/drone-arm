import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
target_net = sys.argv[2] if len(sys.argv) > 2 else '/M2_BATP'
count = 0
for t in board.GetTracks():
    if type(t).__name__ != 'PCB_VIA':
        continue
    if t.GetNetname() != target_net:
        continue
    p = t.GetPosition()
    x, y = round(pcbnew.ToMM(p.x),3), round(pcbnew.ToMM(p.y),3)
    count += 1
    if count > 4:
        break
    print('via', (x,y), 'net', target_net, 'drill', pcbnew.ToMM(t.GetDrillValue()))
    for layer in [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]:
        if not t.IsOnLayer(layer):
            continue
        # zone fill containment check at via center with small clearance probe
        inside_any = False
        for z in board.Zones():
            if not z.GetLayerSet().Contains(layer) or z.GetNetname() != target_net:
                continue
            poly = z.Outline()
            # use IsFilled to test actual copper: sample the filled polygons
            filled = z.GetFilledPolysList(layer)
            contains = filled.Contains(p, pcbnew.ALL_LAYERS if False else layer, -pcbnew.FromMM(0.35))
            print('   layer', pcbnew.LayerName(layer), 'zone fill contains(with -0.35mm shrink):', contains)
