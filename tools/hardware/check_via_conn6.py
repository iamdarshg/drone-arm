import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
for t in board.GetTracks():
    if type(t).__name__ != 'PCB_VIA':
        continue
    if t.GetNetname() != '/M2_BATP':
        continue
    p = t.GetPosition()
    x, y = round(pcbnew.ToMM(p.x),2), round(pcbnew.ToMM(p.y),2)
    hits = []
    for layer in [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]:
        if not t.IsOnLayer(layer):
            continue
        for z in board.Zones():
            if not z.GetLayerSet().Contains(layer) or not z.HasFilledPolysForLayer(layer):
                continue
            if z.HitTestFilledArea(layer, p, 0):
                hits.append(pcbnew.LayerName(layer))
                break
    print((x, y), 'connected_layers:', hits)
