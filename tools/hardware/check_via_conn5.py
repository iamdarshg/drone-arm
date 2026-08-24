import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
target_net = sys.argv[2]
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
    print('via', (x,y))
    for layer in [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]:
        if not t.IsOnLayer(layer):
            continue
        hits = []
        for z in board.Zones():
            if not z.GetLayerSet().Contains(layer):
                continue
            if not z.HasFilledPolysForLayer(layer):
                continue
            try:
                ok = z.HitTestFilledArea(layer, p, pcbnew.FromMM(0.30))
            except Exception as e:
                ok = f'err {e}'
            if ok:
                hits.append(z.GetNetname())
        print('   ', pcbnew.LayerName(layer), '->', hits)
