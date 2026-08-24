import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
# For each M2_BATP via, list which zones' OUTLINES contain it per layer
for t in board.GetTracks():
    if type(t).__name__ != 'PCB_VIA' or t.GetNetname() != '/M2_BATP':
        continue
    p = t.GetPosition()
    x, y = round(pcbnew.ToMM(p.x),2), round(pcbnew.ToMM(p.y),2)
    if abs(x-243.38)<0.05 and abs(y-59.85)<0.05:
        for z in board.Zones():
            if z.GetNetname() != '/M2_BATP':
                continue
            layers = [pcbnew.LayerName(l) for l in range(pcbnew.PCB_LAYER_ID_COUNT) if z.GetLayerSet().Contains(l)]
            poly = z.Outline()
            # point-in-outline test via Collide with tiny distance
            inside = poly.Collide(p, -10000) == False
            print('zone', str(z.m_Uuid.AsString())[:8], layers, 'outline contains:', inside, 'has fill:', z.IsFilled())
        break
