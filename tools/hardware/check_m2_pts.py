import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
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
            if 'F.Cu' not in layers:
                continue
            poly = z.Outline()
            n = poly.Outline(0).PointCount()
            pts = []
            for i in range(n):
                v = poly.Outline(0).CPoint(i)
                pts.append((round(pcbnew.ToMM(v.x),2), round(pcbnew.ToMM(v.y),2)))
            print('zone', str(z.m_Uuid.AsString())[:8], 'pts:', pts)
        break
