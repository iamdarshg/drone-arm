import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
for z in board.Zones():
    if z.GetNetname() == '/M1_BATP':
        layers = [pcbnew.LayerName(l) for l in range(pcbnew.PCB_LAYER_ID_COUNT) if z.GetLayerSet().Contains(l)]
        print('zone', str(z.m_Uuid.AsString())[:8], 'layers', layers, 'priority', z.GetAssignedPriority())
        poly = z.Outline()
        for o in range(poly.OutlineCount()):
            outline = poly.Outline(o)
            n = outline.PointCount()
            xs = []
            ys = []
            for i in range(n):
                v = outline.CPoint(i)
                xs.append(round(pcbnew.ToMM(v.x), 2))
                ys.append(round(pcbnew.ToMM(v.y), 2))
            print('  outline', o, 'pts', n, 'x', min(xs), '->', max(xs), 'y', min(ys), '->', max(ys))
