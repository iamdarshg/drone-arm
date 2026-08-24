import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
for z in board.Zones():
    if z.GetNetname() == '/M1_BATN':
        layers = [pcbnew.LayerName(l) for l in range(pcbnew.PCB_LAYER_ID_COUNT) if z.GetLayerSet().Contains(l)]
        bb = z.GetBoundingBox()
        print('zone', str(z.m_Uuid.AsString())[:8], 'layers', layers, 'prio', z.GetAssignedPriority(),
              'bbox x', round(pcbnew.ToMM(bb.GetX()),1), '-', round(pcbnew.ToMM(bb.GetX()+bb.GetWidth()),1),
              'y', round(pcbnew.ToMM(bb.GetY()),1), '-', round(pcbnew.ToMM(bb.GetY()+bb.GetHeight()),1))
