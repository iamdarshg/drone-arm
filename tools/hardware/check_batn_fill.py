import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
# sample M1_BATN dangling vias against B.Cu zone fills
import re
samples = [(99.625,116.65),(98.275,118),(66.375,116.65),(82.325,128)]
for x, y in samples:
    p = pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))
    for z in board.Zones():
        if z.GetNetname() != '/M1_BATN' or not z.GetLayerSet().Contains(pcbnew.B_Cu):
            continue
        ok = z.HitTestFilledArea(pcbnew.B_Cu, p, 0)
        print((x,y), str(z.m_Uuid.AsString())[:8], 'prio', z.GetAssignedPriority(), 'fill hit:', bool(ok))
