import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
z_small = None
for z in board.Zones():
    if z.GetNetname() == '/M1_BATN' and str(z.m_Uuid.AsString()) == 'd33edddc-c4f7-4e2c-9f14-4bd7e5364cde':
        z_small = z
        break
if z_small is None:
    print('already removed')
    raise SystemExit(0)
layers = [pcbnew.LayerName(l) for l in range(pcbnew.PCB_LAYER_ID_COUNT) if z_small.GetLayerSet().Contains(l)]
print('removing zone on layers', layers)
board.Remove(z_small)
pcbnew.SaveBoard(sys.argv[1], board)
print('saved')
