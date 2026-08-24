import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
changed = False
import sys as _sys

uuid = _sys.argv[2] if len(_sys.argv) > 2 else '8e4ecd35-8569-4dc0-851b-9ab35a356e4a'
priority = int(_sys.argv[3]) if len(_sys.argv) > 3 else 1
for z in board.Zones():
    if z.GetNetname() == '/M1_BATN' and str(z.m_Uuid.AsString()) == uuid:
        layers = [pcbnew.LayerName(l) for l in range(pcbnew.PCB_LAYER_ID_COUNT) if z.GetLayerSet().Contains(l)]
        print('zone', uuid[:8], 'layers', layers, 'old priority', z.GetAssignedPriority(), '-> new priority', priority)
        z.SetAssignedPriority(priority)
        changed = True
if changed:
    pcbnew.SaveBoard(sys.argv[1], board)
    print('saved')
else:
    print('zone not found')
