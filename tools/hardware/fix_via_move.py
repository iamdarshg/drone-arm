import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
for t in board.GetTracks():
    if type(t).__name__ != 'PCB_VIA':
        continue
    if str(t.m_Uuid.AsString()).startswith('30366851'):
        t.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(19.60), pcbnew.FromMM(70.69)))
        print('via moved to (19.60, 70.69)')
        break
pcbnew.SaveBoard(sys.argv[1], board)
print('saved')
