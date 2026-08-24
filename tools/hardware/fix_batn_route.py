import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
for t in board.GetTracks():
    if str(t.m_Uuid.AsString()).startswith('5c768dca'):
        s = t.GetStart()
        t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(75.900), s.y))
        print('BATN start moved to 75.900')
        break
pcbnew.SaveBoard(sys.argv[1], board)
print('saved')
