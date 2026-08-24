import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])

to_remove = []
for t in board.GetTracks():
    s, e = t.GetStart(), t.GetEnd()
    sx, sy = round(pcbnew.ToMM(s.x),3), round(pcbnew.ToMM(s.y),3)
    ex, ey = round(pcbnew.ToMM(e.x),3), round(pcbnew.ToMM(e.y),3)
    # zero-length stub from the bad trim
    if type(t).__name__ == 'PCB_TRACK' and abs(sx-79.373)<0.01 and abs(sy-87.597)<0.01 and abs(ex-79.373)<0.01:
        to_remove.append(t)
    # restore the 0.3mm M1_BATN run: extend its start back over DVDD/VGLS tracks to 78.223
    if type(t).__name__ == 'PCB_TRACK' and abs(sx-78.223)<0.01 and abs(sy-87.597)<0.01 and abs(ex-75.33)<0.01:
        t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(78.223), pcbnew.FromMM(87.597)))
        print('restored track start 78.223 ->', (sx,sy))
for t in to_remove:
    board.Remove(t)
    print('removed zero-length stub')
pcbnew.SaveBoard(sys.argv[1], board)
print('saved')
