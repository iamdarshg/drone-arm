import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
for t in board.GetTracks():
    if type(t).__name__ != 'PCB_TRACK':
        continue
    s, e = t.GetStart(), t.GetEnd()
    sx, sy = round(pcbnew.ToMM(s.x),3), round(pcbnew.ToMM(s.y),3)
    ex, ey = round(pcbnew.ToMM(e.x),3), round(pcbnew.ToMM(e.y),3)
    for name, net in [('BATN','/M1_BATN'),('DVDD','/Motor 1 power cell/M1_DVDD'),('VGLS','/Motor 1 power cell/M1_VGLS')]:
        if t.GetNetname() == net and 74 < max(sx,ex) < 80 and 86 < max(sy,ey) < 89.5:
            print(name, str(t.m_Uuid.AsString())[:8], 'start', (sx,sy), 'end', (ex,ey), 'w', pcbnew.ToMM(t.GetWidth()), 'layer', t.GetLayerName())
