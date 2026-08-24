import sys
import pcbnew

board = pcbnew.LoadBoard(sys.argv[1])
target_net = sys.argv[2] if len(sys.argv) > 2 else '/M2_BATP'
count = 0
for t in board.GetTracks():
    if type(t).__name__ != 'PCB_VIA':
        continue
    if t.GetNetname() != target_net:
        continue
    p = t.GetPosition()
    x, y = round(pcbnew.ToMM(p.x),3), round(pcbnew.ToMM(p.y),3)
    count += 1
    if count > 5:
        break
    info = []
    for layer in [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]:
        if not t.IsOnLayer(layer):
            continue
        near_track = None
        best = None
        for tr in board.GetTracks():
            if tr is t or not tr.IsOnLayer(layer):
                continue
            d = min((tr.GetStart() - p).EuclideanNorm(), (tr.GetEnd() - p).EuclideanNorm())
            if best is None or d < best:
                best = d
                near_track = tr
        if near_track and best is not None and best < pcbnew.FromMM(1.5):
            info.append((pcbnew.LayerName(layer), round(pcbnew.ToMM(best),2), near_track.GetNetname()))
    print('via', (x,y), 'net', target_net)
    for i in info:
        print('   ', i)
