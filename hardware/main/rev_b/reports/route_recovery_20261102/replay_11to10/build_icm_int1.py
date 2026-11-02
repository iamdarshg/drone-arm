#!/usr/bin/env python3
"""Route ICM_INT1 from U10 to U20 with a coordinated local decoupler escape."""
import pathlib
import sys
import pcbnew

source, output = map(pathlib.Path, sys.argv[1:3])
board = pcbnew.LoadBoard(str(source))
mm, to = pcbnew.FromMM, pcbnew.ToMM
point = lambda x, y: pcbnew.VECTOR2I(mm(x), mm(y))
coords = lambda p: (to(p.x), to(p.y))


def remove_tracks(net, layer, segments):
    code = board.FindNet(net).GetNetCode()
    for item in list(board.GetTracks()):
        if isinstance(item, pcbnew.PCB_VIA) or item.GetNetCode() != code or item.GetLayer() != layer:
            continue
        a, b = coords(item.GetStart()), coords(item.GetEnd())
        if any((all(abs(a[i] - x[i]) < 0.002 for i in (0, 1))
                and all(abs(b[i] - y[i]) < 0.002 for i in (0, 1)))
               or (all(abs(b[i] - x[i]) < 0.002 for i in (0, 1))
                   and all(abs(a[i] - y[i]) < 0.002 for i in (0, 1)))
               for x, y in segments):
            board.Remove(item)


def segment(net, a, b, layer, width=0.25):
    item = pcbnew.PCB_TRACK(board)
    item.SetStart(point(*a)); item.SetEnd(point(*b)); item.SetLayer(layer)
    item.SetWidth(mm(width)); item.SetNetCode(board.FindNet(net).GetNetCode())
    board.Add(item)


def through_via(net, xy, diameter=0.6, drill=0.3):
    item = pcbnew.PCB_VIA(board)
    item.SetPosition(point(*xy)); item.SetDrill(mm(drill))
    item.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    for layer in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
        item.SetWidth(layer, mm(diameter))
    item.SetNetCode(board.FindNet(net).GetNetCode()); board.Add(item)


# Move C11 1 mm north to create a via corridor, preserving its net assignments.
c11 = next(fp for fp in board.GetFootprints() if fp.GetReference() == "C11")
pos = c11.GetPosition()
c11.SetPosition(point(to(pos.x), to(pos.y) - 1.0))

# Reconnect the moved C11 /1V1 and GND lands and move the 3V3_ADC In2 chord.
remove_tracks("/1V1", pcbnew.F_Cu, [((50.49, 45.97), (51.52, 46.8))])
segment("/1V1", (50.49, 45.97), (51.52, 45.8), pcbnew.F_Cu)
remove_tracks("/3V3_ADC", pcbnew.In2_Cu, [((53.1801, 46.0513), (50.7806, 48.4508))])
points = [(53.1801, 46.0513), (52.7, 45.55), (52.0, 45.55), (50.7806, 48.4508)]
for a, b in zip(points, points[1:]):
    segment("/3V3_ADC", a, b, pcbnew.In2_Cu)
remove_tracks("/GND", pcbnew.F_Cu, [
    ((52.3078, 44.5622), (52.3078, 46.8278)),
    ((52.3078, 46.8278), (52.48, 46.8)),
])
points = [(52.3078, 44.5622), (52.3078, 45.25), (52.48, 45.8)]
for a, b in zip(points, points[1:]):
    segment("/GND", a, b, pcbnew.F_Cu)

# U10 pad 2 to U20 pad 4, routed on B.Cu around the sensor and MCU fanout.
through_via("/ICM_INT1", (52.8, 46.6))
segment("/ICM_INT1", (54.05, 46.6), (52.8, 46.6), pcbnew.F_Cu, 0.2)
through_via("/ICM_INT1", (47.2, 84.0))
points = [
    (52.8, 46.6), (52.3, 47.0), (52.3, 49.6), (52.45, 50.5),
    (52.45, 55.5), (50.0, 55.5), (50.0, 80.5), (49.3, 80.5),
    (49.3, 84.0), (47.2, 84.0),
]
for a, b in zip(points, points[1:]):
    segment("/ICM_INT1", a, b, pcbnew.B_Cu, 0.2)
segment("/ICM_INT1", (47.2, 84.0), (48.8375, 83.75), pcbnew.F_Cu, 0.2)
pcbnew.SaveBoard(str(output), board)
print(f"saved {output}")
