#!/usr/bin/env python3
"""Rebuild local /1V1 and GND traces after the decoupler-row shift."""
import argparse
import math
import pcbnew

def mm(value):
    return pcbnew.FromMM(value)

def xy(point):
    return pcbnew.VECTOR2I(mm(point[0]), mm(point[1]))

def close(a, b, tol=0.002):
    return math.dist(a, b) < tol

def add_track(board, net_name, start, end, layer=pcbnew.F_Cu, width=0.25):
    track = pcbnew.PCB_TRACK(board)
    track.SetStart(xy(start))
    track.SetEnd(xy(end))
    track.SetLayer(layer)
    track.SetWidth(mm(width))
    track.SetNetCode(board.FindNet(net_name).GetNetCode())
    board.Add(track)

parser = argparse.ArgumentParser()
parser.add_argument("input_board")
parser.add_argument("output_board")
args = parser.parse_args()
board = pcbnew.LoadBoard(args.input_board)
for reference in ("C39", "C40", "C41"):
    fp = next(fp for fp in board.GetFootprints() if fp.GetReference() == reference)
    if abs(pcbnew.ToMM(fp.GetPosition().y) - 56.7) > 0.001:
        raise SystemExit(f"{reference} is not at the pinned 56.7 mm row")

remove_1v1 = [
    ((53.52, 56.5), (54.0822, 57.0622)),
    ((54.0822, 57.0622), (54.8187, 57.0622)),
    ((54.8187, 57.0622), (55.4814, 56.3995)),
    ((55.4814, 56.3995), (55.5818, 56.5)),
    ((55.5818, 56.5), (56.52, 56.5)),
    ((56.52, 56.5), (57.098, 55.922)),
    ((57.098, 55.922), (58.0207, 55.922)),
    ((58.0207, 55.922), (58.5513, 56.4526)),
    ((58.5513, 56.4526), (59.52, 56.4526)),
]
original_tracks = list(board.GetTracks())
for track in original_tracks:
    if isinstance(track, pcbnew.PCB_VIA):
        continue
    if track.GetNetname() == "/1V1" and track.GetLayer() == pcbnew.F_Cu:
        start = (pcbnew.ToMM(track.GetStart().x), pcbnew.ToMM(track.GetStart().y))
        end = (pcbnew.ToMM(track.GetEnd().x), pcbnew.ToMM(track.GetEnd().y))
        if any((close(start, a) and close(end, b)) or
               (close(start, b) and close(end, a)) for a, b in remove_1v1):
            board.Remove(track)
for track in original_tracks:
    if isinstance(track, pcbnew.PCB_VIA):
        continue
    if track.GetNetname() != "/GND" or track.GetLayer() != pcbnew.F_Cu:
        continue
    start = (pcbnew.ToMM(track.GetStart().x), pcbnew.ToMM(track.GetStart().y))
    end = (pcbnew.ToMM(track.GetEnd().x), pcbnew.ToMM(track.GetEnd().y))
    if all(52 < x < 61.2 and 55 < y < 58 for x, y in (start, end)):
        board.Remove(track)

for start, end in (
    ((53.52, 56.7), (53.52, 56.0)),
    ((56.52, 56.7), (56.52, 56.0)),
    ((53.52, 56.0), (58.3, 56.0)),
    ((58.3, 56.0), (58.3, 56.5)),
    ((58.3, 56.5), (59.52, 56.5)),
    ((59.52, 56.5), (59.52, 56.7)),
):
    add_track(board, "/1V1", start, end)
for start, end in (
    ((54.48, 56.7), (55.0, 57.4667)),
    ((57.48, 56.7), (57.5649, 57.0809)),
    ((60.48, 56.7), (60.2225, 57.2919)),
):
    add_track(board, "/GND", start, end)

pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard(args.output_board, board)
print("PASS: rebuilt local /1V1 and GND connections for C39-C41")
