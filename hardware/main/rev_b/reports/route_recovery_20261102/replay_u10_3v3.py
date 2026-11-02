#!/usr/bin/env python3
"""Join U10 /3V3 pads 24 and 29 below the south pad row."""
import argparse
import pcbnew

def mm(value):
    return pcbnew.FromMM(value)

def xy(point):
    return pcbnew.VECTOR2I(mm(point[0]), mm(point[1]))

parser = argparse.ArgumentParser()
parser.add_argument("input_board")
parser.add_argument("output_board")
args = parser.parse_args()
board = pcbnew.LoadBoard(args.input_board)
net = board.FindNet("/3V3").GetNetCode()
for start, end in (
    ((58.4, 54.95), (58.4, 55.6)),
    ((58.4, 55.6), (56.4, 55.6)),
    ((56.4, 55.6), (56.4, 54.95)),
):
    track = pcbnew.PCB_TRACK(board)
    track.SetStart(xy(start))
    track.SetEnd(xy(end))
    track.SetLayer(pcbnew.F_Cu)
    track.SetWidth(mm(0.2))
    track.SetNetCode(net)
    board.Add(track)
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard(args.output_board, board)
print("PASS: routed U10 /3V3 pads 24 and 29 with 0.20 mm F.Cu copper")
