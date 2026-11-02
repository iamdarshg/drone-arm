#!/usr/bin/python3
"""Compare replayed pads and copper geometry with the pinned route snapshot."""
import collections
import os
import sys

import pcbnew


def pad_map(board):
    result = {}
    for fp in board.GetFootprints():
        for index, pad in enumerate(fp.Pads()):
            result[(fp.GetReference(), index)] = (
                pad.GetNumber(), pad.GetNetname(), pad.GetSize().x,
                pad.GetSize().y, pad.GetShape(), pad.GetAttribute(),
                pad.GetPosition().x, pad.GetPosition().y,
            )
    return result


def copper_map(board):
    result = collections.defaultdict(collections.Counter)
    for item in board.GetTracks():
        net = item.GetNetname()
        if isinstance(item, pcbnew.PCB_VIA):
            pos = item.GetPosition()
            shape = (
                "via", pos.x, pos.y, item.GetViaType(), item.TopLayer(),
                item.BottomLayer(), item.GetWidth(item.TopLayer()),
                item.GetDrillValue(),
            )
        else:
            start, end = item.GetStart(), item.GetEnd()
            ends = tuple(sorted(((start.x, start.y), (end.x, end.y))))
            shape = ("track", item.GetLayer(), ends, item.GetWidth())
        result[net][shape] += 1
    return result


target_path, replay_path = sys.argv[1:3]
target = pcbnew.LoadBoard(target_path)
replay = pcbnew.LoadBoard(replay_path)
if len(pad_map(target)) != 657 or pad_map(target) != pad_map(replay):
    raise SystemExit("replayed board pad map differs from route snapshot")
if copper_map(target) != copper_map(replay):
    raise SystemExit("replayed copper geometry differs from route snapshot")
print("PASS: all 657 physical pad instances and complete copper geometry match the route snapshot")
sys.stdout.flush()
os._exit(0)
