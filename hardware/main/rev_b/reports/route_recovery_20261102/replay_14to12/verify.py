#!/usr/bin/env python3
"""Verify source, native DRC, project rules, pad maps, and promoted geometry."""
import hashlib
import json
import pathlib
import subprocess
import sys
import pcbnew

(source, source_pro, candidate, candidate_pro, baseline_drc, candidate_drc,
 promoted, promoted_pro, geometry_comparator) = map(pathlib.Path, sys.argv[1:10])
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
assert sha(source) == "63b00e91cc1ea46e7e539b383c0deb6c7ac4bdfa833dd14cdd59936aef90e0ef"
assert sha(source_pro) == "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
assert sha(candidate_pro) == sha(source_pro) == sha(promoted_pro)
base, routed = json.loads(baseline_drc.read_text()), json.loads(candidate_drc.read_text())
assert len(base["unconnected_items"]) == 14
assert len(routed["unconnected_items"]) == 12
assert len(base["violations"]) == len(routed["violations"]) == 4
def signature(item):
    return json.dumps((item.get("type"), item.get("severity"), item.get("description"),
                       tuple(sorted((entry.get("description"), entry.get("pos"))
                                    for entry in item.get("items", [])))), sort_keys=True)
assert sorted(map(signature, base["violations"])) == sorted(map(signature, routed["violations"]))
for net in ("/915 MHz RF/CC_RBIAS", "/915 MHz RF/CC1121_RF"):
    assert not any(net in repr(item) for item in routed["unconnected_items"]), net

source_board, candidate_board = pcbnew.LoadBoard(str(source)), pcbnew.LoadBoard(str(candidate))
def footprint_map(board):
    result = {}
    for footprint in board.GetFootprints():
        result[footprint.GetReference()] = (
            footprint.GetValue(),
            (round(pcbnew.ToMM(footprint.GetPosition().x), 4),
             round(pcbnew.ToMM(footprint.GetPosition().y), 4)),
            round(footprint.GetOrientationDegrees(), 4),
        )
    return result
def pad_net_size_map(board):
    return {(footprint.GetReference(), pad.GetNumber()):
            (pad.GetNetname(), (round(pcbnew.ToMM(pad.GetSize().x), 4),
                                round(pcbnew.ToMM(pad.GetSize().y), 4)))
            for footprint in board.GetFootprints() for pad in footprint.Pads()}
assert len(footprint_map(source_board)) == len(footprint_map(candidate_board)) == 165
assert len(pad_net_size_map(source_board)) == len(pad_net_size_map(candidate_board)) == 603
assert pad_net_size_map(source_board) == pad_net_size_map(candidate_board)
source_footprints, candidate_footprints = footprint_map(source_board), footprint_map(candidate_board)
assert set(source_footprints) == set(candidate_footprints)
changed = {ref for ref in source_footprints if source_footprints[ref] != candidate_footprints[ref]}
assert changed == {"FL40"}, changed
assert source_footprints["FL40"][1] == candidate_footprints["FL40"][1]
assert candidate_footprints["FL40"][2] in (-90.0, 270.0)
subprocess.run([sys.executable, str(geometry_comparator), str(promoted), str(candidate)], check=True)
print("PASS: native DRC improved 14->12; all four unchanged baseline violations match")
print("PASS: CC_RBIAS and CC1121_RF are closed; 165 footprints and 603 pad/net/size mappings preserved")
print("PASS: only FL40 orientation changed; replay geometry matches the promoted board")
print("source", sha(source), "candidate", sha(candidate), "candidate DRC", sha(candidate_drc))
