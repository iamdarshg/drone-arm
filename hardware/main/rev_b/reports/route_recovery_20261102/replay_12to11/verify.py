#!/usr/bin/env python3
"""Verify source pin, native DRC signatures, unchanged pads, and replay geometry."""
import hashlib
import json
import pathlib
import subprocess
import sys
import pcbnew

source, source_pro, candidate, candidate_pro, baseline_path, candidate_path, promoted, promoted_pro, comparator = map(pathlib.Path, sys.argv[1:10])

def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
assert sha(source) == "a568c7faef6fee3a9a45423d4b7e9653e37e862d0f4db0cada9e3a72651a7a5d"
assert sha(source_pro) == "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
assert sha(candidate_pro) == sha(source_pro) == sha(promoted_pro)
baseline, routed = json.loads(baseline_path.read_text()), json.loads(candidate_path.read_text())
assert len(baseline["unconnected_items"]) == 12
assert len(routed["unconnected_items"]) == 11

def signature(v):
    items = sorted(((i.get("description"), i.get("pos")) for i in v.get("items", [])),
                   key=lambda x: json.dumps(x, sort_keys=True))
    return json.dumps((v.get("type"), v.get("severity"), v.get("description"), items), sort_keys=True)

assert len(baseline["violations"]) == len(routed["violations"]) == 4
assert sorted(map(signature, baseline["violations"])) == sorted(map(signature, routed["violations"]))
assert not any("/915 MHz RF/CC_LNA_P" in repr(item) for item in routed["unconnected_items"])

def footprint_map(board):
    return {fp.GetReference(): (fp.GetValue(),
            (round(pcbnew.ToMM(fp.GetPosition().x), 4), round(pcbnew.ToMM(fp.GetPosition().y), 4)),
            round(fp.GetOrientationDegrees(), 4)) for fp in board.GetFootprints()}

def pad_net_size_map(board):
    return {(fp.GetReference(), pad.GetNumber()): (pad.GetNetname(),
            (round(pcbnew.ToMM(pad.GetSize().x), 4), round(pcbnew.ToMM(pad.GetSize().y), 4)))
            for fp in board.GetFootprints() for pad in fp.Pads()}

source_board, candidate_board = pcbnew.LoadBoard(str(source)), pcbnew.LoadBoard(str(candidate))
assert len(footprint_map(source_board)) == len(footprint_map(candidate_board)) == 165
assert footprint_map(source_board) == footprint_map(candidate_board)
assert len(pad_net_size_map(source_board)) == len(pad_net_size_map(candidate_board)) == 603
assert pad_net_size_map(source_board) == pad_net_size_map(candidate_board)
subprocess.run([sys.executable, str(comparator), str(promoted), str(candidate)], check=True)
print("PASS: native DRC improves from 12 to 11 opens with the same four baseline findings")
print("PASS: 165 footprints and 603 pad/net/size mappings are unchanged")
print("PASS: CC_LNA_P closed and replay geometry matches the promoted board")
print("source", sha(source), "candidate", sha(candidate), "candidate DRC", sha(candidate_path))
