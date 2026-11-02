#!/usr/bin/env python3
"""Shorten four J1 duplicated GND land lengths within GCT drawing tolerance."""
import pathlib
import sys
import pcbnew

source, output = map(pathlib.Path, sys.argv[1:3])
board = pcbnew.LoadBoard(str(source))
changed = []
for footprint in board.GetFootprints():
    if footprint.GetReference() != "J1":
        continue
    for pad in footprint.Pads():
        if pad.GetNumber() in {"A1", "B12", "A12", "B1"}:
            size = pad.GetSize()
            assert abs(pcbnew.ToMM(size.y) - 1.15) < 0.001
            pad.SetSize(pcbnew.VECTOR2I(size.x, pcbnew.FromMM(1.10)))
            changed.append(pad.GetNumber())
assert sorted(changed) == ["A1", "A12", "B1", "B12"]
pcbnew.SaveBoard(str(output), board)
print(f"shortened J1 pads {','.join(sorted(changed))} to 0.60 x 1.10 mm")
