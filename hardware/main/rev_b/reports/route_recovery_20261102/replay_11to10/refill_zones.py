#!/usr/bin/env python3
"""Refill copper zones after the physical land size adjustment."""
import pathlib
import sys
import pcbnew
source, output = map(pathlib.Path, sys.argv[1:3])
board = pcbnew.LoadBoard(str(source))
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard(str(output), board)
print(f"refilled and saved {output}")
