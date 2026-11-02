#!/usr/bin/env python3
"""Refill zones in a separate pcbnew process before native DRC."""
import pcbnew
import sys
board = pcbnew.LoadBoard(sys.argv[1])
if board is None:
    raise SystemExit(f"could not load {sys.argv[1]}")
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard(sys.argv[2], board)
print(f"refilled and saved {sys.argv[2]}", flush=True)
