#!/usr/bin/env python3
"""Import a FreeRouting SES onto a copy of a KiCad board and refill zones."""
import argparse
import pcbnew

parser = argparse.ArgumentParser()
parser.add_argument("board")
parser.add_argument("ses")
parser.add_argument("output")
args = parser.parse_args()
board = pcbnew.LoadBoard(args.board)
if not pcbnew.ImportSpecctraSES(board, args.ses):
    raise SystemExit("KiCad failed to import the SES")
pcbnew.ZONE_FILLER(board).Fill(board.Zones())
pcbnew.SaveBoard(args.output, board)
print("SES imported and zones refilled")
