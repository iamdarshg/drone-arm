#!/usr/bin/env python3
"""Move C39-C41 0.2 mm south without changing pad or net assignments."""
import argparse
import pcbnew

def pad_map(board):
    return tuple(sorted(
        (fp.GetReference(), fp.GetValue(), fp.GetFPID().GetLibItemName(),
         round(fp.GetOrientationDegrees(), 6),
         tuple(sorted((pad.GetNumber(), pad.GetNetname()) for pad in fp.Pads()))
        )
        for fp in board.GetFootprints()
    ))

parser = argparse.ArgumentParser()
parser.add_argument("input_board")
parser.add_argument("output_board")
args = parser.parse_args()
board = pcbnew.LoadBoard(args.input_board)
before = pad_map(board)
for reference in ("C39", "C40", "C41"):
    fp = next(fp for fp in board.GetFootprints() if fp.GetReference() == reference)
    position = fp.GetPosition()
    fp.SetPosition(pcbnew.VECTOR2I(position.x, position.y + pcbnew.FromMM(0.2)))
pcbnew.SaveBoard(args.output_board, board)
if pad_map(pcbnew.LoadBoard(args.output_board)) != before:
    raise SystemExit("footprint pad assignments changed")
print("PASS: C39-C41 shifted +0.2 mm south; all pad/net assignments preserved")
