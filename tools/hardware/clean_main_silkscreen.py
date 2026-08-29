#!/usr/bin/env python3
"""Prepare dense main-board silkscreen for low-cost fabrication."""

from __future__ import annotations

import argparse
from pathlib import Path

import pcbnew


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("board", type=Path)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.board))
    hidden = 0
    for footprint in board.GetFootprints():
        reference = footprint.Reference()
        if reference.IsVisible():
            reference.SetVisible(False)
            hidden += 1

    pcbnew.SaveBoard(str(args.board), board)
    print(f"hidden_silkscreen_references={hidden}")


if __name__ == "__main__":
    main()
