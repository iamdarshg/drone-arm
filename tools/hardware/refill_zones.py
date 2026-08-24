"""Refill all zones on a board using the KiCad Python API.

Usage: kicad-python refill_zones.py <board.kicad_pcb> [<board2.kicad_pcb> ...]
"""
import sys

import pcbnew


def main() -> int:
    for path in sys.argv[1:]:
        print(f"Loading {path}")
        board = pcbnew.LoadBoard(path)
        filler = pcbnew.ZONE_FILLER(board)
        filler.Fill(board.Zones())
        pcbnew.SaveBoard(path, board)
        print(f"Refilled and saved {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
