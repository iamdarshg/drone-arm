#!/usr/bin/env python3
"""Apply standard-process zone clearance and thermal geometry to the main PCB."""

from __future__ import annotations

import argparse
from pathlib import Path

import pcbnew


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("board", type=Path)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.board))
    for zone in board.Zones():
        zone.SetLocalClearance(pcbnew.FromMM(0.25))
        zone.SetMinThickness(pcbnew.FromMM(0.20))
        zone.SetThermalReliefGap(pcbnew.FromMM(0.20))
        zone.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.20))

    # RF transfer nodes and exposed ground pads need continuous copper rather
    # than thermals; spokes either starve completely or add unwanted RF
    # inductance at these geometries.
    solid_pads = {
        "C19": ("1",),
        "C20": ("2",),
        "IC2": ("11",),
        "IC6": ("8", "9"),
        "IC8": ("8", "9"),
        "IC9": ("10",),
        "IC10": ("9",),
    }
    for reference, pad_numbers in solid_pads.items():
        footprint = board.FindFootprintByReference(reference)
        if footprint is None:
            raise RuntimeError(f"missing footprint {reference}")
        for pad_number in pad_numbers:
            pad = footprint.FindPadByNumber(pad_number)
            if pad is None:
                raise RuntimeError(f"missing pad {reference}.{pad_number}")
            pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)

    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(args.board), board)
    print(f"updated_zones={len(list(board.Zones()))}")


if __name__ == "__main__":
    main()
