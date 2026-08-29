#!/usr/bin/env python3
"""Remove only KiCad-identified dangling copper and exact duplicate vias."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pcbnew


REMOVABLE_TYPES = {"track_dangling", "via_dangling", "holes_co_located"}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("board", type=Path)
    parser.add_argument("drc_json", type=Path)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.board))
    report = json.loads(args.drc_json.read_text(encoding="utf-8"))
    copper_by_uuid = {
        str(item.m_Uuid.AsString()): item
        for item in board.GetTracks()
    }
    removed: list[str] = []
    for violation in report["violations"]:
        if violation["type"] not in REMOVABLE_TYPES:
            continue
        described_items = violation["items"]
        if violation["type"] == "holes_co_located":
            if len(described_items) != 2:
                raise RuntimeError("unexpected co-located-hole violation shape")
            first = copper_by_uuid.get(described_items[0]["uuid"])
            second = copper_by_uuid.get(described_items[1]["uuid"])
            if first is None or second is None or first.GetNetCode() != second.GetNetCode():
                raise RuntimeError("co-located holes are not duplicate same-net vias")
            described_items = described_items[1:]
        for described in described_items:
            item = copper_by_uuid.get(described["uuid"])
            if item is None:
                raise RuntimeError(f"missing board item {described['uuid']}")
            board.Remove(item)
            removed.append(described["uuid"])

    pcbnew.SaveBoard(str(args.board), board)
    print(f"removed_dangling_items={len(removed)}")


if __name__ == "__main__":
    main()
