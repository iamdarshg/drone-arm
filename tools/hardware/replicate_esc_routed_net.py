"""Replicate one verified M1 route into the five translated ESC cells."""

from __future__ import annotations

import argparse
import collections
import json
from pathlib import Path

import pcbnew


OFFSETS = {
    2: (85.0, 0.0),
    3: (0.0, 82.5),
    4: (85.0, 82.5),
    5: (0.0, 165.0),
    6: (85.0, 165.0),
}


def shifted(point: pcbnew.VECTOR2I, dx: float, dy: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(point.x + pcbnew.FromMM(dx), point.y + pcbnew.FromMM(dy))


def duplicate_vias(board: pcbnew.BOARD) -> int:
    positions = collections.Counter(
        (item.GetPosition().x, item.GetPosition().y)
        for item in board.GetTracks()
        if isinstance(item, pcbnew.PCB_VIA)
    )
    return sum(count - 1 for count in positions.values() if count > 1)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--m1-net", required=True)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.input))
    board.BuildConnectivity()
    before = board.GetConnectivity().GetUnconnectedCount(False)
    source = [item for item in board.GetTracks() if item.GetNetname() == args.m1_net]
    if not source:
        raise RuntimeError(f"source net has no routed copper: {args.m1_net}")

    expected_prefix = "/Motor 1 power cell/M1_"
    if not args.m1_net.startswith(expected_prefix):
        raise RuntimeError(f"source net must begin with {expected_prefix!r}")
    suffix = args.m1_net[len(expected_prefix):]
    added = {}
    for motor, (dx, dy) in OFFSETS.items():
        target_name = f"/Motor {motor} power cell/M{motor}_{suffix}"
        target_net = board.FindNet(target_name)
        if target_net is None:
            raise RuntimeError(f"missing target net: {target_name}")
        existing = [item for item in board.GetTracks() if item.GetNetname() == target_name]
        if existing:
            raise RuntimeError(f"target net already has {len(existing)} copper items: {target_name}")
        count = 0
        for item in source:
            if isinstance(item, pcbnew.PCB_VIA):
                clone = pcbnew.PCB_VIA(board)
                clone.SetPosition(shifted(item.GetPosition(), dx, dy))
                clone.SetWidth(item.GetWidth(pcbnew.F_Cu))
                clone.SetDrill(item.GetDrillValue())
                clone.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            else:
                clone = pcbnew.PCB_TRACK(board)
                clone.SetStart(shifted(item.GetStart(), dx, dy))
                clone.SetEnd(shifted(item.GetEnd(), dx, dy))
                clone.SetWidth(item.GetWidth())
                clone.SetLayer(item.GetLayer())
            clone.SetNet(target_net)
            board.Add(clone)
            count += 1
        added[target_name] = count

    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.BuildConnectivity()
    after = board.GetConnectivity().GetUnconnectedCount(False)
    duplicates = duplicate_vias(board)
    if after != before - len(OFFSETS):
        raise RuntimeError(f"expected {len(OFFSETS)} fewer opens, got {before} -> {after}")
    if duplicates:
        raise RuntimeError(f"replication created {duplicates} duplicate vias")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(args.output), board)
    print(json.dumps({"source_net": args.m1_net, "added": added, "before": before, "after": after, "duplicate_vias": duplicates}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
