#!/usr/bin/env python3
"""Remove unsafe preserved ESC gate-drive/current-sense copper before rerouting."""

from __future__ import annotations

import argparse
import json
import re
from collections import Counter
from pathlib import Path

import pcbnew

_GATE_RE = re.compile(r"^/Motor [1-6] power cell/M[1-6]_G[HL][ABC](?:_DRV)?$")
_SENSE_RE = re.compile(
    r"_(?:SH[ABC]_[PN]|BUS_SH_[PN]|CSA_[ABC](?:_RAW)?|BUS_CURRENT(?:_RAW)?)$"
)
_EXPECTED_GATE_NETS = 72
_EXPECTED_SENSE_NETS = 96


def classify(name: str) -> str | None:
    if _GATE_RE.fullmatch(name):
        return "gate"
    if _SENSE_RE.search(name):
        return "sense"
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.board))
    net_names = {str(name) for name in board.GetNetsByName().keys()}
    gate_nets = sorted(name for name in net_names if classify(name) == "gate")
    sense_nets = sorted(name for name in net_names if classify(name) == "sense")
    if len(gate_nets) != _EXPECTED_GATE_NETS:
        raise SystemExit(
            f"Expected {_EXPECTED_GATE_NETS} gate nets, found {len(gate_nets)}"
        )
    if len(sense_nets) != _EXPECTED_SENSE_NETS:
        raise SystemExit(
            f"Expected {_EXPECTED_SENSE_NETS} sense nets, found {len(sense_nets)}"
        )

    removed_by_kind: Counter[str] = Counter()
    removed_by_group: Counter[str] = Counter()
    removed_by_layer: Counter[str] = Counter()
    removed_by_net: Counter[str] = Counter()
    for item in list(board.GetTracks()):
        name = str(item.GetNetname())
        group = classify(name)
        if group is None:
            continue
        kind = "via" if isinstance(item, pcbnew.PCB_VIA) else "track"
        if kind == "via":
            layer = "through"
        else:
            layer = str(board.GetLayerName(item.GetLayer()))
        removed_by_kind[kind] += 1
        removed_by_group[group] += 1
        removed_by_layer[layer] += 1
        removed_by_net[name] += 1
        board.Remove(item)

    if not removed_by_net:
        raise SystemExit("No sensitive routed copper found; refusing no-op reroute base")

    remaining = [
        str(item.GetNetname())
        for item in board.GetTracks()
        if classify(str(item.GetNetname())) is not None
    ]
    if remaining:
        raise SystemExit(f"Sensitive copper remained after removal: {remaining[:10]}")

    board.BuildConnectivity()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(args.output), board)

    report = {
        "source_board": str(args.board),
        "output_board": str(args.output),
        "gate_net_count": len(gate_nets),
        "sense_net_count": len(sense_nets),
        "target_net_count": len(gate_nets) + len(sense_nets),
        "removed_item_count": sum(removed_by_kind.values()),
        "removed_by_kind": dict(sorted(removed_by_kind.items())),
        "removed_by_group": dict(sorted(removed_by_group.items())),
        "removed_by_layer": dict(sorted(removed_by_layer.items())),
        "target_nets_with_removed_copper": len(removed_by_net),
        "target_nets_without_existing_copper": sorted(
            (set(gate_nets) | set(sense_nets)) - set(removed_by_net)
        ),
        "remaining_sensitive_copper_items": 0,
        "unconnected_after_strip": int(
            board.GetConnectivity().GetUnconnectedCount(False)
        ),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
