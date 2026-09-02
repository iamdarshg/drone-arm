"""Remove only main-board differential pairs that fail the length audit."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

from replicate_esc_power_pours_ripup_pairs import field, top_level_items


PAIR_GROUPS = [
    (
        "USB MCU-side pair",
        "/USB_DP",
        "/USB_DM",
        0.5,
    ),
    (
        "CAN connector pair",
        "/CANH",
        "/CANL",
        2.0,
    ),
    (
        "CAN transceiver pair",
        "/CAN interface/U50_CANH",
        "/CAN interface/U50_CANL",
        2.0,
    ),
]


def net_names(items: list[tuple[int, int, str, str]]) -> dict[int, str]:
    result: dict[int, str] = {}
    for _, _, kind, block in items:
        if kind != "net":
            continue
        match = re.match(r'\(net\s+(\d+)\s+"([^"]*)"', block)
        if match:
            result[int(match.group(1))] = match.group(2)
    return result


def route_stats(
    items: list[tuple[int, int, str, str]],
    names: dict[int, str],
    selected: set[str],
) -> dict[str, dict[str, float | int]]:
    result = {name: {"segments": 0, "vias": 0, "length_mm": 0.0} for name in selected}
    for _, _, kind, block in items:
        if kind not in {"segment", "via"}:
            continue
        raw_net = field(block, "net")
        if not raw_net:
            continue
        name = names.get(int(raw_net), "")
        if name not in selected:
            continue
        if kind == "via":
            result[name]["vias"] += 1
            continue
        start = re.search(
            r"\(start\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", block
        )
        end = re.search(r"\(end\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", block)
        result[name]["segments"] += 1
        if start and end:
            dx = float(end.group(1)) - float(start.group(1))
            dy = float(end.group(2)) - float(start.group(2))
            result[name]["length_mm"] += (dx * dx + dy * dy) ** 0.5
    for data in result.values():
        data["length_mm"] = round(float(data["length_mm"]), 3)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--backup", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    original = args.board.read_text(encoding="utf-8")
    before = top_level_items(original)
    names = net_names(before)
    pair_names = {name for _, positive, negative, _ in PAIR_GROUPS for name in (positive, negative)}
    known = {name for name in pair_names if name in names.values()}
    if known != pair_names:
        raise SystemExit(f"missing expected main-board pair nets: {sorted(pair_names - known)}")

    stats_before = route_stats(before, names, pair_names)
    ranges = [
        (start, end)
        for start, end, kind, block in before
        if kind in {"segment", "via"}
        and names.get(int(field(block, "net")), "") in pair_names
    ]
    if not ranges:
        raise SystemExit("no route items found for the selected main-board pairs")
    shutil.copy2(args.board, args.backup)

    result = original
    for start, end in sorted(ranges, reverse=True):
        result = result[:start] + result[end:]
    args.board.write_text(result, encoding="utf-8", newline="")

    after = top_level_items(result)
    stats_after = route_stats(after, names, pair_names)
    before_footprints = "".join(block for _, _, kind, block in before if kind == "footprint")
    after_footprints = "".join(block for _, _, kind, block in after if kind == "footprint")
    report = {
        "board": str(args.board),
        "removed_pair_groups": [name for name, _, _, _ in PAIR_GROUPS],
        "kept_pair_group": {
            "name": "USB connector-side pair",
            "positive": "/Power and USB/USB_DP_CONN",
            "negative": "/Power and USB/USB_DN_CONN",
            "reason": "live audit passed the 0.5 mm skew limit",
        },
        "pair_limits_mm": {
            name: limit for name, _, _, limit in PAIR_GROUPS
        },
        "route_stats_before": stats_before,
        "route_stats_after": stats_after,
        "removed_segments": sum(int(data["segments"]) for data in stats_before.values()),
        "removed_vias": sum(int(data["vias"]) for data in stats_before.values()),
        "footprints_unchanged": hashlib.sha256(before_footprints.encode()).hexdigest()
        == hashlib.sha256(after_footprints.encode()).hexdigest(),
        "board_sha256": hashlib.sha256(result.encode("utf-8")).hexdigest(),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
