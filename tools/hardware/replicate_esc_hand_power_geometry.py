"""Replicate the hand-routed ESC high-current copper from cell M1.

The six motor cells use translated placements.  This pass copies only the
explicit high-current nets from M1 to M2..M6, retaining each target net ID and
leaving footprints, zones, and all signal/current-sense routing untouched.
Vias are compared by translated geometry first; target vias are replaced only
when their positions or dimensions differ from the M1 reference.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import uuid
from collections import Counter
from pathlib import Path

from replicate_esc_power_pours_ripup_pairs import field, top_level_items, xy_points, zone_origin


MOTORS = range(1, 7)
POWER_SUFFIXES = ("BATP", "BATN", "PHASE_A", "PHASE_B", "PHASE_C")


def net_role(name: str, motor: int) -> str | None:
    for suffix in POWER_SUFFIXES:
        if name == f"/M{motor}_{suffix}":
            return f"/M#_{suffix}"
    if name == f"/Motor {motor} power cell/M{motor}_BATP_IN":
        return "/Motor # power cell/M#_BATP_IN"
    return None


def net_name_by_id(items: list[tuple[int, int, str, str]]) -> dict[int, str]:
    result: dict[int, str] = {}
    for _, _, kind, block in items:
        if kind != "net":
            continue
        match = re.match(r'\(net\s+(\d+)\s+"([^"]*)"', block)
        if match:
            result[int(match.group(1))] = match.group(2)
    return result


def item_hash(items: list[tuple[int, int, str, str]], token: str) -> str:
    payload = "".join(block for _, _, kind, block in items if kind == token)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def number(value: str) -> float:
    return round(float(value), 6)


def translate_xy(block: str, dx: float, dy: float, token: str) -> str:
    pattern = re.compile(
        rf"\({token}\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)"
    )

    def replace(match: re.Match[str]) -> str:
        x = float(match.group(1)) + dx
        y = float(match.group(2)) + dy
        return f"({token} {x:.6f} {y:.6f}"

    return pattern.sub(replace, block)


def replace_net(block: str, net_id: int) -> str:
    result, count = re.subn(
        r"\(net\s+[-+0-9]+\)", f"(net {net_id})", block, count=1
    )
    if count != 1:
        raise ValueError("route item has no single net field")
    return result


def fresh_uuid(block: str) -> str:
    result, count = re.subn(
        r'\(uuid\s+"[^"]+"\)', f'(uuid "{uuid.uuid4()}")', block, count=1
    )
    if count != 1:
        raise ValueError("route item has no UUID")
    return result


def route_key(
    block: str,
    kind: str,
    motor: int,
    origin: tuple[float, float],
    net_names: dict[int, str],
) -> tuple[object, ...]:
    net_id = int(field(block, "net"))
    role = net_role(net_names[net_id], motor)
    if role is None:
        raise ValueError(f"unexpected high-current net {net_names.get(net_id, net_id)}")
    ox, oy = origin
    if kind == "segment":
        start = re.search(
            r"\(start\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", block
        )
        end = re.search(r"\(end\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", block)
        if not start or not end:
            raise ValueError("segment has no start/end")
        a = (number(str(float(start.group(1)) - ox)), number(str(float(start.group(2)) - oy)))
        b = (number(str(float(end.group(1)) - ox)), number(str(float(end.group(2)) - oy)))
        return ("segment", role, field(block, "layer"), number(field(block, "width")), tuple(sorted((a, b))))
    at = re.search(r"\(at\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", block)
    layers = re.search(r'\(layers\s+"([^"]+)"\s+"([^"]+)"', block)
    if not at or not layers:
        raise ValueError("via has no at/layers")
    return (
        "via",
        role,
        number(str(float(at.group(1)) - ox)),
        number(str(float(at.group(2)) - oy)),
        field(block, "size"),
        field(block, "drill"),
        layers.groups(),
    )


def high_current_ids(
    net_names: dict[int, str], motor: int
) -> set[int]:
    return {
        net_id
        for net_id, name in net_names.items()
        if net_role(name, motor) is not None
    }


def target_net_name(source_name: str, motor: int) -> str:
    if source_name.startswith("/Motor 1 power cell/M1_"):
        return source_name.replace(
            "/Motor 1 power cell/M1_",
            f"/Motor {motor} power cell/M{motor}_",
            1,
        )
    return source_name.replace("/M1_", f"/M{motor}_", 1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--backup", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    original = args.board.read_text(encoding="utf-8")
    items = top_level_items(original)
    net_names = net_name_by_id(items)
    footprint_hash_before = item_hash(items, "footprint")
    origins: dict[int, tuple[float, float]] = {}
    for motor in MOTORS:
        matches = [
            block
            for _, _, kind, block in items
            if kind == "zone"
            and field(block, "net_name") == f"/M{motor}_BATN"
            and len(xy_points(block)) > 1000
        ]
        if len(matches) != 1:
            raise SystemExit(f"expected one large BATN zone for M{motor}, found {len(matches)}")
        origins[motor] = zone_origin(matches[0])

    selected_ids = {motor: high_current_ids(net_names, motor) for motor in MOTORS}
    if any(len(ids) != 6 for ids in selected_ids.values()):
        raise SystemExit("expected five global high-current nets plus BATP_IN per cell")

    source_segments = [
        block
        for _, _, kind, block in items
        if kind == "segment" and int(field(block, "net")) in selected_ids[1]
    ]
    source_vias = [
        block
        for _, _, kind, block in items
        if kind == "via" and int(field(block, "net")) in selected_ids[1]
    ]
    if not source_segments or not source_vias:
        raise SystemExit("M1 has no high-current copper to use as the reference")

    shutil.copy2(args.board, args.backup)

    # Always replace target tracks with the source geometry.  This removes the
    # stale/shorter target paths first, preventing accidental parallel copper.
    remove_ranges: list[tuple[int, int]] = []
    for start, end, kind, block in items:
        if kind == "segment":
            if any(int(field(block, "net")) in selected_ids[motor] for motor in range(2, 7)):
                remove_ranges.append((start, end))

    # Vias are handled separately: if translated geometry is already exact,
    # leave the user-visible UUIDs in place.  Otherwise replace the target set.
    via_replacements: dict[int, list[str]] = {}
    via_report: dict[str, object] = {}
    for motor in range(2, 7):
        source_keys = Counter(
            route_key(block, "via", 1, origins[1], net_names) for block in source_vias
        )
        target_blocks = [
            block
            for _, _, kind, block in items
            if kind == "via" and int(field(block, "net")) in selected_ids[motor]
        ]
        target_keys = Counter(
            route_key(block, "via", motor, origins[motor], net_names)
            for block in target_blocks
        )
        exact = source_keys == target_keys
        if exact:
            via_report[f"M{motor}"] = {
                "source_vias": len(source_vias),
                "target_vias_before": len(target_blocks),
                "translated_geometry_exact": True,
                "replaced": False,
            }
            continue
        via_report[f"M{motor}"] = {
            "source_vias": len(source_vias),
            "target_vias_before": len(target_blocks),
            "translated_geometry_exact": False,
            "missing_vs_source": sum((source_keys - target_keys).values()),
            "extra_vs_source": sum((target_keys - source_keys).values()),
            "replaced": True,
        }
        for start, end, kind, block in items:
            if kind == "via" and int(field(block, "net")) in selected_ids[motor]:
                remove_ranges.append((start, end))
        dx = origins[motor][0] - origins[1][0]
        dy = origins[motor][1] - origins[1][1]
        translated: list[str] = []
        for block in source_vias:
            source_net = int(field(block, "net"))
            source_name = net_names[source_net]
            target_name = target_net_name(source_name, motor)
            target_net = next(
                net_id for net_id, name in net_names.items() if name == target_name
            )
            translated.append(fresh_uuid(replace_net(translate_xy(block, dx, dy, "at"), target_net)))
        via_replacements[motor] = translated

    # Delete selected ranges from back to front, retaining all other board text.
    result = original
    for start, end in sorted(set(remove_ranges), reverse=True):
        result = result[:start] + result[end:]

    # Offsets changed after removals, so append new routes before the board's
    # final close.  M1 remains untouched; M2..M6 get fresh UUIDs.
    appended: list[str] = []
    for motor in range(2, 7):
        dx = origins[motor][0] - origins[1][0]
        dy = origins[motor][1] - origins[1][1]
        for block in source_segments:
            source_net = int(field(block, "net"))
            source_name = net_names[source_net]
            target_name = target_net_name(source_name, motor)
            target_net = next(
                net_id for net_id, name in net_names.items() if name == target_name
            )
            translated = translate_xy(block, dx, dy, "start")
            translated = translate_xy(translated, dx, dy, "end")
            translated = fresh_uuid(replace_net(translated, target_net))
            appended.append(translated)
        if motor in via_replacements:
            appended.extend(via_replacements[motor])

    closing = result.rfind(")")
    if closing < 0:
        raise SystemExit("board has no final close")
    result = result[:closing] + "\n" + "\n".join(appended) + "\n" + result[closing:]
    args.board.write_text(result, encoding="utf-8", newline="")

    after_items = top_level_items(result)
    footprint_hash_after = item_hash(after_items, "footprint")
    if footprint_hash_before != footprint_hash_after:
        raise SystemExit("footprint blocks changed during high-current replication")

    final_route_counts: dict[str, dict[str, int]] = {}
    geometry_verified: dict[str, bool] = {}
    for motor in MOTORS:
        ids = selected_ids[motor]
        segs = [
            block for _, _, kind, block in after_items
            if kind == "segment" and int(field(block, "net")) in ids
        ]
        vias = [
            block for _, _, kind, block in after_items
            if kind == "via" and int(field(block, "net")) in ids
        ]
        final_route_counts[f"M{motor}"] = {
            "segments": len(segs),
            "vias": len(vias),
        }
        source_seg_keys = Counter(
            route_key(block, "segment", 1, origins[1], net_names) for block in source_segments
        )
        source_via_keys = Counter(
            route_key(block, "via", 1, origins[1], net_names) for block in source_vias
        )
        final_seg_keys = Counter(
            route_key(block, "segment", motor, origins[motor], net_names) for block in segs
        )
        final_via_keys = Counter(
            route_key(block, "via", motor, origins[motor], net_names) for block in vias
        )
        geometry_verified[f"M{motor}"] = (
            final_seg_keys == source_seg_keys and final_via_keys == source_via_keys
        )

    report = {
        "board": str(args.board),
        "source_cell": "M1",
        "replicated_cells": [f"M{motor}" for motor in range(2, 7)],
        "nets": [
            "/M#_BATP",
            "/M#_BATN",
            "/M#_PHASE_A",
            "/M#_PHASE_B",
            "/M#_PHASE_C",
            "/Motor # power cell/M#_BATP_IN",
        ],
        "source_routes": {
            "segments": len(source_segments),
            "vias": len(source_vias),
        },
        "target_route_counts_after": final_route_counts,
        "geometry_verified_against_translated_M1": geometry_verified,
        "via_comparison": via_report,
        "target_segments_replaced": len(source_segments) * 5,
        "footprint_hash_unchanged": footprint_hash_before == footprint_hash_after,
        "board_sha256": hashlib.sha256(result.encode("utf-8")).hexdigest(),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
