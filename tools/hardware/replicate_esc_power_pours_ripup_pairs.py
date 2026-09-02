"""Verify/copy ESC cell power pours and rip up selected mismatched pairs.

This is intentionally a text-preserving KiCad board edit.  It changes only
the selected zone geometry (when a target differs from the M1 source) and
removes tracks/vias belonging to the explicitly selected differential pairs.
Footprint blocks and all other top-level board items are retained verbatim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import uuid
from pathlib import Path


XY_RE = re.compile(r"\(xy\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\)")
FIELD_RE = re.compile(
    r'\((?P<name>[A-Za-z_][A-Za-z0-9_]*)\s+'
    r'(?:"(?P<quoted>[^"]*)"|(?P<bare>[^\s\)]+))'
)


def top_level_items(text: str) -> list[tuple[int, int, str, str]]:
    """Return (start, end, token, block) for direct children of kicad_pcb."""

    out: list[tuple[int, int, str, str]] = []
    depth = 0
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c == '"':
            i += 1
            while i < n:
                if text[i] == "\\":
                    i += 2
                elif text[i] == '"':
                    i += 1
                    break
                else:
                    i += 1
            continue
        if c != "(":
            if c == ")":
                depth -= 1
            i += 1
            continue
        if depth == 1:
            start = i
            d = 0
            j = i
            quoted = False
            escaped = False
            while j < n:
                ch = text[j]
                if quoted:
                    if escaped:
                        escaped = False
                    elif ch == "\\":
                        escaped = True
                    elif ch == '"':
                        quoted = False
                else:
                    if ch == '"':
                        quoted = True
                    elif ch == "(":
                        d += 1
                    elif ch == ")":
                        d -= 1
                        if d == 0:
                            block = text[start : j + 1]
                            token = block[1:].lstrip().split(None, 1)[0]
                            out.append((start, j + 1, token, block))
                            i = j + 1
                            break
                j += 1
            else:
                raise ValueError("unterminated top-level board item")
            continue
        depth += 1
        i += 1
    return out


def field(block: str, name: str) -> str:
    match = re.search(
        rf'\({re.escape(name)}\s+(?:"([^"]*)"|([^\s\)]+))', block
    )
    if not match:
        return ""
    return match.group(1) if match.group(1) is not None else match.group(2)


def nested_block(block: str, token: str) -> str:
    start = block.find(f"({token}")
    if start < 0:
        return ""
    depth = 0
    quoted = False
    escaped = False
    for i in range(start, len(block)):
        c = block[i]
        if quoted:
            if escaped:
                escaped = False
            elif c == "\\":
                escaped = True
            elif c == '"':
                quoted = False
            continue
        if c == '"':
            quoted = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return block[start : i + 1]
    raise ValueError(f"unterminated {token} block")


def xy_points(block: str) -> list[tuple[float, float]]:
    return [(float(x), float(y)) for x, y in XY_RE.findall(block)]


def replace_xy(block: str, points: list[tuple[float, float]]) -> str:
    index = 0

    def repl(match: re.Match[str]) -> str:
        nonlocal index
        if index >= len(points):
            raise ValueError("source/target zone point count mismatch")
        x, y = points[index]
        index += 1
        return f"(xy {x:.6f} {y:.6f})"

    result = XY_RE.sub(repl, block)
    if index != len(points):
        raise ValueError("source/target zone point count mismatch")
    return result


def replace_zone_geometry(target: str, source_points: list[tuple[float, float]], dx: float, dy: float) -> str:
    translated = [(x + dx, y + dy) for x, y in source_points]
    return replace_xy(target, translated)


def zone_signature(block: str) -> tuple[tuple[float, float], ...]:
    outline = nested_block(block, "polygon")
    points = xy_points(outline)
    if not points:
        return ()
    x0 = min(x for x, _ in points)
    y0 = min(y for _, y in points)
    return tuple((round(x - x0, 6), round(y - y0, 6)) for x, y in xy_points(block))


def zone_origin(block: str) -> tuple[float, float]:
    points = xy_points(nested_block(block, "polygon"))
    if not points:
        raise ValueError("zone has no polygon points")
    return min(x for x, _ in points), min(y for _, y in points)


def item_hash(items: list[tuple[int, int, str, str]], token: str) -> str:
    payload = "".join(block for _, _, kind, block in items if kind == token)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def stats_for_nets(items: list[tuple[int, int, str, str]], net_ids: set[int]) -> dict[str, int]:
    result = {"segments": 0, "vias": 0}
    for _, _, kind, block in items:
        if kind not in {"segment", "via"}:
            continue
        try:
            net_id = int(field(block, "net"))
        except ValueError:
            continue
        if net_id in net_ids:
            result["segments" if kind == "segment" else "vias"] += 1
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--backup", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    original = args.board.read_text(encoding="utf-8")
    items_before = top_level_items(original)
    footprint_hash_before = item_hash(items_before, "footprint")

    # The large B.Cu BATN pour in the top-left cell is the M1 source.  The
    # six target zones must keep their own net IDs/names and UUIDs.
    zones = [
        (int(field(block, "net")), start, end, block)
        for start, end, kind, block in items_before
        if kind == "zone" and field(block, "layer") == "B.Cu"
    ]
    source = None
    for net_id, start, end, block in zones:
        if field(block, "net_name") == "/M1_BATN" and len(xy_points(block)) > 1000:
            source = (net_id, start, end, block)
            break
    if source is None:
        raise SystemExit("could not find the top-left M1_BATN main B.Cu pour")
    source_net, _, _, source_block = source
    source_origin = zone_origin(source_block)
    source_signature = zone_signature(source_block)

    # Resolve the board net table from the direct top-level declarations.
    net_names: dict[int, str] = {}
    for _, _, kind, block in items_before:
        if kind != "net":
            continue
        match = re.match(r'\(net\s+(\d+)\s+"([^"]*)"', block)
        if match:
            net_names[int(match.group(1))] = match.group(2)
    pair_names: list[tuple[str, str]] = []
    for motor in range(1, 7):
        pair_names.append(
            (
                f"/Motor {motor} power cell/M{motor}_BUS_SH_P",
                f"/Motor {motor} power cell/M{motor}_BUS_SH_N",
            )
        )
    pair_ids = {
        net_id
        for net_id, name in net_names.items()
        if any(name == wanted for pair in pair_names for wanted in pair)
    }
    if len(pair_ids) != 12:
        raise SystemExit(f"expected 12 BUS_SH nets, found {len(pair_ids)}")

    before_stats = stats_for_nets(items_before, pair_ids)
    shutil.copy2(args.board, args.backup)

    # Replace only geometry in the five target main-pour zones when needed.
    replacements: dict[int, str] = {}
    zone_report: list[dict[str, object]] = []
    for motor in range(1, 7):
        wanted = f"/M{motor}_BATN"
        matches = [
            (net_id, start, end, block)
            for net_id, start, end, block in zones
            if field(block, "net_name") == wanted and len(xy_points(block)) > 1000
        ]
        if len(matches) != 1:
            raise SystemExit(f"expected one large B.Cu {wanted} zone, found {len(matches)}")
        net_id, start, end, target = matches[0]
        target_origin = zone_origin(target)
        expected_sig = source_signature
        actual_sig = zone_signature(target)
        changed = actual_sig != expected_sig
        if changed:
            dx = target_origin[0] - source_origin[0]
            dy = target_origin[1] - source_origin[1]
            replacements[start] = replace_zone_geometry(target, xy_points(source_block), dx, dy)
        zone_report.append(
            {
                "motor": motor,
                "net": wanted,
                "net_id": net_id,
                "origin": [round(target_origin[0], 6), round(target_origin[1], 6)],
                "copied_from_m1": changed,
                "verified_after": True,
            }
        )

    # Drop all segments and vias for the six explicitly named mismatched
    # BUS_SH pairs.  No pads, footprints, zones, or net definitions are moved.
    remove_spans = []
    removed = {"segments": 0, "vias": 0}
    for start, end, kind, block in items_before:
        if kind not in {"segment", "via"}:
            continue
        try:
            net_id = int(field(block, "net"))
        except ValueError:
            continue
        if net_id in pair_ids:
            remove_spans.append((start, end))
            removed["segments" if kind == "segment" else "vias"] += 1

    edits = [(start, end, replacements[start]) for start in replacements]
    # Apply non-overlapping spans from right to left, preserving all untouched
    # bytes.  A removed item's surrounding whitespace is intentionally kept.
    spans: list[tuple[int, int, str]] = edits + [(start, end, "") for start, end in remove_spans]
    result = original
    for start, end, replacement in sorted(spans, reverse=True):
        result = result[:start] + replacement + result[end:]
    args.board.write_text(result, encoding="utf-8", newline="")

    items_after = top_level_items(result)
    footprint_hash_after = item_hash(items_after, "footprint")
    if footprint_hash_before != footprint_hash_after:
        raise SystemExit("footprint block hash changed; refusing to accept edit")
    after_stats = stats_for_nets(items_after, pair_ids)
    if after_stats != {"segments": 0, "vias": 0}:
        raise SystemExit(f"selected pair copper remains: {after_stats}")

    # Re-check target pour signatures after the edit.
    for report in zone_report:
        wanted = str(report["net"])
        matching = [
            block
            for _, _, kind, block in items_after
            if kind == "zone" and field(block, "net_name") == wanted and len(xy_points(block)) > 1000
        ]
        if len(matching) != 1 or zone_signature(matching[0]) != source_signature:
            raise SystemExit(f"power-pour verification failed for {wanted}")

    report = {
        "board": str(args.board),
        "backup": str(args.backup),
        "source_zone": {"net": "/M1_BATN", "net_id": source_net},
        "power_zones": zone_report,
        "power_pour_action": "verified_existing_translated_copies" if not replacements else "copied_geometry_to_targets",
        "differential_pairs_ripped_up": [list(pair) for pair in pair_names],
        "selected_pair_net_ids": sorted(pair_ids),
        "copper_before": before_stats,
        "copper_removed": removed,
        "copper_after": after_stats,
        "footprint_hash_unchanged": footprint_hash_before == footprint_hash_after,
        "footprint_count": sum(1 for _, _, kind, _ in items_after if kind == "footprint"),
        "zone_count": sum(1 for _, _, kind, _ in items_after if kind == "zone"),
        "board_sha256": hashlib.sha256(result.encode("utf-8")).hexdigest(),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
