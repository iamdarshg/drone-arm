#!/usr/bin/env python3
"""Replicate the complete M1 cell layout and local copper to M2 through M6."""

from __future__ import annotations

import argparse
import collections
import hashlib
import json
import re
from pathlib import Path

import pcbnew


OFFSETS_MM = {
    2: (85.0, 0.0),
    3: (0.0, 82.5),
    4: (85.0, 82.5),
    5: (0.0, 165.0),
    6: (85.0, 165.0),
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def ref_cell(ref: str) -> int | None:
    if ref == "TH1":
        return 1
    if re.fullmatch(r"TH[2-6]", ref):
        return int(ref[2:])
    match = re.fullmatch(r"[A-Z]+1([1-6])\d\d", ref)
    return int(match.group(1)) if match else None


def map_ref(ref: str, cell: int) -> str:
    if ref == "TH1":
        return f"TH{cell}"
    match = re.fullmatch(r"([A-Z]+)11(\d\d)", ref)
    if not match:
        raise RuntimeError(f"cannot map M1 reference {ref}")
    return f"{match.group(1)}1{cell}{match.group(2)}"


def net_cell(name: str) -> int | None:
    match = re.search(r"(?:/Motor |/M)([1-6])(?: power cell/|_)", name)
    return int(match.group(1)) if match else None


def map_net(name: str, cell: int) -> str:
    if net_cell(name) != 1:
        raise RuntimeError(f"cannot map non-M1 net {name}")
    return name.replace("Motor 1 power cell", f"Motor {cell} power cell").replace(
        "M1_", f"M{cell}_"
    )


def footprint_transform(fp: pcbnew.FOOTPRINT) -> tuple[int, int, float, int]:
    position = fp.GetPosition()
    return (
        position.x,
        position.y,
        fp.GetOrientation().AsTenthsOfADegree(),
        fp.GetLayer(),
    )


def local_counts(board: pcbnew.BOARD, cell: int) -> dict[str, object]:
    return {
        "footprints": sum(ref_cell(fp.GetReference()) == cell for fp in board.GetFootprints()),
        "zones": collections.Counter(
            zone.GetNetname() for zone in board.Zones() if net_cell(zone.GetNetname()) == cell
        ),
        "vias": collections.Counter(
            item.GetNetname()
            for item in board.GetTracks()
            if isinstance(item, pcbnew.PCB_VIA) and net_cell(item.GetNetname()) == cell
        ),
        "track_segments": collections.Counter(
            item.GetNetname()
            for item in board.GetTracks()
            if not isinstance(item, pcbnew.PCB_VIA) and net_cell(item.GetNetname()) == cell
        ),
    }


def normalize_counts(counts: dict[str, object], cell: int) -> dict[str, object]:
    def normalize_name(name: str) -> str:
        return name.replace(f"Motor {cell} power cell", "Motor 1 power cell").replace(
            f"M{cell}_", "M1_"
        )

    return {
        "footprints": counts["footprints"],
        "zones": collections.Counter(
            {normalize_name(name): count for name, count in counts["zones"].items()}
        ),
        "vias": collections.Counter(
            {normalize_name(name): count for name, count in counts["vias"].items()}
        ),
        "track_segments": collections.Counter(
            {
                normalize_name(name): count
                for name, count in counts["track_segments"].items()
            }
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.source))
    before_counts = {str(cell): local_counts(board, cell) for cell in range(1, 7)}
    footprints = {fp.GetReference(): fp for fp in board.GetFootprints()}
    source_footprints = [fp for fp in footprints.values() if ref_cell(fp.GetReference()) == 1]
    source_zones = [zone for zone in board.Zones() if net_cell(zone.GetNetname()) == 1]
    source_copper = [item for item in board.GetTracks() if net_cell(item.GetNetname()) == 1]

    # Remove every target cell's old local copper. Shared controller and global
    # nets remain untouched.
    removed_zones = 0
    for zone in list(board.Zones()):
        if net_cell(zone.GetNetname()) in OFFSETS_MM:
            board.Delete(zone)
            removed_zones += 1
    removed_copper = 0
    for item in list(board.GetTracks()):
        if net_cell(item.GetNetname()) in OFFSETS_MM:
            board.Delete(item)
            removed_copper += 1

    for cell, (dx, dy) in OFFSETS_MM.items():
        delta = pcbnew.VECTOR2I_MM(dx, dy)
        for source_fp in source_footprints:
            target_ref = map_ref(source_fp.GetReference(), cell)
            target = footprints.get(target_ref)
            if target is None:
                raise RuntimeError(f"missing target footprint {target_ref}")
            target.SetPosition(source_fp.GetPosition() + delta)
            target.SetOrientation(source_fp.GetOrientation())
            target.SetLayer(pcbnew.F_Cu)

        for source_zone in source_zones:
            target_name = map_net(source_zone.GetNetname(), cell)
            target_net = board.FindNet(target_name)
            if target_net is None:
                raise RuntimeError(f"missing target net {target_name}")
            duplicate = source_zone.Duplicate()
            duplicate.Move(delta)
            duplicate.SetNet(target_net)
            board.Add(duplicate)

        for source_item in source_copper:
            target_name = map_net(source_item.GetNetname(), cell)
            target_net = board.FindNet(target_name)
            if target_net is None:
                raise RuntimeError(f"missing target net {target_name}")
            duplicate = source_item.Duplicate()
            duplicate.Move(delta)
            duplicate.SetNet(target_net)
            board.Add(duplicate)

    # Clearing stale fills before saving prevents the target zones from carrying
    # translated copies of M1's fill cache. The normal KiCad refill follows.
    for zone in board.Zones():
        zone.UnFill()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(args.output), board)

    check = pcbnew.LoadBoard(str(args.output))
    check_footprints = {fp.GetReference(): fp for fp in check.GetFootprints()}
    transform_errors = []
    for source_fp in source_footprints:
        source_transform = footprint_transform(source_fp)
        for cell, (dx, dy) in OFFSETS_MM.items():
            target = check_footprints[map_ref(source_fp.GetReference(), cell)]
            target_transform = footprint_transform(target)
            expected = (
                source_transform[0] + pcbnew.FromMM(dx),
                source_transform[1] + pcbnew.FromMM(dy),
                source_transform[2],
                pcbnew.F_Cu,
            )
            if target_transform != expected:
                transform_errors.append(
                    {
                        "source": source_fp.GetReference(),
                        "target": target.GetReference(),
                        "expected": expected,
                        "actual": target_transform,
                    }
                )
    after_counts = {str(cell): local_counts(check, cell) for cell in range(1, 7)}
    expected_counts = normalize_counts(after_counts["1"], 1)
    count_errors = [
        cell
        for cell in range(2, 7)
        if normalize_counts(after_counts[str(cell)], cell) != expected_counts
    ]
    back = [
        fp.GetReference() for fp in check.GetFootprints() if fp.GetLayer() != pcbnew.F_Cu
    ]
    report = {
        "source": str(args.source),
        "source_sha256": sha256(args.source),
        "output": str(args.output),
        "output_sha256": sha256(args.output),
        "offsets_mm": OFFSETS_MM,
        "source_footprints": len(source_footprints),
        "source_zones": len(source_zones),
        "source_copper_items": len(source_copper),
        "removed_target_zones": removed_zones,
        "removed_target_copper_items": removed_copper,
        "before_counts": before_counts,
        "after_counts": after_counts,
        "transform_errors": transform_errors,
        "count_errors": count_errors,
        "back_side_footprints": back,
    }
    report["gate_pass"] = not transform_errors and not count_errors and not back
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "source_footprints": len(source_footprints),
                "source_zones": len(source_zones),
                "source_copper_items": len(source_copper),
                "removed_target_zones": removed_zones,
                "removed_target_copper_items": removed_copper,
                "after_counts": after_counts,
                "transform_error_count": len(transform_errors),
                "count_errors": count_errors,
                "back_side_footprints": back,
                "gate_pass": report["gate_pass"],
            },
            indent=2,
        )
    )
    return 0 if report["gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
