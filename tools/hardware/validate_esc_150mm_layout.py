#!/usr/bin/env python3
"""Validate the mechanical contract for the compact Rev-B ESC placement."""

from __future__ import annotations

import argparse
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pcbnew


MM = 1_000_000.0
TARGET_MM = 150.0
MOTOR_GRIDS = {
    1: (2.0, 24.0, 50.0, 78.0),
    2: (51.0, 24.0, 99.0, 78.0),
    3: (100.0, 24.0, 148.0, 78.0),
    4: (2.0, 79.0, 50.0, 133.0),
    5: (51.0, 79.0, 99.0, 133.0),
    6: (100.0, 79.0, 148.0, 133.0),
}


def electrical_refs(netlist_path: Path) -> set[str]:
    root = ET.parse(netlist_path).getroot()
    return {item.attrib["ref"] for item in root.findall("./components/comp")}


def motor_for_ref(ref: str) -> int | None:
    if match := re.fullmatch(r"TH([1-6])", ref):
        return int(match.group(1))
    match = re.search(r"(\d+)$", ref)
    if not match:
        return None
    number = int(match.group(1))
    for motor in range(1, 7):
        if 1000 + motor * 100 <= number < 1100 + motor * 100:
            return motor
    return None


def bbox_mm(item: object) -> tuple[float, float, float, float]:
    box = item.GetBoundingBox()
    return (
        box.GetX() / MM,
        box.GetY() / MM,
        box.GetRight() / MM,
        box.GetBottom() / MM,
    )


def courtyard_bbox_mm(fp: pcbnew.FOOTPRINT) -> tuple[float, float, float, float]:
    layer = pcbnew.F_CrtYd if fp.GetLayer() == pcbnew.F_Cu else pcbnew.B_CrtYd
    box = fp.GetCourtyard(layer).BBox()
    if not box.GetWidth() or not box.GetHeight():
        return bbox_mm(fp)
    return (
        box.GetX() / MM,
        box.GetY() / MM,
        box.GetRight() / MM,
        box.GetBottom() / MM,
    )


def inside(box: tuple[float, float, float, float], bounds: tuple[float, float, float, float], tolerance: float = 0.01) -> bool:
    x0, y0, x1, y1 = box
    bx0, by0, bx1, by1 = bounds
    return x0 >= bx0 - tolerance and y0 >= by0 - tolerance and x1 <= bx1 + tolerance and y1 <= by1 + tolerance


def edge_extents(board: pcbnew.BOARD) -> tuple[float, float, float, float]:
    points: list[tuple[float, float]] = []
    for item in board.GetDrawings():
        if item.GetLayer() != pcbnew.Edge_Cuts:
            continue
        for getter in ("GetStart", "GetEnd"):
            if hasattr(item, getter):
                point = getattr(item, getter)()
                points.append((point.x / MM, point.y / MM))
    if not points:
        raise ValueError("board has no Edge.Cuts segments")
    xs, ys = zip(*points)
    return min(xs), min(ys), max(xs), max(ys)


def audit(board_path: Path, netlist_path: Path) -> dict[str, object]:
    board = pcbnew.LoadBoard(str(board_path))
    expected = electrical_refs(netlist_path)
    actual = {fp.GetReference() for fp in board.GetFootprints() if fp.GetReference() in expected}
    missing = sorted(expected - actual)
    duplicate_refs = sorted(ref for ref in expected if sum(fp.GetReference() == ref for fp in board.GetFootprints()) != 1)
    extra_electrical = sorted(actual - expected)

    outline = edge_extents(board)
    outline_ok = all(abs(value - target) <= 0.001 for value, target in zip(outline, (0.0, 0.0, TARGET_MM, TARGET_MM)))
    off_board: list[str] = []
    invalid_layer: list[str] = []
    wrong_grid: list[dict[str, object]] = []
    grid_boundary_bleed: list[dict[str, object]] = []
    grid_populations = {str(motor): 0 for motor in MOTOR_GRIDS}
    board_bounds = (0.0, 0.0, TARGET_MM, TARGET_MM)
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if ref not in expected:
            continue
        physical_box = bbox_mm(fp)
        box = courtyard_bbox_mm(fp)
        if not inside(physical_box, board_bounds):
            off_board.append(ref)
        if fp.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu):
            invalid_layer.append(ref)
        motor = motor_for_ref(ref)
        if motor is not None:
            grid_populations[str(motor)] += 1
            position = fp.GetPosition()
            anchor = (position.x / MM, position.y / MM)
            x0, y0, x1, y1 = MOTOR_GRIDS[motor]
            if not (x0 <= anchor[0] <= x1 and y0 <= anchor[1] <= y1):
                wrong_grid.append({"ref": ref, "motor": motor, "bbox_mm": [round(v, 3) for v in box]})
            elif not inside(box, MOTOR_GRIDS[motor]):
                grid_boundary_bleed.append({"ref": ref, "motor": motor, "bbox_mm": [round(v, 3) for v in box]})

    checks = {
        "outline_exact_150mm": outline_ok,
        "six_copper_layers": board.GetCopperLayerCount() == 6,
        "electrical_footprint_parity": not missing and not duplicate_refs and not extra_electrical,
        "front_or_back_copper_only": not invalid_layer,
        "all_electrical_footprints_inside_outline": not off_board,
        "motor_grid_membership": not wrong_grid and all(grid_populations.values()),
    }
    return {
        "board": str(board_path),
        "passed": all(checks.values()),
        "checks": checks,
        "outline_extents_mm": [round(v, 3) for v in outline],
        "copper_layers": board.GetCopperLayerCount(),
        "expected_electrical_footprints": len(expected),
        "actual_electrical_footprints": len(actual),
        "grid_populations": grid_populations,
        "failures": {
            "missing": missing,
            "duplicate_refs": duplicate_refs,
            "extra_electrical": extra_electrical,
            "invalid_layer": sorted(invalid_layer),
            "off_board": sorted(off_board),
            "wrong_grid": wrong_grid,
            "grid_boundary_bleed": grid_boundary_bleed,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--netlist", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(args.board, args.netlist)
    rendered = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    raise SystemExit(0 if result["passed"] else 2)


if __name__ == "__main__":
    main()
