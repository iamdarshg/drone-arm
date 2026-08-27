#!/usr/bin/env python3
"""Build and validate a flat Quilter upload set for the Rev-B ESC."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pcbnew


ROOT = Path(__file__).resolve().parents[2]
ESC_DIR = ROOT / "hardware" / "esc" / "rev_b"
DEFAULT_OUTPUT = ESC_DIR / "quilter_upload"
MM = 1_000_000.0
BOARD_BOUNDS = (0.0, 0.0, 150.0, 150.0)


def motor_number(ref: str) -> int | None:
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


def schematic_groups(netlist: Path) -> tuple[dict[str, str], set[str]]:
    root = ET.parse(netlist).getroot()
    groups: dict[str, str] = {}
    refs: set[str] = set()
    for component in root.findall("./components/comp"):
        ref = component.attrib["ref"]
        refs.add(ref)
        motor = motor_number(ref)
        if motor is not None:
            groups[ref] = f"motor_{motor}"
            continue
        sheet = component.find("./sheetpath").attrib.get("names", "").lower()
        groups[ref] = "power" if "power" in sheet else "controller"
    return groups, refs


def footprint_size(fp: pcbnew.FOOTPRINT) -> tuple[float, float]:
    try:
        box = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
        width, height = box.GetWidth() / MM, box.GetHeight() / MM
    except Exception:
        box = fp.GetBoundingBox()
        width, height = box.GetWidth() / MM, box.GetHeight() / MM
    pad_box = fp.GetFpPadsLocalBbox()
    return (
        max(width, pad_box.GetWidth() / MM, 1.0) + 1.0,
        max(height, pad_box.GetHeight() / MM, 1.0) + 1.0,
    )


def move_to_front(fp: pcbnew.FOOTPRINT) -> None:
    if fp.GetLayer() == pcbnew.B_Cu:
        fp.Flip(fp.GetPosition(), False)


def place_group(
    footprints: list[pcbnew.FOOTPRINT],
    x0: float,
    y0: float,
    width_limit: float = 180.0,
) -> tuple[float, float, float, float]:
    x, y, row_height = x0, y0, 0.0
    max_x = x0
    for fp in sorted(footprints, key=lambda item: (-footprint_size(item)[0] * footprint_size(item)[1], item.GetReference())):
        width, height = footprint_size(fp)
        if x + width > x0 + width_limit:
            x = x0
            y += row_height + 2.0
            row_height = 0.0
        move_to_front(fp)
        fp.SetOrientationDegrees(0.0)
        fp.SetPosition(pcbnew.VECTOR2I(int((x + width / 2) * MM), int((y + height / 2) * MM)))
        x += width + 2.0
        row_height = max(row_height, height)
        max_x = max(max_x, x)
    return x0, y0, max_x, y + row_height


def remove_copper(board: pcbnew.BOARD) -> None:
    for item in list(board.GetTracks()):
        board.Remove(item)
    for zone in list(board.Zones()):
        board.Remove(zone)


def build_board(source: Path, destination: Path, netlist: Path) -> dict[str, object]:
    board = pcbnew.LoadBoard(str(source))
    groups, expected_refs = schematic_groups(netlist)
    remove_copper(board)
    grouped: dict[str, list[pcbnew.FOOTPRINT]] = {
        "power": [],
        "controller": [],
        **{f"motor_{motor}": [] for motor in range(1, 7)},
    }
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        if ref not in groups:
            raise RuntimeError(f"Footprint {ref} is absent from the schematic netlist")
        grouped[groups[ref]].append(fp)

    layouts: dict[str, list[float]] = {}
    columns = ((170.0, 0.0), (370.0, 0.0), (570.0, 0.0))
    ordered_groups = ["power", "controller", "motor_1", "motor_2", "motor_3", "motor_4", "motor_5", "motor_6"]
    column_heights = [0.0, 0.0, 0.0]
    for index, name in enumerate(ordered_groups):
        column = index % 3
        x0, _ = columns[column]
        y0 = column_heights[column]
        bounds = place_group(grouped[name], x0, y0)
        layouts[name] = [round(value, 3) for value in bounds]
        column_heights[column] = bounds[3] + 15.0

    board.BuildConnectivity()
    destination.parent.mkdir(parents=True, exist_ok=True)
    pcbnew.SaveBoard(str(destination), board)
    actual_refs = {fp.GetReference() for fp in board.GetFootprints()}
    return {
        "expected_refs": expected_refs,
        "actual_refs": actual_refs,
        "group_populations": {name: len(items) for name, items in grouped.items()},
        "group_bounds_mm": layouts,
    }


def bbox_mm(fp: pcbnew.FOOTPRINT) -> tuple[float, float, float, float]:
    box = fp.GetBoundingBox()
    return box.GetX() / MM, box.GetY() / MM, box.GetRight() / MM, box.GetBottom() / MM


def intersects_board(box: tuple[float, float, float, float]) -> bool:
    x0, y0, x1, y1 = box
    bx0, by0, bx1, by1 = BOARD_BOUNDS
    return x0 < bx1 and x1 > bx0 and y0 < by1 and y1 > by0


def edge_extents(board: pcbnew.BOARD) -> tuple[float, float, float, float]:
    points: list[tuple[float, float]] = []
    for item in board.GetDrawings():
        if item.GetLayer() != pcbnew.Edge_Cuts:
            continue
        if hasattr(item, "GetStart") and hasattr(item, "GetEnd"):
            for point in (item.GetStart(), item.GetEnd()):
                points.append((point.x / MM, point.y / MM))
    xs, ys = zip(*points)
    return min(xs), min(ys), max(xs), max(ys)


def validate(board_path: Path, expected_refs: set[str]) -> dict[str, object]:
    board = pcbnew.LoadBoard(str(board_path))
    refs = [fp.GetReference() for fp in board.GetFootprints()]
    inside = sorted(fp.GetReference() for fp in board.GetFootprints() if intersects_board(bbox_mm(fp)))
    outline = edge_extents(board)
    checks = {
        "outline_exact_150mm": all(abs(a - b) <= 0.001 for a, b in zip(outline, BOARD_BOUNDS)),
        "six_copper_layers": board.GetCopperLayerCount() == 6,
        "footprint_parity": set(refs) == expected_refs and len(refs) == len(set(refs)),
        "all_footprints_outside_edge_cuts": not inside,
        "zero_tracks_and_vias": len(list(board.GetTracks())) == 0,
        "zero_copper_zones": len(list(board.Zones())) == 0,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "outline_extents_mm": outline,
        "copper_layers": board.GetCopperLayerCount(),
        "footprint_count": len(refs),
        "inside_or_overlapping_refs": inside,
        "track_via_count": len(list(board.GetTracks())),
        "zone_count": len(list(board.Zones())),
    }


def copy_flat_inputs(output: Path) -> list[Path]:
    names = [
        "esc_rev_b.kicad_pro",
        "esc_rev_b.kicad_sch",
        "controller.kicad_sch",
        "power.kicad_sch",
        "motor_1.kicad_sch",
        "motor_2.kicad_sch",
        "motor_3.kicad_sch",
        "motor_4.kicad_sch",
        "motor_5.kicad_sch",
        "motor_6.kicad_sch",
        "revb.kicad_sym",
        "sym-lib-table",
    ]
    copied: list[Path] = []
    for name in names:
        destination = output / name
        shutil.copy2(ESC_DIR / name, destination)
        copied.append(destination)
    return copied


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=ESC_DIR / "esc_rev_b.kicad_pcb")
    parser.add_argument("--netlist", type=Path, default=ESC_DIR / "reports" / "esc_rev_b_netlist.xml")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    board_path = args.output / "esc_rev_b.kicad_pcb"
    build = build_board(args.source, board_path, args.netlist)
    copied = copy_flat_inputs(args.output)
    audit = validate(board_path, build["expected_refs"])
    audit["group_populations"] = build["group_populations"]
    audit["group_bounds_mm"] = build["group_bounds_mm"]
    audit_path = args.output / "QUILTER_INPUT_AUDIT.json"
    audit_path.write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
    readme = args.output / "README_UPLOAD.txt"
    readme.write_text(
        "Quilter upload set for the Rev-B six-channel ESC\n\n"
        "Upload the individual .kicad_pcb, .kicad_sch, .kicad_pro, and .kicad_sym files together.\n"
        "Do not ZIP the files; Quilter's current uploader does not accept ZIP or nested directories.\n"
        "The board is exactly 150 x 150 mm, has six copper layers, and contains 646 footprints.\n"
        "All footprints are outside Edge.Cuts, so Quilter may place and route every component.\n"
        "There are no starter traces, vias, or copper zones to be treated as locked routing.\n"
        "After parsing, verify Quilter reports 646 components, six layers, and a 150 x 150 mm outline.\n"
        "This high-current ESC still requires explicit placement constraints, busbar design, thermal review, double-pulse testing, and full-load validation.\n",
        encoding="utf-8",
    )
    files = [board_path, *copied, audit_path, readme]
    manifest = args.output / "SHA256SUMS.txt"
    manifest.write_text("".join(f"{sha256(path)}  {path.name}\n" for path in sorted(files)), encoding="utf-8")
    print(json.dumps(audit, indent=2))
    if not audit["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
