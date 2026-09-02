#!/usr/bin/env python3
"""Convert the compact Rev-B ESC to front-side-only component placement."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
from prepare_quilter_upload import schematic_groups  # noqa: E402


MM = 1_000_000
MARGIN = 0.15
CLIPPED_CELL_GUIDES = {
    "1fa2306e-6a0f-499b-b112-dff6e43a0030",
    "35f2e04f-1a5b-42e2-84fe-a40cd9be7015",
    "6e4953c9-8056-41ed-b658-d153c355cebb",
}


@dataclass(frozen=True)
class Rect:
    x: float
    y: float
    w: float
    h: float

    @property
    def right(self) -> float:
        return self.x + self.w

    @property
    def bottom(self) -> float:
        return self.y + self.h


def intersects(a: Rect, b: Rect) -> bool:
    return a.x < b.right and a.right > b.x and a.y < b.bottom and a.bottom > b.y


def contained(inner: Rect, outer: Rect) -> bool:
    return (
        inner.x >= outer.x
        and inner.y >= outer.y
        and inner.right <= outer.right
        and inner.bottom <= outer.bottom
    )


def split_free_rectangles(free: list[Rect], used: Rect) -> list[Rect]:
    split: list[Rect] = []
    for candidate in free:
        if not intersects(candidate, used):
            split.append(candidate)
            continue
        if used.x > candidate.x:
            split.append(Rect(candidate.x, candidate.y, used.x - candidate.x, candidate.h))
        if used.right < candidate.right:
            split.append(Rect(used.right, candidate.y, candidate.right - used.right, candidate.h))
        if used.y > candidate.y:
            split.append(Rect(candidate.x, candidate.y, candidate.w, used.y - candidate.y))
        if used.bottom < candidate.bottom:
            split.append(Rect(candidate.x, used.bottom, candidate.w, candidate.bottom - used.bottom))

    useful = [rect for rect in split if rect.w >= 0.3 and rect.h >= 0.3]
    pruned: list[Rect] = []
    for index, rect in enumerate(useful):
        if any(index != other and contained(rect, candidate) for other, candidate in enumerate(useful)):
            continue
        pruned.append(rect)
    return pruned


def pad_bbox_mm(footprint: pcbnew.FOOTPRINT) -> Rect:
    boxes = [pad.GetBoundingBox() for pad in footprint.Pads()]
    if not boxes:
        box = footprint.GetBoundingBox()
        return Rect(box.GetX() / MM, box.GetY() / MM, box.GetWidth() / MM, box.GetHeight() / MM)
    left = min(box.GetX() for box in boxes) / MM
    top = min(box.GetY() for box in boxes) / MM
    right = max(box.GetRight() for box in boxes) / MM
    bottom = max(box.GetBottom() for box in boxes) / MM
    return Rect(left, top, right - left, bottom - top)


def body_bbox_mm(board: pcbnew.BOARD, footprint: pcbnew.FOOTPRINT) -> Rect:
    """Return the physical package envelope, excluding text and courtyards."""
    boxes = [pad.GetBoundingBox() for pad in footprint.Pads()]
    boxes.extend(
        item.GetBoundingBox()
        for item in footprint.GraphicalItems()
        if board.GetLayerName(item.GetLayer()) == "F.Fab"
        and not isinstance(item, pcbnew.PCB_TEXT)
    )
    if not boxes:
        return pad_bbox_mm(footprint)
    left = min(box.GetX() for box in boxes) / MM
    top = min(box.GetY() for box in boxes) / MM
    right = max(box.GetRight() for box in boxes) / MM
    bottom = max(box.GetBottom() for box in boxes) / MM
    return Rect(left, top, right - left, bottom - top)


def contain_package_bodies(board: pcbnew.BOARD) -> None:
    """Pull package bodies inside the 150 mm outline after dense pad packing."""
    for footprint in board.GetFootprints():
        box = body_bbox_mm(board, footprint)
        dx = max(0.0, -box.x) - max(0.0, box.right - 150.0)
        dy = max(0.0, -box.y) - max(0.0, box.bottom - 150.0)
        if abs(dx) > 1e-9 or abs(dy) > 1e-9:
            position = footprint.GetPosition()
            footprint.SetPosition(
                pcbnew.VECTOR2I(position.x + round(dx * MM), position.y + round(dy * MM))
            )


def remove_clipped_silkscreen_guides(board: pcbnew.BOARD) -> None:
    for drawing in list(board.GetDrawings()):
        if drawing.m_Uuid.AsString() in CLIPPED_CELL_GUIDES:
            board.Remove(drawing)


def item_dimensions(footprint: pcbnew.FOOTPRINT) -> tuple[float, float]:
    box = pad_bbox_mm(footprint)
    return box.w + 2 * MARGIN, box.h + 2 * MARGIN


def place_footprint(footprint: pcbnew.FOOTPRINT, target: Rect, rotate: bool) -> None:
    if rotate:
        footprint.SetOrientationDegrees(footprint.GetOrientationDegrees() + 90.0)
    box = pad_bbox_mm(footprint)
    position = footprint.GetPosition()
    dx = target.x + MARGIN - box.x
    dy = target.y + MARGIN - box.y
    footprint.SetPosition(
        pcbnew.VECTOR2I(position.x + round(dx * MM), position.y + round(dy * MM))
    )


def pack_bin(footprints: list[pcbnew.FOOTPRINT], bounds: Rect) -> None:
    free = [bounds]
    ordered = sorted(
        footprints,
        key=lambda footprint: item_dimensions(footprint)[0] * item_dimensions(footprint)[1],
        reverse=True,
    )
    for footprint in ordered:
        width, height = item_dimensions(footprint)
        best: tuple[float, float, int, Rect, bool] | None = None
        for index, candidate in enumerate(free):
            for rotated, (item_width, item_height) in (
                (False, (width, height)),
                (True, (height, width)),
            ):
                if item_width > candidate.w + 1e-6 or item_height > candidate.h + 1e-6:
                    continue
                short_side = min(candidate.w - item_width, candidate.h - item_height)
                long_side = max(candidate.w - item_width, candidate.h - item_height)
                score = (short_side, long_side, index, candidate, rotated)
                if best is None or score[:2] < best[:2]:
                    best = score
        if best is None:
            raise RuntimeError(
                f"cannot pack {footprint.GetReference()} ({width:.2f} x {height:.2f} mm) "
                f"inside bin {bounds}"
            )
        _, _, _, candidate, rotated = best
        item_width, item_height = (height, width) if rotated else (width, height)
        used = Rect(candidate.x, candidate.y, item_width, item_height)
        place_footprint(footprint, used, rotated)
        free = split_free_rectangles(free, used)


def repack(board: pcbnew.BOARD, netlist: Path) -> None:
    groups, _ = schematic_groups(netlist)
    bins = [
        Rect(1.5, 1.5, 48.0, 73.0),
        Rect(50.5, 1.5, 49.0, 73.0),
        Rect(100.5, 1.5, 48.0, 73.0),
        Rect(1.5, 75.5, 48.0, 73.0),
        Rect(50.5, 75.5, 49.0, 73.0),
        Rect(100.5, 75.5, 48.0, 73.0),
    ]
    allocations: list[list[pcbnew.FOOTPRINT]] = [[] for _ in bins]
    extras: dict[str, list[pcbnew.FOOTPRINT]] = {"controller": [], "power": []}
    for footprint in board.GetFootprints():
        group = groups[footprint.GetReference()]
        if group.startswith("motor_"):
            allocations[int(group.removeprefix("motor_")) - 1].append(footprint)
        else:
            extras[group].append(footprint)

    for group, allowed_bins in (("controller", range(3)), ("power", range(3, 6))):
        extra_area = {index: 0.0 for index in allowed_bins}
        for footprint in sorted(
            extras[group],
            key=lambda item: item_dimensions(item)[0] * item_dimensions(item)[1],
            reverse=True,
        ):
            target = min(extra_area, key=extra_area.get)
            allocations[target].append(footprint)
            width, height = item_dimensions(footprint)
            extra_area[target] += width * height

    for footprints, bounds in zip(allocations, bins):
        pack_bin(footprints, bounds)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("board", type=Path)
    parser.add_argument("--netlist", type=Path)
    parser.add_argument("--pack", action="store_true")
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.board))
    flipped = 0
    for footprint in board.GetFootprints():
        if footprint.GetLayer() == pcbnew.B_Cu:
            footprint.Flip(footprint.GetPosition(), False)
            flipped += 1

    if args.pack:
        if args.netlist is None:
            raise SystemExit("--netlist is required with --pack")
        repack(board, args.netlist)
        remove_clipped_silkscreen_guides(board)

    pcbnew.SaveBoard(str(args.board), board)
    print(f"flipped_to_front={flipped}")


if __name__ == "__main__":
    main()
