#!/usr/bin/env python3
"""Repack the compact ESC with real courtyards and physical two-side clearance."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re

import pcbnew


MM = 1_000_000
GAP = 0.12


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


def contains(outer: Rect, inner: Rect) -> bool:
    return (
        inner.x >= outer.x
        and inner.y >= outer.y
        and inner.right <= outer.right
        and inner.bottom <= outer.bottom
    )


class MaxRectsBin:
    def __init__(self, name: str, layer: int, bounds: Rect) -> None:
        self.name = name
        self.layer = layer
        self.bounds = bounds
        self.free = [bounds]
        self.count = 0

    def candidates(self, width: float, height: float) -> list[tuple[tuple[float, ...], Rect, bool]]:
        choices: list[tuple[tuple[float, ...], Rect, bool]] = []
        for free in self.free:
            for rotated, (item_width, item_height) in (
                (False, (width + GAP, height + GAP)),
                (True, (height + GAP, width + GAP)),
            ):
                if item_width > free.w + 1e-6 or item_height > free.h + 1e-6:
                    continue
                used = Rect(free.x, free.y, item_width, item_height)
                short_side = min(free.w - item_width, free.h - item_height)
                long_side = max(free.w - item_width, free.h - item_height)
                waste = free.w * free.h - item_width * item_height
                choices.append(((short_side, long_side, waste, free.y, free.x), used, rotated))
        return sorted(choices, key=lambda item: item[0])

    def reserve(self, used: Rect) -> None:
        split: list[Rect] = []
        for free in self.free:
            if not intersects(free, used):
                split.append(free)
                continue
            if used.x > free.x:
                split.append(Rect(free.x, free.y, used.x - free.x, free.h))
            if used.right < free.right:
                split.append(Rect(used.right, free.y, free.right - used.right, free.h))
            if used.y > free.y:
                split.append(Rect(free.x, free.y, free.w, used.y - free.y))
            if used.bottom < free.bottom:
                split.append(Rect(free.x, used.bottom, free.w, free.bottom - used.bottom))
        useful = [rect for rect in split if rect.w >= 0.25 and rect.h >= 0.25]
        pruned: list[Rect] = []
        for index, rect in enumerate(useful):
            if any(index != other and contains(candidate, rect) for other, candidate in enumerate(useful)):
                continue
            pruned.append(rect)
        self.free = pruned

    def place(self, width: float, height: float) -> tuple[Rect, bool] | None:
        choices = self.candidates(width, height)
        if not choices:
            return None
        _, used, rotated = choices[0]
        self.reserve(used)
        self.count += 1
        return used, rotated


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


def has_drilled_pad(footprint: pcbnew.FOOTPRINT) -> bool:
    return any(pad.GetDrillSize().x > 0 or pad.GetDrillSize().y > 0 for pad in footprint.Pads())


def power_priority(ref: str, motor: int) -> bool:
    if ref == f"TH{motor}":
        return True
    match = re.search(r"(\d+)$", ref)
    if not match:
        return False
    number = int(match.group(1))
    base = 1000 + motor * 100
    prefix = ref.rstrip("0123456789")
    offset = number - base
    return (
        prefix == "Q"
        or (prefix == "U" and 1 <= offset <= 6)
        or (prefix == "D" and offset in {1, 2})
        or (prefix == "C" and (1 <= offset <= 7 or 30 <= offset <= 36))
        or (prefix == "R" and 1 <= offset <= 23)
    )


def normalize_footprint(footprint: pcbnew.FOOTPRINT) -> None:
    if footprint.GetLayer() == pcbnew.B_Cu:
        footprint.Flip(footprint.GetPosition(), False)
    footprint.SetOrientationDegrees(0.0)


def envelope_size(footprint: pcbnew.FOOTPRINT) -> tuple[float, float]:
    normalize_footprint(footprint)
    courtyard = footprint.GetCourtyard(pcbnew.F_CrtYd).BBox()
    if courtyard.GetWidth() and courtyard.GetHeight():
        return pcbnew.ToMM(courtyard.GetWidth()), pcbnew.ToMM(courtyard.GetHeight())
    boxes = [pad.GetBoundingBox() for pad in footprint.Pads()]
    if not boxes:
        box = footprint.GetBoundingBox()
        return pcbnew.ToMM(box.GetWidth()) + 0.2, pcbnew.ToMM(box.GetHeight()) + 0.2
    left = min(box.GetX() for box in boxes)
    top = min(box.GetY() for box in boxes)
    right = max(box.GetRight() for box in boxes)
    bottom = max(box.GetBottom() for box in boxes)
    return pcbnew.ToMM(right - left) + 0.2, pcbnew.ToMM(bottom - top) + 0.2


def place_footprint(
    footprint: pcbnew.FOOTPRINT,
    used: Rect,
    rotated: bool,
    layer: int,
) -> None:
    normalize_footprint(footprint)
    if layer == pcbnew.B_Cu:
        footprint.Flip(footprint.GetPosition(), False)
    footprint.SetOrientationDegrees(90.0 if rotated else 0.0)
    target_x = used.x + used.w / 2
    target_y = used.y + used.h / 2
    footprint.SetPosition(pcbnew.VECTOR2I(round(target_x * MM), round(target_y * MM)))
    courtyard_layer = pcbnew.F_CrtYd if layer == pcbnew.F_Cu else pcbnew.B_CrtYd
    courtyard = footprint.GetCourtyard(courtyard_layer).BBox()
    if courtyard.GetWidth() and courtyard.GetHeight():
        actual_x = (courtyard.GetX() + courtyard.GetRight()) / 2
        actual_y = (courtyard.GetY() + courtyard.GetBottom()) / 2
        position = footprint.GetPosition()
        footprint.SetPosition(
            pcbnew.VECTOR2I(
                round(position.x + target_x * MM - actual_x),
                round(position.y + target_y * MM - actual_y),
            )
        )


def place_in_best_bin(
    footprint: pcbnew.FOOTPRINT,
    dimensions: tuple[float, float],
    bins: list[MaxRectsBin],
) -> MaxRectsBin | None:
    width, height = dimensions
    choices: list[tuple[tuple[float, ...], MaxRectsBin, Rect, bool]] = []
    for index, candidate_bin in enumerate(bins):
        for score, used, rotated in candidate_bin.candidates(width, height):
            choices.append(((*score, index), candidate_bin, used, rotated))
    if not choices:
        return None
    _, selected, used, rotated = min(choices, key=lambda item: item[0])
    selected.reserve(used)
    selected.count += 1
    place_footprint(footprint, used, rotated, selected.layer)
    return selected


def ordered_refs(refs: list[str], dimensions: dict[str, tuple[float, float]]) -> list[str]:
    return sorted(refs, key=lambda ref: (-dimensions[ref][0] * dimensions[ref][1], ref))


def repack(board: pcbnew.BOARD) -> dict[str, object]:
    footprints = {footprint.GetReference(): footprint for footprint in board.GetFootprints()}
    dimensions = {ref: envelope_size(footprint) for ref, footprint in footprints.items()}
    motor_bounds = {
        1: Rect(0.5, 23.5, 50.0, 55.0),
        2: Rect(50.5, 23.5, 49.0, 55.0),
        3: Rect(99.5, 23.5, 50.0, 55.0),
        4: Rect(0.5, 78.5, 50.0, 55.0),
        5: Rect(50.5, 78.5, 49.0, 55.0),
        6: Rect(99.5, 78.5, 50.0, 55.0),
    }
    local_bins = {
        motor: {
            pcbnew.F_Cu: MaxRectsBin(f"m{motor}-front", pcbnew.F_Cu, bounds),
            pcbnew.B_Cu: MaxRectsBin(f"m{motor}-back", pcbnew.B_Cu, bounds),
        }
        for motor, bounds in motor_bounds.items()
    }
    strip_bins: dict[tuple[int, int], MaxRectsBin] = {}
    for motor in range(1, 7):
        column = (motor - 1) % 3
        x0 = (0.5, 50.5, 99.5)[column]
        width = (50.0, 49.0, 50.0)[column]
        bounds = Rect(x0, 0.5 if motor <= 3 else 133.5, width, 23.0 if motor <= 3 else 16.0)
        for layer, suffix in ((pcbnew.F_Cu, "front"), (pcbnew.B_Cu, "back")):
            strip_bins[(motor, layer)] = MaxRectsBin(f"strip-m{motor}-{suffix}", layer, bounds)

    motor_refs = {
        motor: [ref for ref in footprints if motor_for_ref(ref) == motor]
        for motor in range(1, 7)
    }
    shared_refs = [ref for ref in footprints if motor_for_ref(ref) is None]
    spilled: dict[int, list[str]] = {motor: [] for motor in range(1, 7)}

    for motor in range(1, 7):
        front = local_bins[motor][pcbnew.F_Cu]
        back = local_bins[motor][pcbnew.B_Cu]
        drilled = [ref for ref in motor_refs[motor] if has_drilled_pad(footprints[ref])]
        for ref in ordered_refs(drilled, dimensions):
            width, height = dimensions[ref]
            choices = front.candidates(width, height)
            if not choices:
                raise RuntimeError(f"No local drilled placement for {ref}")
            _, used, rotated = choices[0]
            front.reserve(used)
            front.count += 1
            back.reserve(used)
            place_footprint(footprints[ref], used, rotated, pcbnew.F_Cu)

        smd = [ref for ref in motor_refs[motor] if ref not in drilled]
        for ref in ordered_refs(smd, dimensions):
            preferred_layer = pcbnew.F_Cu if power_priority(ref, motor) else pcbnew.B_Cu
            preferred = local_bins[motor][preferred_layer]
            alternate = local_bins[motor][pcbnew.B_Cu if preferred_layer == pcbnew.F_Cu else pcbnew.F_Cu]
            selected = place_in_best_bin(footprints[ref], dimensions[ref], [preferred, alternate])
            if selected is None:
                spilled[motor].append(ref)

    drilled_shared = [ref for ref in shared_refs if has_drilled_pad(footprints[ref])]
    non_drilled_shared = [ref for ref in shared_refs if ref not in drilled_shared]
    all_front_strips = [strip_bins[(motor, pcbnew.F_Cu)] for motor in range(1, 7)]
    all_back_strips = [strip_bins[(motor, pcbnew.B_Cu)] for motor in range(1, 7)]
    for ref in ordered_refs(drilled_shared, dimensions):
        selected = place_in_best_bin(footprints[ref], dimensions[ref], all_front_strips)
        if selected is None:
            raise RuntimeError(f"No shared drilled placement for {ref}")
        mirror = next(
            candidate
            for candidate in all_back_strips
            if candidate.bounds == selected.bounds
        )
        footprint = footprints[ref]
        layer = pcbnew.F_CrtYd
        box = footprint.GetCourtyard(layer).BBox()
        used = Rect(
            pcbnew.ToMM(box.GetX()) - GAP / 2,
            pcbnew.ToMM(box.GetY()) - GAP / 2,
            pcbnew.ToMM(box.GetWidth()) + GAP,
            pcbnew.ToMM(box.GetHeight()) + GAP,
        )
        mirror.reserve(used)

    for motor in range(1, 7):
        row_motors = [1, 2, 3] if motor <= 3 else [4, 5, 6]
        preferred_bins = [strip_bins[(motor, pcbnew.B_Cu)], strip_bins[(motor, pcbnew.F_Cu)]]
        nearby_bins = [
            strip_bins[(other, layer)]
            for other in row_motors
            if other != motor
            for layer in (pcbnew.B_Cu, pcbnew.F_Cu)
        ]
        for ref in ordered_refs(spilled[motor], dimensions):
            if place_in_best_bin(footprints[ref], dimensions[ref], preferred_bins + nearby_bins) is None:
                raise RuntimeError(f"No adjacent strip placement for {ref} from motor {motor}")

    for ref in ordered_refs(non_drilled_shared, dimensions):
        prefix = ref.rstrip("0123456789")
        number_match = re.search(r"(\d+)$", ref)
        number = int(number_match.group(1)) if number_match else 0
        preferred = all_front_strips if number >= 700 or prefix in {"L", "F", "D"} else all_back_strips
        alternate = all_back_strips if preferred is all_front_strips else all_front_strips
        if place_in_best_bin(footprints[ref], dimensions[ref], preferred + alternate) is None:
            raise RuntimeError(f"No shared placement for {ref}")

    return {
        "motor_spill_counts": {str(motor): len(spilled[motor]) for motor in range(1, 7)},
        "bin_populations": {
            candidate.name: candidate.count
            for candidate in [
                *[local_bins[motor][layer] for motor in range(1, 7) for layer in (pcbnew.F_Cu, pcbnew.B_Cu)],
                *strip_bins.values(),
            ]
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("board", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or args.board
    board = pcbnew.LoadBoard(str(args.board))
    result = repack(board)
    pcbnew.SaveBoard(str(output), board)
    print(result)


if __name__ == "__main__":
    main()
