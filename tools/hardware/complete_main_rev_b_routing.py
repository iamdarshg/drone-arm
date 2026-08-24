#!/usr/bin/env python3
"""Finish residual Rev-B control-board airwires with a constrained 2-layer A*.

FreeRouting is used for the bulk route.  This deterministic finishing pass reads
KiCad's JSON DRC report and joins only the remaining missing connections.  It
routes on F.Cu/B.Cu, models existing copper/pads/vias as net-aware obstacles, and
keeps local RF-network connections on F.Cu.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import pcbnew


NET_RE = re.compile(r"\[([^\]]+)\]")
PAD_RE = re.compile(r"Pad ([^ ]+) .* of ([^ ]+) on ")
GRID_MM = 0.25
CLEARANCE_MM = 0.125
DEFAULT_WIDTH_MM = 0.25
VIA_DIAMETER_MM = 0.60
VIA_DRILL_MM = 0.30
LAYERS = (pcbnew.F_Cu, pcbnew.B_Cu)
LAYER_INDEX = {layer: index for index, layer in enumerate(LAYERS)}


def mm(value: float) -> int:
    return pcbnew.FromMM(float(value))


def grid_point(x_mm: float, y_mm: float) -> tuple[int, int]:
    return round(x_mm / GRID_MM), round(y_mm / GRID_MM)


def board_point(point: tuple[int, int]) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(mm(point[0] * GRID_MM), mm(point[1] * GRID_MM))


def item_net(item: dict) -> str | None:
    match = NET_RE.search(item.get("description", ""))
    return match.group(1) if match else None


def find_pad(board: pcbnew.BOARD, item: dict):
    match = PAD_RE.search(item.get("description", ""))
    if not match:
        return None
    pad_number, reference = match.groups()
    footprint = board.FindFootprintByReference(reference)
    if footprint is None:
        return None
    for pad in footprint.Pads():
        if pad.GetNumber() == pad_number:
            return footprint, pad
    return None


def routed_endpoint(
    board: pcbnew.BOARD,
    item: dict,
    other_position: dict,
    bounds: tuple[int, int, int, int],
) -> tuple[pcbnew.VECTOR2I, pcbnew.VECTOR2I | None]:
    """Return A* endpoint and optional original pad point needing an escape."""
    exact = pcbnew.VECTOR2I(mm(item["pos"]["x"]), mm(item["pos"]["y"]))
    found = find_pad(board, item)
    if found is None:
        return exact, None
    footprint, pad = found
    pad_position = pad.GetPosition()
    footprint_position = footprint.GetPosition()
    dx = pcbnew.ToMM(pad_position.x - footprint_position.x)
    dy = pcbnew.ToMM(pad_position.y - footprint_position.y)
    magnitude = math.hypot(dx, dy)
    if magnitude < 0.05:
        dx = float(other_position["x"]) - pcbnew.ToMM(pad_position.x)
        dy = float(other_position["y"]) - pcbnew.ToMM(pad_position.y)
        magnitude = max(math.hypot(dx, dy), 0.001)
    dx /= magnitude
    dy /= magnitude

    escape_mm = 0.90
    escape_x = pcbnew.ToMM(pad_position.x) + dx * escape_mm
    escape_y = pcbnew.ToMM(pad_position.y) + dy * escape_mm
    gx, gy = grid_point(escape_x, escape_y)
    x_min, y_min, x_max, y_max = bounds
    if not (x_min <= gx <= x_max and y_min <= gy <= y_max):
        # Edge connectors must escape toward the board interior.
        escape_x = pcbnew.ToMM(pad_position.x) - dx * escape_mm
        escape_y = pcbnew.ToMM(pad_position.y) - dy * escape_mm
    return pcbnew.VECTOR2I(mm(escape_x), mm(escape_y)), pad_position


def route_width_mm(net_name: str) -> float:
    if net_name.startswith("/915 MHz RF/") and any(
        token in net_name
        for token in (
            "RF",
            "LNA",
            "PA",
            "MATCH",
            "SAW",
            "TR_SW",
            "RX_COMBINE",
            "ANT_915",
        )
    ):
        return 0.35
    return DEFAULT_WIDTH_MM


def top_only(net_name: str) -> bool:
    return net_name.startswith("/915 MHz RF/") and any(
        token in net_name
        for token in (
            "CC1121_RF",
            "CC1190_BIAS",
            "CC_LNA",
            "CC_RBIAS",
            "CC_PA",
            "MATCH_",
            "RF_",
            "SAW_",
            "TR_SW",
            "RX_COMBINE",
            "ANT_915",
        )
    )


def cells_in_radius(
    x_mm: float, y_mm: float, radius_mm: float
) -> set[tuple[int, int]]:
    cx, cy = grid_point(x_mm, y_mm)
    steps = math.ceil(radius_mm / GRID_MM)
    result: set[tuple[int, int]] = set()
    for dx in range(-steps, steps + 1):
        for dy in range(-steps, steps + 1):
            if math.hypot(dx * GRID_MM, dy * GRID_MM) <= radius_mm:
                result.add((cx + dx, cy + dy))
    return result


def cells_along_segment(
    start: pcbnew.VECTOR2I,
    end: pcbnew.VECTOR2I,
    radius_mm: float,
) -> set[tuple[int, int]]:
    x0, y0 = pcbnew.ToMM(start.x), pcbnew.ToMM(start.y)
    x1, y1 = pcbnew.ToMM(end.x), pcbnew.ToMM(end.y)
    length = math.hypot(x1 - x0, y1 - y0)
    samples = max(1, math.ceil(length / (GRID_MM / 2)))
    result: set[tuple[int, int]] = set()
    for index in range(samples + 1):
        ratio = index / samples
        x = x0 + ((x1 - x0) * ratio)
        y = y0 + ((y1 - y0) * ratio)
        result.update(cells_in_radius(x, y, radius_mm))
    return result


def add_cells(
    occupancy: list[dict[tuple[int, int], set[int]]],
    layer_index: int,
    cells: set[tuple[int, int]],
    net_code: int,
) -> None:
    for cell in cells:
        occupancy[layer_index][cell].add(net_code)


def build_occupancy(
    board: pcbnew.BOARD,
) -> tuple[
    list[dict[tuple[int, int], set[int]]],
    tuple[int, int, int, int],
]:
    occupancy: list[dict[tuple[int, int], set[int]]] = [
        defaultdict(set),
        defaultdict(set),
    ]
    candidate_half = DEFAULT_WIDTH_MM / 2

    for item in board.GetTracks():
        net_code = item.GetNetCode()
        if isinstance(item, pcbnew.PCB_VIA):
            position = item.GetPosition()
            radius = (
                pcbnew.ToMM(item.GetWidth(pcbnew.F_Cu)) / 2
                + CLEARANCE_MM
                + candidate_half
            )
            cells = cells_in_radius(
                pcbnew.ToMM(position.x), pcbnew.ToMM(position.y), radius
            )
            for layer_index in range(len(LAYERS)):
                add_cells(occupancy, layer_index, cells, net_code)
            continue
        layer_index = LAYER_INDEX.get(item.GetLayer())
        if layer_index is None:
            continue
        radius = (
            pcbnew.ToMM(item.GetWidth()) / 2
            + CLEARANCE_MM
            + candidate_half
        )
        add_cells(
            occupancy,
            layer_index,
            cells_along_segment(item.GetStart(), item.GetEnd(), radius),
            net_code,
        )

    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            net_code = pad.GetNetCode()
            bbox = pad.GetBoundingBox()
            expand = CLEARANCE_MM + candidate_half
            x0 = pcbnew.ToMM(bbox.GetX()) - expand
            y0 = pcbnew.ToMM(bbox.GetY()) - expand
            x1 = pcbnew.ToMM(bbox.GetRight()) + expand
            y1 = pcbnew.ToMM(bbox.GetBottom()) + expand
            gx0, gy0 = grid_point(x0, y0)
            gx1, gy1 = grid_point(x1, y1)
            cells = {
                (gx, gy)
                for gx in range(min(gx0, gx1), max(gx0, gx1) + 1)
                for gy in range(min(gy0, gy1), max(gy0, gy1) + 1)
            }
            for layer_index, layer in enumerate(LAYERS):
                if pad.IsOnLayer(layer):
                    add_cells(occupancy, layer_index, cells, net_code)

    bbox = board.GetBoardEdgesBoundingBox()
    margin = 0.75
    x_min, y_min = grid_point(
        pcbnew.ToMM(bbox.GetX()) + margin,
        pcbnew.ToMM(bbox.GetY()) + margin,
    )
    x_max, y_max = grid_point(
        pcbnew.ToMM(bbox.GetRight()) - margin,
        pcbnew.ToMM(bbox.GetBottom()) - margin,
    )
    return occupancy, (x_min, y_min, x_max, y_max)


def find_path(
    occupancy: list[dict[tuple[int, int], set[int]]],
    bounds: tuple[int, int, int, int],
    start: tuple[int, int],
    goal: tuple[int, int],
    net_code: int,
    allow_bottom: bool,
) -> list[tuple[int, int, int]] | None:
    x_min, y_min, x_max, y_max = bounds
    directions = ((1, 0), (-1, 0), (0, 1), (0, -1))
    endpoint_cells = cells_in_radius(
        start[0] * GRID_MM, start[1] * GRID_MM, 0.30
    ) | cells_in_radius(goal[0] * GRID_MM, goal[1] * GRID_MM, 0.30)

    def blocked(point: tuple[int, int], layer_index: int) -> bool:
        if layer_index == 0 and point in endpoint_cells:
            return False
        nets = occupancy[layer_index].get(point)
        return bool(nets and any(code != net_code for code in nets))

    def via_blocked(point: tuple[int, int]) -> bool:
        # Occupancy already includes the clearance required by a 0.25 mm
        # track.  A via needs another 0.175 mm of radial room.
        extra = (VIA_DIAMETER_MM / 2) - (DEFAULT_WIDTH_MM / 2)
        for cell in cells_in_radius(
            point[0] * GRID_MM, point[1] * GRID_MM, extra
        ):
            for layer_index in range(len(LAYERS)):
                nets = occupancy[layer_index].get(cell)
                if nets and any(code != net_code for code in nets):
                    return True
        return False

    # state: x, y, layer, direction-x, direction-y
    start_state = (start[0], start[1], 0, 0, 0)
    queue: list[tuple[float, float, tuple[int, int, int, int, int]]] = []
    heapq.heappush(queue, (0.0, 0.0, start_state))
    parents: dict[
        tuple[int, int, int, int, int],
        tuple[int, int, int, int, int] | None,
    ] = {start_state: None}
    best_cost = {start_state: 0.0}
    final_state = None

    while queue:
        _, cost, state = heapq.heappop(queue)
        if cost != best_cost.get(state):
            continue
        x, y, layer_index, old_dx, old_dy = state
        if (x, y) == goal and layer_index == 0:
            final_state = state
            break

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if not (x_min <= nx <= x_max and y_min <= ny <= y_max):
                continue
            if blocked((nx, ny), layer_index):
                continue
            turn_penalty = (
                0.0
                if (old_dx, old_dy) in ((0, 0), (dx, dy))
                else 0.30
            )
            next_cost = cost + 1.0 + turn_penalty
            next_state = (nx, ny, layer_index, dx, dy)
            if next_cost >= best_cost.get(next_state, math.inf):
                continue
            best_cost[next_state] = next_cost
            parents[next_state] = state
            heuristic = abs(nx - goal[0]) + abs(ny - goal[1])
            if layer_index != 0:
                heuristic += 12.0
            heapq.heappush(
                queue,
                (next_cost + heuristic, next_cost, next_state),
            )

        if allow_bottom and not via_blocked((x, y)):
            next_layer = 1 - layer_index
            next_cost = cost + 12.0
            next_state = (x, y, next_layer, 0, 0)
            if next_cost < best_cost.get(next_state, math.inf):
                best_cost[next_state] = next_cost
                parents[next_state] = state
                heuristic = abs(x - goal[0]) + abs(y - goal[1])
                if next_layer != 0:
                    heuristic += 12.0
                heapq.heappush(
                    queue,
                    (next_cost + heuristic, next_cost, next_state),
                )

    if final_state is None:
        return None
    result = []
    state = final_state
    while state is not None:
        result.append((state[0], state[1], state[2]))
        state = parents[state]
    result.reverse()
    return result


def simplify_path(
    path: list[tuple[int, int, int]],
) -> list[tuple[int, int, int]]:
    if len(path) <= 2:
        return path
    result = [path[0]]
    for index in range(1, len(path) - 1):
        previous = path[index - 1]
        current = path[index]
        following = path[index + 1]
        if current[2] != previous[2] or following[2] != current[2]:
            result.append(current)
            continue
        before_direction = (
            current[0] - previous[0],
            current[1] - previous[1],
        )
        after_direction = (
            following[0] - current[0],
            following[1] - current[1],
        )
        if before_direction != after_direction:
            result.append(current)
    result.append(path[-1])
    return result


def add_via(board: pcbnew.BOARD, point: pcbnew.VECTOR2I, net) -> None:
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(point)
    via.SetWidth(mm(VIA_DIAMETER_MM))
    via.SetDrill(mm(VIA_DRILL_MM))
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    via.SetNet(net)
    board.Add(via)


def description_is_inner(item: dict) -> bool:
    description = item.get("description", "")
    return " on In1.Cu" in description or " on In2.Cu" in description


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--drc", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.board))
    drc = json.loads(args.drc.read_text(encoding="utf-8"))
    occupancy, bounds = build_occupancy(board)
    issues = []
    for issue in drc.get("unconnected_items", []):
        items = issue.get("items", [])
        if len(items) != 2:
            continue
        net_name = item_net(items[0]) or item_net(items[1])
        if not net_name:
            continue
        first, second = items
        distance = math.hypot(
            float(second["pos"]["x"]) - float(first["pos"]["x"]),
            float(second["pos"]["y"]) - float(first["pos"]["y"]),
        )
        issues.append((not top_only(net_name), distance, net_name, items))
    issues.sort(key=lambda value: (value[0], value[1]))

    routed = []
    failed = []
    for _, _, net_name, items in issues:
        net = board.FindNet(net_name)
        if net is None:
            failed.append({"net": net_name, "reason": "net not found"})
            continue
        start_pos, goal_pos = items[0]["pos"], items[1]["pos"]
        start_exact, start_pad = routed_endpoint(
            board, items[0], goal_pos, bounds
        )
        goal_exact, goal_pad = routed_endpoint(
            board, items[1], start_pos, bounds
        )
        start = grid_point(
            pcbnew.ToMM(start_exact.x), pcbnew.ToMM(start_exact.y)
        )
        goal = grid_point(
            pcbnew.ToMM(goal_exact.x), pcbnew.ToMM(goal_exact.y)
        )
        path = find_path(
            occupancy,
            bounds,
            start,
            goal,
            net.GetNetCode(),
            allow_bottom=not top_only(net_name),
        )
        if path is None:
            failed.append({"net": net_name, "reason": "no path"})
            continue
        path = simplify_path(path)
        exact_start = start_exact
        exact_goal = goal_exact

        # Convert the grid path into per-layer polylines.  Exact DRC marker
        # coordinates are retained at the ends so tracks intersect the
        # reported pad/track/via objects.
        points = [
            (
                exact_start if index == 0 else exact_goal
                if index == len(path) - 1
                else board_point((state[0], state[1])),
                state[2],
            )
            for index, state in enumerate(path)
        ]
        width_mm = route_width_mm(net_name)
        for pad_point, escape_point in (
            (start_pad, exact_start),
            (goal_pad, exact_goal),
        ):
            if pad_point is None or pad_point == escape_point:
                continue
            escape_segment = pcbnew.PCB_TRACK(board)
            escape_segment.SetStart(pad_point)
            escape_segment.SetEnd(escape_point)
            escape_segment.SetLayer(pcbnew.F_Cu)
            escape_segment.SetWidth(mm(width_mm))
            escape_segment.SetNet(net)
            board.Add(escape_segment)
            radius = (
                width_mm / 2
                + CLEARANCE_MM
                + DEFAULT_WIDTH_MM / 2
            )
            add_cells(
                occupancy,
                0,
                cells_along_segment(pad_point, escape_point, radius),
                net.GetNetCode(),
            )
        for index in range(len(points) - 1):
            first, first_layer = points[index]
            second, second_layer = points[index + 1]
            if first_layer != second_layer:
                add_via(board, first, net)
                continue
            segment = pcbnew.PCB_TRACK(board)
            segment.SetStart(first)
            segment.SetEnd(second)
            segment.SetLayer(LAYERS[first_layer])
            segment.SetWidth(mm(width_mm))
            segment.SetNet(net)
            board.Add(segment)
            radius = (
                width_mm / 2
                + CLEARANCE_MM
                + DEFAULT_WIDTH_MM / 2
            )
            add_cells(
                occupancy,
                first_layer,
                cells_along_segment(first, second, radius),
                net.GetNetCode(),
            )

        # An inner-layer DRC endpoint needs a through via at the marker so the
        # new F.Cu route actually joins the original inner trace.
        if description_is_inner(items[0]):
            add_via(board, exact_start, net)
        if description_is_inner(items[1]):
            add_via(board, exact_goal, net)

        routed.append(
            {
                "net": net_name,
                "segments": sum(
                    1
                    for index in range(len(points) - 1)
                    if points[index][1] == points[index + 1][1]
                ),
                "vias": sum(
                    1
                    for index in range(len(points) - 1)
                    if points[index][1] != points[index + 1][1]
                )
                + sum(description_is_inner(item) for item in items),
            }
        )

    pcbnew.SaveBoard(str(args.output), board)
    print(
        json.dumps(
            {
                "routed": routed,
                "failed": failed,
                "output": str(args.output),
            },
            indent=2,
        )
    )
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
