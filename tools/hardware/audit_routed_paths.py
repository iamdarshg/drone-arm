#!/usr/bin/env python3
"""Layer-aware routed-path checks for branched Rev-B signal nets."""

from __future__ import annotations

import heapq
import math
from collections import defaultdict
from collections.abc import Callable

import pcbnew

MM = 1_000_000.0
PathSpec = tuple[str, tuple[str, str], tuple[str, str]]
CheckWriter = Callable[[list[dict], str, bool, object], None]


def _net_name(item: pcbnew.BOARD_CONNECTED_ITEM) -> str:
    return str(item.GetNetname())


def _track_length_mm(item: pcbnew.PCB_TRACK) -> float:
    start = item.GetStart()
    end = item.GetEnd()
    return math.hypot(end.x - start.x, end.y - start.y) / MM


def collect_route_graphs(board: pcbnew.BOARD) -> dict[str, dict]:
    """Build layer-aware copper graphs; vias join layers at one coordinate."""
    graphs: dict[str, dict] = defaultdict(
        lambda: {
            "adjacency": defaultdict(list),
            "nodes_by_xy": defaultdict(set),
            "via_positions": set(),
        }
    )
    for item in board.GetTracks():
        name = _net_name(item)
        if not name:
            continue
        graph = graphs[name]
        if isinstance(item, pcbnew.PCB_VIA):
            position = item.GetPosition()
            graph["via_positions"].add((position.x, position.y))
            continue
        start = item.GetStart()
        end = item.GetEnd()
        layer = int(item.GetLayer())
        start_node = (start.x, start.y, layer)
        end_node = (end.x, end.y, layer)
        length = _track_length_mm(item)
        graph["adjacency"][start_node].append((end_node, length))
        graph["adjacency"][end_node].append((start_node, length))
        graph["nodes_by_xy"][(start.x, start.y)].add(start_node)
        graph["nodes_by_xy"][(end.x, end.y)].add(end_node)

    for graph in graphs.values():
        adjacency = graph["adjacency"]
        for position in graph["via_positions"]:
            nodes = list(graph["nodes_by_xy"].get(position, ()))
            for index, first in enumerate(nodes):
                for second in nodes[index + 1 :]:
                    adjacency[first].append((second, 0.0))
                    adjacency[second].append((first, 0.0))
    return graphs


def _pad_nodes(
    board: pcbnew.BOARD,
    graph: dict,
    reference: str,
    pad_number: str,
) -> tuple[set[tuple[int, int, int]], dict]:
    footprint = board.FindFootprintByReference(reference)
    if footprint is None:
        return set(), {
            "reference": reference,
            "pad": pad_number,
            "error": "missing footprint",
        }
    pad = footprint.FindPadByNumber(str(pad_number))
    if pad is None:
        return set(), {
            "reference": reference,
            "pad": pad_number,
            "error": "missing pad",
        }
    position = pad.GetPosition()
    nodes = set(graph["nodes_by_xy"].get((position.x, position.y), ()))
    if not nodes:
        tolerance = int(0.05 * MM)
        nearest_distance = None
        nearest_nodes: set[tuple[int, int, int]] = set()
        for (x, y), candidate_nodes in graph["nodes_by_xy"].items():
            distance = math.hypot(x - position.x, y - position.y)
            if distance > tolerance:
                continue
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_nodes = set(candidate_nodes)
        nodes = nearest_nodes
    return nodes, {
        "reference": reference,
        "pad": str(pad_number),
        "position_mm": [round(position.x / MM, 4), round(position.y / MM, 4)],
        "route_nodes": len(nodes),
    }


def _shortest_path_mm(
    adjacency: dict,
    sources: set[tuple[int, int, int]],
    targets: set[tuple[int, int, int]],
) -> float | None:
    if not sources or not targets:
        return None
    distances: dict[tuple[int, int, int], float] = {}
    queue: list[tuple[float, tuple[int, int, int]]] = []
    for source in sources:
        distances[source] = 0.0
        heapq.heappush(queue, (0.0, source))
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances.get(node):
            continue
        if node in targets:
            return distance
        for neighbour, edge_length in adjacency.get(node, ()):
            candidate = distance + edge_length
            if candidate < distances.get(neighbour, math.inf):
                distances[neighbour] = candidate
                heapq.heappush(queue, (candidate, neighbour))
    return None


def routed_path_length(
    board: pcbnew.BOARD,
    graphs: dict[str, dict],
    net: str,
    source: tuple[str, str],
    target: tuple[str, str],
) -> tuple[float | None, dict]:
    graph = graphs.get(net)
    if graph is None:
        return None, {"net": net, "error": "net has no routed copper"}
    source_nodes, source_detail = _pad_nodes(board, graph, *source)
    target_nodes, target_detail = _pad_nodes(board, graph, *target)
    length = _shortest_path_mm(graph["adjacency"], source_nodes, target_nodes)
    return length, {
        "net": net,
        "source": source_detail,
        "target": target_detail,
        "length_mm": None if length is None else round(length, 3),
    }


def _chain_length(
    board: pcbnew.BOARD,
    graphs: dict[str, dict],
    paths: list[PathSpec],
) -> tuple[float | None, list[dict]]:
    total = 0.0
    details = []
    found = True
    for net, source, target in paths:
        length, detail = routed_path_length(board, graphs, net, source, target)
        details.append(detail)
        if length is None:
            found = False
        else:
            total += length
    return (total if found else None), details


def _chain_pair_check(
    checks: list[dict],
    add_check: CheckWriter,
    board: pcbnew.BOARD,
    graphs: dict[str, dict],
    routes: dict[str, dict],
    name: str,
    comparisons: list[tuple[str, list[PathSpec], list[PathSpec]]],
    max_skew_mm: float,
) -> None:
    comparison_details = []
    all_found = True
    max_skew = 0.0
    involved_nets: set[str] = set()
    for label, positive_paths, negative_paths in comparisons:
        involved_nets.update(net for net, _source, _target in positive_paths)
        involved_nets.update(net for net, _source, _target in negative_paths)
        positive, positive_details = _chain_length(board, graphs, positive_paths)
        negative, negative_details = _chain_length(board, graphs, negative_paths)
        found = positive is not None and negative is not None
        all_found = all_found and found
        skew = None if not found else abs(positive - negative)
        if skew is not None:
            max_skew = max(max_skew, skew)
        comparison_details.append(
            {
                "label": label,
                "positive": {
                    "paths": positive_details,
                    "total_length_mm": None if positive is None else round(positive, 3),
                },
                "negative": {
                    "paths": negative_details,
                    "total_length_mm": None if negative is None else round(negative, 3),
                },
                "skew_mm": None if skew is None else round(skew, 3),
            }
        )
    add_check(
        checks,
        name,
        all_found and max_skew <= max_skew_mm,
        {
            "comparisons": comparison_details,
            "max_skew_mm": round(max_skew, 3) if all_found else None,
            "limit_mm": max_skew_mm,
            "measurement": "complete-routed-pad-path-chain",
            "diagnostic_total_net_lengths_mm": {
                net: round(routes.get(net, {}).get("track_length_mm", 0.0), 3)
                for net in sorted(involved_nets)
            },
        },
    )


def _duplicate_pin_branch_check(
    checks: list[dict],
    add_check: CheckWriter,
    board: pcbnew.BOARD,
    graphs: dict[str, dict],
    max_delta_mm: float,
) -> None:
    branches = [
        (
            "D+ A6 versus B6",
            "/Power and USB/USB_DP_CONN",
            ("U1", "3"),
            ("J1", "A6"),
            ("J1", "B6"),
        ),
        (
            "D- A7 versus B7",
            "/Power and USB/USB_DN_CONN",
            ("U1", "1"),
            ("J1", "A7"),
            ("J1", "B7"),
        ),
    ]
    details = []
    all_found = True
    max_delta = 0.0
    for label, net, source, first_target, second_target in branches:
        first, first_detail = routed_path_length(
            board, graphs, net, source, first_target
        )
        second, second_detail = routed_path_length(
            board, graphs, net, source, second_target
        )
        found = first is not None and second is not None
        all_found = all_found and found
        delta = None if not found else abs(first - second)
        if delta is not None:
            max_delta = max(max_delta, delta)
        details.append(
            {
                "label": label,
                "net": net,
                "first": first_detail,
                "second": second_detail,
                "delta_mm": None if delta is None else round(delta, 3),
            }
        )
    add_check(
        checks,
        "USB-C duplicate-pin branch balance",
        all_found and max_delta <= max_delta_mm,
        {
            "branches": details,
            "max_delta_mm": round(max_delta, 3) if all_found else None,
            "limit_mm": max_delta_mm,
            "measurement": "same-net-duplicate-pin-path-delta",
        },
    )


def audit_main_signal_paths(
    board: pcbnew.BOARD,
    routes: dict[str, dict],
    checks: list[dict],
    add_check: CheckWriter,
) -> None:
    graphs = collect_route_graphs(board)

    usb_dp_common: list[PathSpec] = [
        ("/USB_DP", ("U1", "4"), ("R10", "1")),
        ("/RP2354B MCU/USB_DP_MCU", ("R10", "2"), ("U10", "67")),
    ]
    usb_dm_common: list[PathSpec] = [
        ("/USB_DM", ("U1", "6"), ("R11", "1")),
        ("/RP2354B MCU/USB_DM_MCU", ("R11", "2"), ("U10", "66")),
    ]
    _chain_pair_check(
        checks,
        add_check,
        board,
        graphs,
        routes,
        "USB connector-to-MCU pair skew",
        [
            (
                "USB-C A orientation",
                [
                    (
                        "/Power and USB/USB_DP_CONN",
                        ("U1", "3"),
                        ("J1", "A6"),
                    ),
                    *usb_dp_common,
                ],
                [
                    (
                        "/Power and USB/USB_DN_CONN",
                        ("U1", "1"),
                        ("J1", "A7"),
                    ),
                    *usb_dm_common,
                ],
            ),
            (
                "USB-C B orientation",
                [
                    (
                        "/Power and USB/USB_DP_CONN",
                        ("U1", "3"),
                        ("J1", "B6"),
                    ),
                    *usb_dp_common,
                ],
                [
                    (
                        "/Power and USB/USB_DN_CONN",
                        ("U1", "1"),
                        ("J1", "B7"),
                    ),
                    *usb_dm_common,
                ],
            ),
        ],
        0.5,
    )
    _duplicate_pin_branch_check(checks, add_check, board, graphs, 0.5)

    _chain_pair_check(
        checks,
        add_check,
        board,
        graphs,
        routes,
        "CAN transceiver-to-connector pair skew",
        [
            (
                "transceiver through common-mode choke to connector",
                [
                    (
                        "/CAN interface/U50_CANH",
                        ("U50", "7"),
                        ("FL50", "1"),
                    ),
                    ("/CANH", ("FL50", "2"), ("J2", "3")),
                ],
                [
                    (
                        "/CAN interface/U50_CANL",
                        ("U50", "6"),
                        ("FL50", "3"),
                    ),
                    ("/CANL", ("FL50", "4"), ("J2", "4")),
                ],
            )
        ],
        2.0,
    )
