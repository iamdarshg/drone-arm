#!/usr/bin/env python3
"""Electrical-layout audit for the routed Rev-B ESC and control boards.

KiCad DRC proves geometric legality.  This script adds design-intent checks
that DRC cannot express conveniently: zero ratsnest edges, layer/via policy on
RF and gate-drive nets, and length/skew reporting for differential and Kelvin
pairs.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import pcbnew

from audit_routed_paths import audit_main_signal_paths


MM = 1_000_000.0


def net_name(item: pcbnew.BOARD_CONNECTED_ITEM) -> str:
    return str(item.GetNetname())


def track_length_mm(item: pcbnew.PCB_TRACK) -> float:
    start = item.GetStart()
    end = item.GetEnd()
    return math.hypot(end.x - start.x, end.y - start.y) / MM


def collect_routes(board: pcbnew.BOARD) -> dict[str, dict]:
    routes: dict[str, dict] = defaultdict(
        lambda: {
            "track_length_mm": 0.0,
            "track_count": 0,
            "via_count": 0,
            "layers": set(),
            "min_width_mm": None,
        }
    )
    for item in board.GetTracks():
        name = net_name(item)
        if not name:
            continue
        route = routes[name]
        if isinstance(item, pcbnew.PCB_VIA):
            route["via_count"] += 1
            continue
        route["track_count"] += 1
        route["track_length_mm"] += track_length_mm(item)
        route["layers"].add(str(board.GetLayerName(item.GetLayer())))
        width = item.GetWidth() / MM
        if route["min_width_mm"] is None or width < route["min_width_mm"]:
            route["min_width_mm"] = width
    return routes


def rounded_route(route: dict | None) -> dict:
    route = route or {
        "track_length_mm": 0.0,
        "track_count": 0,
        "via_count": 0,
        "layers": set(),
        "min_width_mm": None,
    }
    return {
        "track_length_mm": round(route["track_length_mm"], 3),
        "track_count": route["track_count"],
        "via_count": route["via_count"],
        "layers": sorted(route["layers"]),
        "min_width_mm": (
            None if route["min_width_mm"] is None else round(route["min_width_mm"], 3)
        ),
    }


def add_check(checks: list[dict], name: str, passed: bool, detail: object) -> None:
    checks.append({"name": name, "passed": bool(passed), "detail": detail})


def pair_check(
    checks: list[dict],
    routes: dict[str, dict],
    name: str,
    positive: str,
    negative: str,
    max_skew_mm: float,
) -> None:
    p_route = routes.get(positive)
    n_route = routes.get(negative)
    p_length = 0.0 if p_route is None else p_route["track_length_mm"]
    n_length = 0.0 if n_route is None else n_route["track_length_mm"]
    skew = abs(p_length - n_length)
    add_check(
        checks,
        name,
        p_route is not None and n_route is not None and skew <= max_skew_mm,
        {
            "positive": positive,
            "negative": negative,
            "positive_length_mm": round(p_length, 3),
            "negative_length_mm": round(n_length, 3),
            "skew_mm": round(skew, 3),
            "limit_mm": max_skew_mm,
        },
    )


def audit_main(board: pcbnew.BOARD, routes: dict[str, dict], checks: list[dict]) -> None:
    rf_prefixes = ("/915 MHz RF/", "/GNSS/GNSS_RF_")
    rf_routes = {
        name: rounded_route(route)
        for name, route in routes.items()
        if name.startswith(rf_prefixes)
    }
    rf_failures = {
        name: route
        for name, route in rf_routes.items()
        if route["via_count"] or any(layer != "F.Cu" for layer in route["layers"])
    }
    add_check(
        checks,
        "RF routes remain on F.Cu without vias",
        bool(rf_routes) and not rf_failures,
        {"audited_nets": len(rf_routes), "failures": rf_failures},
    )

    audit_main_signal_paths(board, routes, checks, add_check)


def motor_net(motor: int, suffix: str) -> str:
    return f"/Motor {motor} power cell/M{motor}_{suffix}"


def audit_esc(board: pcbnew.BOARD, routes: dict[str, dict], checks: list[dict]) -> None:
    gate_pattern = re.compile(r"^/Motor [1-6] power cell/M[1-6]_G[HL][ABC](?:_DRV)?$")
    gate_routes = {
        name: rounded_route(route)
        for name, route in routes.items()
        if gate_pattern.match(name)
    }
    gate_failures = {}
    for name, route in gate_routes.items():
        limit = 35.0 if name.endswith("_DRV") else 10.0
        if (
            route["via_count"]
            or any(layer != "F.Cu" for layer in route["layers"])
            or route["track_length_mm"] > limit
        ):
            gate_failures[name] = {**route, "length_limit_mm": limit}
    add_check(
        checks,
        "Gate-drive routes stay on F.Cu, use no vias, and meet length limits",
        len(gate_routes) == 72 and not gate_failures,
        {
            "expected_nets": 72,
            "audited_nets": len(gate_routes),
            "failures": gate_failures,
        },
    )

    sensitive_suffix = re.compile(
        r"_(?:SH[ABC]_[PN]|BUS_SH_[PN]|CSA_[ABC](?:_RAW)?|BUS_CURRENT(?:_RAW)?)$"
    )
    sensitive_routes = {
        name: rounded_route(route)
        for name, route in routes.items()
        if sensitive_suffix.search(name)
    }
    sensitive_failures = {
        name: route
        for name, route in sensitive_routes.items()
        if route["via_count"] or any(layer != "F.Cu" for layer in route["layers"])
    }
    add_check(
        checks,
        "Current-sense routes stay on F.Cu without vias",
        len(sensitive_routes) == 96 and not sensitive_failures,
        {
            "expected_nets": 96,
            "audited_nets": len(sensitive_routes),
            "failures": sensitive_failures,
        },
    )

    for motor in range(1, 7):
        for phase in "ABC":
            pair_check(
                checks,
                routes,
                f"M{motor} phase-{phase} Kelvin-pair skew",
                motor_net(motor, f"SH{phase}_P"),
                motor_net(motor, f"SH{phase}_N"),
                2.0,
            )
        pair_check(
            checks,
            routes,
            f"M{motor} bus Kelvin-pair skew",
            motor_net(motor, "BUS_SH_P"),
            motor_net(motor, "BUS_SH_N"),
            2.0,
        )

    pair_check(checks, routes, "ESC CAN connector pair skew", "/CANH", "/CANL", 2.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kind", choices=("esc", "main"), required=True)
    parser.add_argument("--board", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.board))
    board.BuildConnectivity()
    routes = collect_routes(board)
    checks: list[dict] = []
    unconnected = int(board.GetConnectivity().GetUnconnectedCount(False))
    add_check(
        checks,
        "Zero unrouted connections",
        unconnected == 0,
        {"unconnected_count": unconnected},
    )

    if args.kind == "main":
        audit_main(board, routes, checks)
    else:
        audit_esc(board, routes, checks)

    failures = [check for check in checks if not check["passed"]]
    result = {
        "board": str(args.board),
        "kind": args.kind,
        "passed": not failures,
        "summary": {
            "checks": len(checks),
            "failures": len(failures),
            "unconnected_count": unconnected,
            "track_count": sum(
                1 for item in board.GetTracks() if not isinstance(item, pcbnew.PCB_VIA)
            ),
            "via_count": sum(
                1 for item in board.GetTracks() if isinstance(item, pcbnew.PCB_VIA)
            ),
        },
        "checks": checks,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["summary"], indent=2))
    raise SystemExit(0 if result["passed"] else 2)


if __name__ == "__main__":
    main()
