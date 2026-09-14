#!/usr/bin/env python3
"""Build and verify the exact schematic-to-PCB endpoint contract for ESC routing."""

from __future__ import annotations

import argparse
import collections
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pcbnew


DENSITY_RADIUS_MM = 8.0


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def millimetres(value: int) -> float:
    return pcbnew.ToMM(value)


def schematic_contract(path: Path) -> tuple[set[str], dict[tuple[str, str], str]]:
    root = ET.parse(path).getroot()
    components = {
        item.attrib["ref"]
        for item in root.findall("./components/comp")
        if not item.attrib["ref"].startswith("#")
    }
    pins: dict[tuple[str, str], str] = {}
    for net in root.findall("./nets/net"):
        for node in net.findall("node"):
            key = (node.attrib["ref"], node.attrib["pin"])
            name = net.attrib["name"]
            if key in pins and pins[key] != name:
                raise RuntimeError(f"schematic pin {key} appears on multiple nets")
            pins[key] = name
    return components, pins


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--netlist", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    components, schematic_pins = schematic_contract(args.netlist)
    board = pcbnew.LoadBoard(str(args.board))
    footprints = {fp.GetReference(): fp for fp in board.GetFootprints()}
    missing_refs = sorted(components - footprints.keys())
    stale_refs = sorted(footprints.keys() - components)
    all_pad_points = [
        (millimetres(pad.GetPosition().x), millimetres(pad.GetPosition().y))
        for fp in footprints.values()
        for pad in fp.Pads()
    ]

    endpoints: list[dict[str, object]] = []
    mismatches: list[dict[str, object]] = []
    seen_uuids: set[str] = set()
    for ref in sorted(components & footprints.keys()):
        for pad in footprints[ref].Pads():
            number = pad.GetNumber()
            if not number:
                continue
            expected = schematic_pins.get((ref, number))
            actual = pad.GetNetname()
            actual_is_nc = not actual or actual.startswith("unconnected-(")
            if expected is None and actual_is_nc:
                continue
            if expected != actual:
                mismatches.append(
                    {"ref": ref, "pad": number, "expected": expected, "actual": actual}
                )
                continue
            uuid = pad.m_Uuid.AsString()
            if uuid in seen_uuids:
                raise RuntimeError(f"duplicate pad UUID {uuid}")
            seen_uuids.add(uuid)
            position = pad.GetPosition()
            x = millimetres(position.x)
            y = millimetres(position.y)
            neighbors = [
                (px, py)
                for px, py in all_pad_points
                if math.hypot(px - x, py - y) <= DENSITY_RADIUS_MM
            ]
            center_x = sum(px for px, _ in neighbors) / len(neighbors)
            center_y = sum(py for _, py in neighbors) / len(neighbors)
            endpoints.append(
                {
                    "ref": ref,
                    "pad": number,
                    "pad_uuid": uuid,
                    "net": actual,
                    "net_code": pad.GetNetCode(),
                    "position_mm": [round(x, 6), round(y, 6)],
                    "copper_layers_hex": pad.GetLayerSet().FmtHex(),
                    "local_pin_density_8mm": len(neighbors),
                    "distance_from_local_density_center_mm": round(
                        math.hypot(x - center_x, y - center_y), 6
                    ),
                }
            )

    nets: dict[str, list[dict[str, object]]] = collections.defaultdict(list)
    for endpoint in endpoints:
        nets[str(endpoint["net"])].append(endpoint)
    net_rows = []
    for name, members in nets.items():
        net_codes = {int(member["net_code"]) for member in members}
        if len(net_codes) != 1:
            raise RuntimeError(f"net {name} has inconsistent PCB net codes {net_codes}")
        net_rows.append(
            {
                "net": name,
                "net_code": next(iter(net_codes)),
                "endpoint_count": len(members),
                "priority": {
                    "density_sum": sum(
                        int(member["local_pin_density_8mm"]) for member in members
                    ),
                    "centrality_sum_mm": round(
                        sum(
                            float(member["distance_from_local_density_center_mm"])
                            for member in members
                        ),
                        6,
                    ),
                },
                "endpoint_uuids": sorted(str(member["pad_uuid"]) for member in members),
            }
        )
    net_rows.sort(
        key=lambda row: (
            -int(row["priority"]["density_sum"]),
            float(row["priority"]["centrality_sum_mm"]),
            str(row["net"]),
        )
    )

    result = {
        "board": str(args.board),
        "board_sha256": sha256(args.board),
        "netlist": str(args.netlist),
        "netlist_sha256": sha256(args.netlist),
        "board_footprints": len(footprints),
        "schematic_components": len(components),
        "missing_board_references": missing_refs,
        "stale_board_references": stale_refs,
        "pad_mismatch_count": len(mismatches),
        "pad_mismatches": mismatches,
        "endpoint_count": len(endpoints),
        "net_count": len(net_rows),
        "endpoints": sorted(
            endpoints,
            key=lambda item: (
                str(item["net"]), str(item["ref"]), str(item["pad"]), str(item["pad_uuid"])
            ),
        ),
        "nets_in_density_priority_order": net_rows,
    }
    result["gate_pass"] = not missing_refs and not mismatches
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                key: result[key]
                for key in (
                    "board_sha256",
                    "netlist_sha256",
                    "missing_board_references",
                    "stale_board_references",
                    "pad_mismatch_count",
                    "endpoint_count",
                    "net_count",
                    "gate_pass",
                )
            },
            indent=2,
        )
    )
    return 0 if result["gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
