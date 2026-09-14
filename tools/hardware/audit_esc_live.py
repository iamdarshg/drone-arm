#!/usr/bin/env python3
"""Write a compact, fail-closed audit for the current ESC PCB."""

from __future__ import annotations

import argparse
import collections
import hashlib
import json
from pathlib import Path

import pcbnew


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--drc", required=True, type=Path)
    parser.add_argument("--parity", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.board))
    board.BuildConnectivity()
    footprints = list(board.GetFootprints())
    tracks = list(board.GetTracks())
    vias = [item for item in tracks if item.Type() == pcbnew.PCB_VIA_T]
    via_positions = collections.Counter(
        (item.GetPosition().x, item.GetPosition().y) for item in vias
    )
    drc = json.loads(args.drc.read_text(encoding="utf-8"))
    parity = json.loads(args.parity.read_text(encoding="utf-8"))
    violations = drc.get("violations", [])
    violation_groups = collections.Counter(
        (item.get("type"), item.get("severity")) for item in violations
    )

    result = {
        "board": str(args.board),
        "board_sha256": sha256(args.board),
        "footprints": len(footprints),
        "front_footprints": sum(fp.GetLayer() == pcbnew.F_Cu for fp in footprints),
        "back_footprints": sum(fp.GetLayer() == pcbnew.B_Cu for fp in footprints),
        "track_segments": sum(item.Type() == pcbnew.PCB_TRACE_T for item in tracks),
        "vias": len(vias),
        "duplicate_via_positions": sum(
            count - 1 for count in via_positions.values() if count > 1
        ),
        "zones": len(board.Zones()),
        "true_unconnected_count": board.GetConnectivity().GetUnconnectedCount(False),
        "silkscreen_graphics": sum(
            1
            for fp in footprints
            for item in fp.GraphicalItems()
            if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS)
        )
        + sum(
            1
            for item in board.GetDrawings()
            if item.GetLayer() in (pcbnew.F_SilkS, pcbnew.B_SilkS)
        ),
        "drc": {
            "errors": sum(item.get("severity") == "error" for item in violations),
            "shorts": sum(item.get("type") == "shorting_items" for item in violations),
            "courtyard_overlaps": sum(
                item.get("type") == "courtyards_overlap" for item in violations
            ),
            "warnings": sum(item.get("severity") == "warning" for item in violations),
            "violation_groups": [
                {"type": kind, "severity": severity, "count": count}
                for (kind, severity), count in sorted(violation_groups.items())
            ],
            "reported_unconnected_items_capped": len(drc.get("unconnected_items", [])),
        },
        "netlist_parity": {
            "matched_pads": parity.get("matched_pads"),
            "missing_board_references": parity.get("missing_board_references", []),
            "stale_board_references": parity.get("stale_board_references", []),
            "pad_mismatches": parity.get("mismatch_count"),
        },
        "notes": [
            "The six board-only C1130/C1230/C1330/C1430/C1530/C1630 bulk capacitors are deliberately retained.",
            "Dangling-via warnings are expected in this placement and zone checkpoint because signal track routing has not started.",
        ],
    }
    result["gate_pass"] = (
        result["back_footprints"] == 0
        and result["duplicate_via_positions"] == 0
        and result["silkscreen_graphics"] == 0
        and result["drc"]["errors"] == 0
        and result["drc"]["shorts"] == 0
        and result["drc"]["courtyard_overlaps"] == 0
        and not result["netlist_parity"]["missing_board_references"]
        and result["netlist_parity"]["pad_mismatches"] == 0
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["gate_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
