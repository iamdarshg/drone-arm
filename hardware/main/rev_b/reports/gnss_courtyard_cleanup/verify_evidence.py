#!/usr/bin/env python3
"""Verify source-pinned DRC evidence for the GNSS fab-layer annotation move."""
from __future__ import annotations
import collections
import hashlib
import json
import subprocess
from pathlib import Path

SOURCE_COMMIT = "481274bad6fda4f1fcfd991c06b134bb563cc651"
BOARD_PATH = "hardware/main/rev_b/main_rev_b.kicad_pcb"
PROJECT_PATH = "hardware/main/rev_b/main_rev_b.kicad_pro"
SOURCE_BOARD_SHA256 = "095aa3fd44285b885c2a4ca68c05633b169dd053fbc8407d2160e8585e73336b"
CANDIDATE_BOARD_SHA256 = "28ebb3c19ab3e8f7d3c7ee9acd5ab427a1868142399c27cabd758c7b14c2613f"
PROJECT_SHA256 = "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
EVIDENCE = Path(__file__).resolve().parent

def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def signature(row: dict) -> tuple:
    items = []
    for item in row.get("items", []):
        pos = item.get("pos", {})
        items.append((item.get("description"), round(float(pos.get("x", 0)), 6), round(float(pos.get("y", 0)), 6)))
    return (row.get("type"), row.get("description"), tuple(sorted(items)))

def net_counts(rows: list[dict]) -> collections.Counter:
    counts = collections.Counter()
    for row in rows:
        names = set()
        for item in row.get("items", []):
            desc = item.get("description", "")
            if "[" in desc and "]" in desc:
                names.add(desc.split("[", 1)[1].split("]", 1)[0])
        counts.update(names)
    return counts

def main() -> None:
    root = Path(subprocess.check_output(["git", "rev-parse", "--show-toplevel"], text=True).strip())
    source_board = subprocess.check_output(["git", "show", f"{SOURCE_COMMIT}:{BOARD_PATH}"], cwd=root)
    candidate_board = (root / BOARD_PATH).read_bytes()
    project = (root / PROJECT_PATH).read_bytes()
    assert digest(source_board) == SOURCE_BOARD_SHA256, "source board SHA mismatch"
    assert digest(candidate_board) == CANDIDATE_BOARD_SHA256, "candidate board SHA mismatch"
    assert digest(project) == PROJECT_SHA256, "project SHA mismatch"

    source = json.loads((EVIDENCE / "source_drc.json").read_text())
    candidate = json.loads((EVIDENCE / "candidate_drc.json").read_text())
    assert source.get("kicad_version") == candidate.get("kicad_version") == "9.0.2"
    old = collections.Counter(signature(x) for x in source["violations"])
    new = collections.Counter(signature(x) for x in candidate["violations"])
    assert len(source["violations"]) == 7 and len(candidate["violations"]) == 4
    assert not (new - old), f"new DRC signatures: {list((new-old).elements())}"
    cleared = old - new
    assert len(cleared) == 3 and collections.Counter(sig[0] for sig in cleared.elements()) == {
        "silk_edge_clearance": 2, "silk_overlap": 1
    }
    assert len(source["unconnected_items"]) == len(candidate["unconnected_items"]) == 42
    assert net_counts(source["unconnected_items"]) == net_counts(candidate["unconnected_items"])
    print("PASS: board/project hashes and paired KiCad 9.0.2 DRC evidence verify")
    print("DRC: 7 -> 4 violations; 3 silk signatures cleared, 0 new; 42 opens unchanged")

if __name__ == "__main__":
    main()
