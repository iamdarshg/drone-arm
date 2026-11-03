#!/usr/bin/env python3
"""Recheck the source-pinned GPIO2 routing increment with KiCad 9.0.2."""

from __future__ import annotations

import collections
import hashlib
import json
import pathlib
import subprocess
import tempfile

import pcbnew


ROOT = pathlib.Path(__file__).resolve().parents[5]
REPORTS = pathlib.Path(__file__).resolve().parent
BOARD_REL = pathlib.Path("hardware/main/rev_b/main_rev_b.kicad_pcb")
PROJECT_REL = pathlib.Path("hardware/main/rev_b/main_rev_b.kicad_pro")
SOURCE_COMMIT = "a007a13d1b3828fd9fcb128ade1af097c901b3d0"
SOURCE_BOARD_SHA256 = "fde09ade461c58ad4b3cc1952dca15e961e417dea0e5875055b503114e83be3a"
SOURCE_PREP_SHA256 = "19f3405fee98c1da4a156c6db314f16d2fac410a064ccc927a9e7ce96b43890d"
PREP_PROJECT_SHA256 = "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
ROUTE_SCRIPT_SHA256 = "b6ee5393decea360110c47caa721c83eca8bd05f995e1a02f76d3ba866b07943"
CANDIDATE_SHA256 = "7923ad03efefebfba5251be189821cd3ed78e588c92cb7c62774082a207ed41d"


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_blob(revision: str, path: pathlib.Path) -> bytes:
    return subprocess.check_output(
        ["git", "-C", str(ROOT), "show", f"{revision}:{path.as_posix()}"]
    )


def pad_map(path: pathlib.Path) -> dict[tuple[str, str], tuple[object, ...]]:
    board = pcbnew.LoadBoard(str(path))
    return {
        (footprint.GetReference(), pad.GetNumber()): (
            pad.GetNetname(),
            pad.GetPosition().x,
            pad.GetPosition().y,
            pad.GetSize().x,
            pad.GetSize().y,
            pad.GetShape(),
        )
        for footprint in board.GetFootprints()
        for pad in footprint.Pads()
    }


def copper_map(path: pathlib.Path) -> collections.Counter:
    board = pcbnew.LoadBoard(str(path))
    copper = collections.Counter()
    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA):
            position = item.GetPosition()
            key = (
                item.GetNetname(), "via", position.x, position.y,
                item.GetViaType(), item.TopLayer(), item.BottomLayer(),
                item.GetWidth(item.TopLayer()), item.GetDrillValue(),
            )
        else:
            start, end = item.GetStart(), item.GetEnd()
            key = (
                item.GetNetname(), "track", item.GetLayer(),
                tuple(sorted(((start.x, start.y), (end.x, end.y)))),
                item.GetWidth(),
            )
        copper[key] += 1
    return copper


def run_drc(board: pathlib.Path, report: pathlib.Path) -> dict:
    subprocess.run(
        [
            "kicad-cli", "pcb", "drc", "--severity-all", "--format", "json",
            "-o", str(report), str(board),
        ],
        check=True,
        cwd=board.parent,
    )
    return json.loads(report.read_text())


def open_nets(drc: dict) -> set[str]:
    nets = set()
    for finding in drc.get("unconnected_items", []):
        for item in finding.get("items", []):
            description = item.get("description", "")
            left = description.find("[/")
            right = description.find("]", left)
            if left >= 0 and right > left:
                nets.add(description[left + 1 : right])
    return nets


def main() -> None:
    candidate = ROOT / BOARD_REL
    assert sha256(candidate) == CANDIDATE_SHA256, "candidate board hash changed"
    assert sha256(ROOT / PROJECT_REL) == "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
    prep = REPORTS / "prep.kicad_pcb"
    project = REPORTS / "prep.kicad_pro"
    route_script = REPORTS / "route_gpio2.py"
    assert sha256(prep) == SOURCE_PREP_SHA256, "prep board hash changed"
    assert sha256(project) == PREP_PROJECT_SHA256, "prep project hash changed"
    assert sha256(route_script) == ROUTE_SCRIPT_SHA256, "route script hash changed"

    replayed = REPORTS / "replayed_candidate.kicad_pcb"
    subprocess.run(
        ["/usr/bin/python3", str(route_script)],
        check=True,
        cwd=REPORTS,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    assert copper_map(replayed) == copper_map(candidate), "source-pinned route replay changed copper geometry"
    replayed.unlink()
    for suffix in (".kicad_prl", ".kicad_pro"):
        (REPORTS / f"replayed_candidate{suffix}").unlink(missing_ok=True)

    with tempfile.TemporaryDirectory(prefix="rf-gpio2-verify-") as temp_name:
        temp = pathlib.Path(temp_name)
        source = temp / BOARD_REL.name
        source.write_bytes(git_blob(SOURCE_COMMIT, BOARD_REL))
        (temp / PROJECT_REL.name).write_bytes(git_blob(SOURCE_COMMIT, PROJECT_REL))
        assert sha256(source) == SOURCE_BOARD_SHA256, "source board hash changed"

        baseline_drc = run_drc(source, REPORTS / "baseline.drc.json")
        candidate_drc = run_drc(candidate, REPORTS / "candidate.drc.json")
        source_pads = pad_map(source)
        candidate_pads = pad_map(candidate)
        source_copper = copper_map(source)
        candidate_copper = copper_map(candidate)

    assert source_pads == candidate_pads, "footprint/pad geometry or net assignment changed"
    assert len(baseline_drc["unconnected_items"]) == 2
    assert open_nets(baseline_drc) == {"/RF_GPIO2", "/RF_MOSI"}
    assert len(candidate_drc["unconnected_items"]) == 1
    assert open_nets(candidate_drc) == {"/RF_MOSI"}
    assert candidate_drc["violations"] == []

    additions = candidate_copper - source_copper
    removals = source_copper - candidate_copper
    changed_nets = sorted({key[0] for key in additions} | {key[0] for key in removals})
    assert changed_nets == ["/HGM", "/RF_GPIO0", "/RF_GPIO2", "/RF_MISO", "/SENS_SCLK"]
    assert sum(additions.values()) == 144
    assert sum(removals.values()) == 34

    summary = {
        "kicad_version": candidate_drc.get("kicad_version"),
        "source_commit": SOURCE_COMMIT,
        "source_board_sha256": SOURCE_BOARD_SHA256,
        "source_prep_sha256": SOURCE_PREP_SHA256,
        "route_script_sha256": ROUTE_SCRIPT_SHA256,
        "route_replay_geometry_equal": True,
        "candidate_sha256": CANDIDATE_SHA256,
        "baseline_unconnected": len(baseline_drc["unconnected_items"]),
        "candidate_unconnected": len(candidate_drc["unconnected_items"]),
        "candidate_open_nets": sorted(open_nets(candidate_drc)),
        "candidate_violations": len(candidate_drc["violations"]),
        "footprint_pad_map_equal": True,
        "changed_nets": changed_nets,
        "added_copper_items": sum(additions.values()),
        "removed_copper_items": sum(removals.values()),
    }
    (REPORTS / "verification.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
