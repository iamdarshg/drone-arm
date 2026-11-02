#!/usr/bin/env python3
"""Reproduce and verify the pinned PCB routing recovery with KiCad CLI."""
import hashlib
import json
import pathlib
import shutil
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[5]
BOARD_REL = pathlib.Path("hardware/main/rev_b/main_rev_b.kicad_pcb")
PROJECT_REL = pathlib.Path("hardware/main/rev_b/main_rev_b.kicad_pro")
SOURCE_REV = "fc80ded19dd070b476866e84de6c872ee3480d89"
SOURCE_SHA256 = "28ebb3c19ab3e8f7d3c7ee9acd5ab427a1868142399c27cabd758c7b14c2613f"
PROJECT_SHA256 = "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
CANDIDATE_SHA256 = "49b03f98c942f625f46f670fc420dda754b739f362e6fb5d65ce8c36f3a3f88f"
EVIDENCE = pathlib.Path(__file__).resolve().parent


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def drc(board, output):
    subprocess.run(
        ["kicad-cli", "pcb", "drc", "--severity-all", "--format", "json",
         "--output", str(output), str(board)],
        cwd=board.parent, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True,
    )
    return json.loads(output.read_text())


def violation_signature(v):
    items = tuple(sorted((i.get("description"), i.get("pos")) for i in v.get("items", [])))
    return json.dumps((v.get("type"), v.get("severity"), v.get("description"), items), sort_keys=True)


def main():
    candidate = ROOT / BOARD_REL
    project = ROOT / PROJECT_REL
    assert sha(project) == PROJECT_SHA256, "pinned project hash differs"
    assert sha(candidate) == CANDIDATE_SHA256, "candidate board hash differs"
    with tempfile.TemporaryDirectory(prefix="drone-arm-route-verify-") as td:
        baseline_dir = pathlib.Path(td)
        baseline = baseline_dir / BOARD_REL.name
        data = subprocess.run(
            ["git", "-C", str(ROOT), "show", f"{SOURCE_REV}:{BOARD_REL.as_posix()}"],
            check=True, stdout=subprocess.PIPE,
        ).stdout
        baseline.write_bytes(data)
        (baseline_dir / PROJECT_REL.name).write_bytes(project.read_bytes())
        assert sha(baseline) == SOURCE_SHA256, "reconstructed source hash differs"
        source_json = EVIDENCE / "source_drc.json"
        candidate_json = EVIDENCE / "candidate_drc.json"
        assert sha(source_json) == "cb141f87371a4ad58328edfff4f26418464e14ec52256191a2b1d6779ac0d18a"
        assert sha(candidate_json) == "09ab425900f233dbaadf6c4cd4d53b693759f559f8079841bdd8a24f46d6f603"
        before = drc(baseline, baseline_dir / "source.json")
        after = drc(candidate, baseline_dir / "candidate.json")
        assert len(before["unconnected_items"]) == 42
        assert len(after["unconnected_items"]) == 17
        assert len(before["violations"]) == len(after["violations"]) == 4
        assert sorted(map(violation_signature, before["violations"])) == sorted(
            map(violation_signature, after["violations"])
        ), "candidate changed the source violation signatures"
        print("PASS: source 42 unconnected / 4 violations; candidate 17 / 4")
        print("PASS: pinned board, project, and archived DRC evidence hashes match")
        print("PASS: all four normalized violation signatures are unchanged")


if __name__ == "__main__":
    main()
