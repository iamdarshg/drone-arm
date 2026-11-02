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
INTERMEDIATE_REV = "bccafdc25473696788f707b56de49654367c4451"
INTERMEDIATE16_REV = "7f9a9eff504ad5ebc7f33bd1af25ee743d9381c1"
SOURCE_SHA256 = "28ebb3c19ab3e8f7d3c7ee9acd5ab427a1868142399c27cabd758c7b14c2613f"
PROJECT_SHA256 = "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
CANDIDATE16_SHA256 = "60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191"
CANDIDATE_SHA256 = "f7deda79edbd6ac1a3a3eceba10fed34b283b704f1a61a1fa6a347e5fe153d15"
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
        candidate16_json = EVIDENCE / "candidate_drc_16_open.json"
        candidate_json = EVIDENCE / "candidate_drc.json"
        assert sha(source_json) == "cb141f87371a4ad58328edfff4f26418464e14ec52256191a2b1d6779ac0d18a"
        assert sha(candidate16_json) == "599987f963bb5e4ef78207ea8404ea081d902bc48fcc35f3795597bfdb196abd"
        assert sha(candidate_json) == "0e1f4a95f1fe5b15a557cd6a46b3b070b85af6e92939d500f100877796d22fe6"
        intermediate = EVIDENCE / "candidate_drc_17_open.json"
        assert sha(intermediate) == "09ab425900f233dbaadf6c4cd4d53b693759f559f8079841bdd8a24f46d6f603"
        assert len(json.loads(intermediate.read_text())["unconnected_items"]) == 17
        before = drc(baseline, baseline_dir / "source.json")
        after = drc(candidate, baseline_dir / "candidate.json")
        candidate16 = baseline_dir / BOARD_REL.name
        candidate16_data = subprocess.run(
            ["git", "-C", str(ROOT), "show",
             f"{INTERMEDIATE16_REV}:{BOARD_REL.as_posix()}"],
            check=True, stdout=subprocess.PIPE,
        ).stdout
        candidate16.write_bytes(candidate16_data)
        assert sha(candidate16) == CANDIDATE16_SHA256, "reconstructed 16-open board hash differs"
        (baseline_dir / PROJECT_REL.name).write_bytes(project.read_bytes())
        candidate16_drc = drc(candidate16, baseline_dir / "candidate16.json")
        archived16 = json.loads(candidate16_json.read_text())
        assert len(candidate16_drc["unconnected_items"]) == 16
        assert sorted(map(violation_signature, candidate16_drc["violations"])) == sorted(
            map(violation_signature, archived16["violations"])
        ), "archived 16-open report does not match the pinned board"
        intermediate_board = baseline_dir / "intermediate.kicad_pcb"
        intermediate_data = subprocess.run(
            ["git", "-C", str(ROOT), "show",
             f"{INTERMEDIATE_REV}:{BOARD_REL.as_posix()}"],
            check=True, stdout=subprocess.PIPE,
        ).stdout
        intermediate_board.write_bytes(intermediate_data)
        intermediate_project = baseline_dir / "intermediate.kicad_pro"
        intermediate_project.write_bytes(project.read_bytes())
        replay_board = baseline_dir / "replay.kicad_pcb"
        subprocess.run(
            ["/usr/bin/python3", str(EVIDENCE / "reproduce_long_3v3.py"),
             str(intermediate_board), str(replay_board)],
            check=True,
        )
        (baseline_dir / "replay.kicad_pro").write_bytes(intermediate_project.read_bytes())
        replay = drc(replay_board, baseline_dir / "replay.json")
        replay15_board = baseline_dir / "replay15.kicad_pcb"
        subprocess.run(
            ["/usr/bin/python3", str(EVIDENCE / "reproduce_c11_shift_1v1.py"),
             str(candidate16), str(replay15_board)],
            check=True,
        )
        (baseline_dir / "replay15.kicad_pro").write_bytes(project.read_bytes())
        replay15 = drc(replay15_board, baseline_dir / "replay15.json")
        subprocess.run(
            ["/usr/bin/python3", str(EVIDENCE / "compare_board_geometry.py"),
             str(candidate), str(replay15_board)], check=True,
        )
        assert len(before["unconnected_items"]) == 42
        assert len(candidate16_drc["unconnected_items"]) == 16
        assert len(after["unconnected_items"]) == 15
        assert len(replay["unconnected_items"]) == 16
        assert len(replay15["unconnected_items"]) == 15
        assert len(before["violations"]) == len(candidate16_drc["violations"]) == len(after["violations"]) == len(replay15["violations"]) == 4
        assert sorted(map(violation_signature, before["violations"])) == sorted(
            map(violation_signature, candidate16_drc["violations"])
        ), "16-open candidate changed the source violation signatures"
        assert sorted(map(violation_signature, candidate16_drc["violations"])) == sorted(
            map(violation_signature, after["violations"])
        ), "15-open candidate changed the four baseline violation signatures"
        assert sorted(map(violation_signature, replay15["violations"])) == sorted(
            map(violation_signature, after["violations"])
        ), "reproduced /1V1 route DRC does not match promoted candidate"
        assert sorted(map(violation_signature, replay["violations"])) == sorted(
            map(violation_signature, after["violations"])
        ), "reproduced route does not match promoted candidate DRC"
        print("PASS: source 42 / 4; historical candidate 16 / 4; promoted candidate 15 / 4")
        print("PASS: 17-open /3V3 route replay produces the historical 16-open board")
        print("PASS: /1V1 In2 replay matches promoted board geometry, 165 footprints and all pad mappings")
        print("PASS: pinned board, project, and archived DRC evidence hashes match")
        print("PASS: all four normalized violation signatures are unchanged")


if __name__ == "__main__":
    main()
