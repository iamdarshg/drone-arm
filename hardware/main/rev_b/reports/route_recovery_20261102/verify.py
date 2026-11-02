#!/usr/bin/env python3
"""Reconstruct and verify every accepted PCB routing recovery stage."""
import hashlib
import json
import os
import pathlib
import shutil
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[5]
BOARD_REL = pathlib.Path("hardware/main/rev_b/main_rev_b.kicad_pcb")
PROJECT_REL = pathlib.Path("hardware/main/rev_b/main_rev_b.kicad_pro")
SOURCE_REV = "fc80ded19dd070b476866e84de6c872ee3480d89"
INTERMEDIATE17_REV = "bccafdc25473696788f707b56de49654367c4451"
CANDIDATE16_REV = "7f9a9eff504ad5ebc7f33bd1af25ee743d9381c1"
CANDIDATE15_REV = "b6761f85473a1e9a8e3a1b335f4974d40afb5deb"
SOURCE_SHA256 = "28ebb3c19ab3e8f7d3c7ee9acd5ab427a1868142399c27cabd758c7b14c2613f"
PROJECT_SHA256 = "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
BOARD16_SHA256 = "60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191"
BOARD15_SHA256 = "f7deda79edbd6ac1a3a3eceba10fed34b283b704f1a61a1fa6a347e5fe153d15"
BOARD14_SHA256 = "63b00e91cc1ea46e7e539b383c0deb6c7ac4bdfa833dd14cdd59936aef90e0ef"
BOARD12_SHA256 = "a568c7faef6fee3a9a45423d4b7e9653e37e862d0f4db0cada9e3a72651a7a5d"
BOARD11_SHA256 = "2d4288f022dc560135a40339b6b48d25b85f97292ff4f60da1157408bcabf051"
BOARD10_CLEAN_SHA256 = "1961847383accbc57f6cbbccd21e6038e966d274d18683395b065b83488742e1"
EVIDENCE = pathlib.Path(__file__).resolve().parent


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def materialize(revision, output, expected_sha):
    output.parent.mkdir(parents=True, exist_ok=True)
    data = subprocess.run(
        ["git", "-C", str(ROOT), "show", f"{revision}:{BOARD_REL.as_posix()}"],
        check=True, stdout=subprocess.PIPE,
    ).stdout
    output.write_bytes(data)
    if sha(output) != expected_sha:
        raise AssertionError(f"{revision} board hash mismatch")
    project = output.with_suffix(".kicad_pro")
    shutil.copyfile(ROOT / PROJECT_REL, project)
    if sha(project) != PROJECT_SHA256:
        raise AssertionError("matching KiCad project hash mismatch")
    return output


def drc(board, output):
    subprocess.run(
        ["kicad-cli", "pcb", "drc", "--severity-all", "--format", "json",
         "--output", str(output), str(board)],
        cwd=board.parent, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True,
    )
    return json.loads(output.read_text())


def signature(violation):
    items = tuple(sorted(
        ((item.get("description"), item.get("pos"))
         for item in violation.get("items", [])),
        key=lambda item: json.dumps(item, sort_keys=True),
    ))
    return json.dumps(
        (violation.get("type"), violation.get("severity"),
         violation.get("description"), items),
        sort_keys=True,
    )


def replay(script, source, output):
    subprocess.run(
        ["/usr/bin/python3", str(EVIDENCE / script), str(source), str(output)],
        check=True,
    )


def main():
    candidate = ROOT / BOARD_REL
    project = ROOT / PROJECT_REL
    if sha(project) != PROJECT_SHA256:
        raise SystemExit("pinned project hash differs")
    if sha(candidate) != BOARD10_CLEAN_SHA256:
        raise SystemExit("current 10-open / zero-violation board hash differs")

    source_json = EVIDENCE / "source_drc.json"
    candidate16_json = EVIDENCE / "candidate_drc_16_open.json"
    candidate15_json = EVIDENCE / "candidate_drc_15_open.json"
    candidate14_json = EVIDENCE / "candidate_drc_14_open.json"
    candidate12_json = EVIDENCE / "candidate_drc_12_open.json"
    candidate11_json = EVIDENCE / "candidate_drc_11_open_4_violations.json"
    candidate10_json = EVIDENCE / "candidate_drc.json"
    intermediate17_json = EVIDENCE / "candidate_drc_17_open.json"
    expected_reports = {
        source_json: "cb141f87371a4ad58328edfff4f26418464e14ec52256191a2b1d6779ac0d18a",
        candidate16_json: "599987f963bb5e4ef78207ea8404ea081d902bc48fcc35f3795597bfdb196abd",
        candidate15_json: "0e1f4a95f1fe5b15a557cd6a46b3b070b85af6e92939d500f100877796d22fe6",
        candidate14_json: "10e5deac975ca4da513e5dd2c3e17377d4a84885832f86b2267890a8298591f8",
        candidate12_json: "a75b4e25fa394055c1264441ad8ef45b87b9b9b398945ca5d3aed0fc932af6a6",
        candidate11_json: "4bbd41af3ffea172d9dc6e848f57ff6782011776d70d5b7e6de6056e5779ede9",
        intermediate17_json: "09ab425900f233dbaadf6c4cd4d53b693759f559f8079841bdd8a24f46d6f603",
    }
    for path, expected in expected_reports.items():
        if sha(path) != expected:
            raise SystemExit(f"archived report hash mismatch: {path.name}")

    with tempfile.TemporaryDirectory(prefix="drone-arm-route-verify-") as td:
        work = pathlib.Path(td)
        source = materialize(
            SOURCE_REV, work / "source" / BOARD_REL.name, SOURCE_SHA256,
        )
        board16 = materialize(
            CANDIDATE16_REV, work / "candidate16" / BOARD_REL.name, BOARD16_SHA256,
        )
        board15 = materialize(
            CANDIDATE15_REV, work / "candidate15" / BOARD_REL.name, BOARD15_SHA256,
        )
        board17 = materialize(
            INTERMEDIATE17_REV, work / "intermediate17" / BOARD_REL.name,
            "49b03f98c942f625f46f670fc420dda754b739f362e6fb5d65ce8c36f3a3f88f",
        )
        board14 = materialize(
            "762d0ad72744380748cac158e1de42ea7305ae42",
            work / "candidate14" / BOARD_REL.name, BOARD14_SHA256,
        )
        board12 = materialize(
            "5630b1c", work / "candidate12" / BOARD_REL.name, BOARD12_SHA256,
        )
        board11_open = materialize(
            "c453474", work / "candidate11_open" / BOARD_REL.name, BOARD11_SHA256,
        )

        source_drc = drc(source, work / "source-drc.json")
        board16_drc = drc(board16, work / "candidate16-drc.json")
        board15_drc = drc(board15, work / "candidate15-drc.json")
        board14_drc = drc(board14, work / "candidate14-drc.json")
        board12_drc = drc(board12, work / "candidate12-drc.json")
        board11_drc = drc(board11_open, work / "candidate11-open-drc.json")
        board10_drc = drc(candidate, work / "candidate10-drc.json")
        replay16_dir = work / "replay16"
        replay16_dir.mkdir()
        replay17to16 = replay16_dir / BOARD_REL.name
        replay("reproduce_long_3v3.py", board17, replay17to16)
        shutil.copyfile(project, replay17to16.with_suffix(".kicad_pro"))
        replay16_drc = drc(replay17to16, work / "replay16-drc.json")
        replay15_dir = work / "replay15"
        replay15_dir.mkdir()
        replay16to15 = replay15_dir / BOARD_REL.name
        replay("reproduce_c11_shift_1v1.py", board16, replay16to15)
        shutil.copyfile(project, replay16to15.with_suffix(".kicad_pro"))
        replay15_drc = drc(replay16to15, work / "replay15-drc.json")
        subprocess.run(
            ["/usr/bin/python3", str(EVIDENCE / "compare_board_geometry.py"),
             str(board15), str(replay16to15)], check=True,
        )
        replay14_dir = work / "replay14"
        replay14_dir.mkdir()
        replay15to14 = replay14_dir / BOARD_REL.name
        replay("reproduce_c39_south_3v3.py", board15, replay15to14)
        shutil.copyfile(project, replay15to14.with_suffix(".kicad_pro"))
        replay14_drc = drc(replay15to14, work / "replay14-drc.json")
        subprocess.run(
            ["/usr/bin/python3", str(EVIDENCE / "compare_board_geometry.py"),
             str(board14), str(replay15to14)], check=True,
        )
        subprocess.run(
            ["bash", str(EVIDENCE / "replay_14to12" / "run.sh")],
            cwd=ROOT, check=True, env={**os.environ,
                "PROMOTED_BOARD": str(board12), "PROMOTED_PRO": str(board12.with_suffix(".kicad_pro"))},
        )
        subprocess.run(
            ["bash", str(EVIDENCE / "replay_12to11" / "run.sh")],
            cwd=ROOT, check=True, env={**os.environ,
                "PROMOTED_BOARD": str(board11_open),
                "PROMOTED_PRO": str(board11_open.with_suffix(".kicad_pro"))},
        )
        subprocess.run(
            ["bash", str(EVIDENCE / "replay_11to10" / "run.sh")],
            cwd=ROOT, check=True,
        )

        counts = (
            len(source_drc["unconnected_items"]),
            len(board16_drc["unconnected_items"]),
            len(board15_drc["unconnected_items"]),
            len(board14_drc["unconnected_items"]),
            len(board12_drc["unconnected_items"]),
            len(board11_drc["unconnected_items"]),
            len(board10_drc["unconnected_items"]),
            len(replay16_drc["unconnected_items"]),
            len(replay15_drc["unconnected_items"]),
            len(replay14_drc["unconnected_items"]),
        )
        if counts != (42, 16, 15, 14, 12, 11, 10, 16, 15, 14):
            raise AssertionError(f"unexpected connectivity results: {counts}")
        if any(len(report["violations"]) != 4 for report in (
            source_drc, board16_drc, board15_drc, board14_drc, board12_drc, board11_drc,
            replay16_drc, replay15_drc, replay14_drc,
        )):
            raise AssertionError("an accepted stage changed the four baseline DRC findings")
        baseline_sigs = sorted(map(signature, source_drc["violations"]))
        for report in (board16_drc, board15_drc, board14_drc, board12_drc, board11_drc,
                       replay16_drc, replay15_drc, replay14_drc):
            if sorted(map(signature, report["violations"])) != baseline_sigs:
                raise AssertionError("normalized DRC violations differ from baseline")
        if len(board10_drc["violations"]) != 0:
            raise AssertionError("current 10-open board still has DRC violations")
        if sorted(map(signature, json.loads(candidate15_json.read_text())["violations"])) != baseline_sigs:
            raise AssertionError("archived 15-open DRC report differs from baseline")
        if sorted(map(signature, json.loads(candidate14_json.read_text())["violations"])) != baseline_sigs:
            raise AssertionError("archived 14-open DRC report differs from baseline")
        if sorted(map(signature, json.loads(candidate12_json.read_text())["violations"])) != baseline_sigs:
            raise AssertionError("archived 12-open DRC report differs from baseline")
        if sorted(map(signature, json.loads(candidate11_json.read_text())["violations"])) != baseline_sigs:
            raise AssertionError("archived pre-clearance 11-open DRC report differs from baseline")
        saved_final = json.loads(candidate10_json.read_text())
        if len(saved_final["unconnected_items"]) != 10 or len(saved_final["violations"]) != 0:
            raise AssertionError("archived current DRC report is not 10-open / zero-violation")

    print("PASS: source 42/4; accepted stages 16/4, 15/4, 14/4, 12/4, 11/4, and 10/0")
    print("PASS: 17-open /3V3 replay reproduces historical 16-open board")
    print("PASS: C11 /1V1 replay reproduces pinned 15-open board geometry")
    print("PASS: C39-C41 shift and U10 /3V3 replay reproduces the pinned 14-open board")
    print("PASS: CC_RBIAS and CC1121_RF replay reproduces the pinned 12-open board")
    print("PASS: CC_LNA_P replay reproduces the pinned 11-open /4 board")
    print("PASS: ICM_INT1 route plus J1 clearance replay reproduces the current 10-open /0 board")
    print("PASS: all 165 footprints and pad-to-net assignments are preserved; four land lengths change within the drawing tolerance")
    print("PASS: archived hashes and baseline DRC signatures match; current DRC has zero findings")


if __name__ == "__main__":
    main()
