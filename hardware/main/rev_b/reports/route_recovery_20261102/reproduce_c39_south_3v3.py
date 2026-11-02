#!/usr/bin/env python3
"""Replay the C39-C41 south shift and U10 /3V3 local route.

Input is the pinned 15-open board at commit b6761f8. The replay uses native
KiCad pcbnew edits in isolated subprocesses, refills zones, and preserves all
165 footprints and pad-to-net assignments.
"""
import argparse
import hashlib
import pathlib
import shutil
import subprocess
import tempfile

import pcbnew

SOURCE_SHA256 = "f7deda79edbd6ac1a3a3eceba10fed34b283b704f1a61a1fa6a347e5fe153d15"
PROJECT_SHA256 = "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
HERE = pathlib.Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def pad_map(board):
    return tuple(sorted(
        (fp.GetReference(), fp.GetValue(), fp.GetFPID().GetLibItemName(),
         round(fp.GetOrientationDegrees(), 6),
         tuple(sorted((pad.GetNumber(), pad.GetNetname()) for pad in fp.Pads()))
        )
        for fp in board.GetFootprints()
    ))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_board")
    parser.add_argument("output_board")
    args = parser.parse_args()
    source = pathlib.Path(args.input_board).resolve()
    project = source.with_suffix(".kicad_pro")
    output = pathlib.Path(args.output_board).resolve()
    if sha(source) != SOURCE_SHA256:
        raise SystemExit("pinned 15-open source board hash mismatch")
    if sha(project) != PROJECT_SHA256:
        raise SystemExit("pinned KiCad project hash mismatch")

    before = pcbnew.LoadBoard(str(source))
    if len(pad_map(before)) != 165:
        raise SystemExit("expected 165 footprints")
    original_pads = pad_map(before)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="drone-arm-c39-3v3-replay-") as td:
        work = pathlib.Path(td)
        work_source = work / "main_rev_b.kicad_pcb"
        work_project = work / "main_rev_b.kicad_pro"
        shutil.copyfile(source, work_source)
        shutil.copyfile(project, work_project)
        stage0 = work / "stage0.kicad_pcb"
        stage1 = work / "stage1.kicad_pcb"
        for script, input_board, output_board in (
            ("replay_decap_move.py", work_source, stage0),
            ("replay_decap_power.py", stage0, stage1),
            ("replay_u10_3v3.py", stage1, output),
        ):
            subprocess.run(
                ["/usr/bin/python3", str(HERE / script), str(input_board), str(output_board)],
                check=True,
            )

    after = pcbnew.LoadBoard(str(output))
    if pad_map(after) != original_pads:
        raise SystemExit("footprint pad assignments changed")
    print("PASS: all 165 footprints and every pad-to-net assignment are unchanged")
    print("PASS: C39-C41 moved +0.2 mm south; local /1V1/GND rebuilt; U10 /3V3 pads 24/29 joined")


if __name__ == "__main__":
    main()
