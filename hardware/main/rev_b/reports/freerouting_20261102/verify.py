#!/usr/bin/env python3
"""Independently replay SES import and verify the rejected DRC result."""
import hashlib
import json
import pathlib
import shutil
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[5]
BOARD_REL = pathlib.Path("hardware/main/rev_b/main_rev_b.kicad_pcb")
PROJECT_REL = pathlib.Path("hardware/main/rev_b/main_rev_b.kicad_pro")
HERE = pathlib.Path(__file__).resolve().parent
BOARD_SHA = "60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191"
PROJECT_SHA = "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
SES_SHA = "f9ba660a4c6f1e9a91017830b3c529d4e0b98fc4ed004dcf6a2403dc01b3bde6"
DRC_SHA = "fee3e6427ca1b84e4a7ee1053539a5a144e2a41d190c00fce8938668ee7307ad"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def signature(v):
    items = tuple(sorted((i.get("description"), i.get("pos")) for i in v.get("items", [])))
    return json.dumps((v.get("type"), v.get("severity"), v.get("description"), items), sort_keys=True)


def drc(board, report):
    subprocess.run(
        ["kicad-cli", "pcb", "drc", "--severity-all", "--format", "json",
         "--output", str(report), str(board)],
        cwd=board.parent, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    return json.loads(report.read_text())


def main():
    source = ROOT / BOARD_REL
    project = ROOT / PROJECT_REL
    session = HERE / "output.ses"
    saved_report = HERE / "imported_drc.json"
    assert sha(source) == BOARD_SHA
    assert sha(project) == PROJECT_SHA
    assert sha(session) == SES_SHA
    assert sha(saved_report) == DRC_SHA
    with tempfile.TemporaryDirectory(prefix="freerouting-ses-verify-") as td:
        tmp = pathlib.Path(td)
        input_board = tmp / "input.kicad_pcb"
        input_board.write_bytes(source.read_bytes())
        output_board = tmp / "output.kicad_pcb"
        (tmp / "input.kicad_pro").write_bytes(project.read_bytes())
        (tmp / "output.kicad_pro").write_bytes(project.read_bytes())
        subprocess.run(
            ["/usr/bin/python3", str(HERE / "import_ses.py"), str(input_board),
             str(session), str(output_board)], check=True,
        )
        result = drc(output_board, tmp / "drc.json")
        archived = json.loads(saved_report.read_text())
        assert len(result["unconnected_items"]) == 16
        assert len(result["violations"]) == 12
        assert sorted(map(signature, result["violations"])) == sorted(
            map(signature, archived["violations"])
        )
        print("PASS: pinned SES import reproduces 16 unconnected / 12 DRC violations")
        print("PASS: imported result matches archived KiCad DRC violation signatures")


if __name__ == "__main__":
    main()
