#!/usr/bin/env python3
"""Replay the rejected RF_50R-width /CC1121_RF via route."""
from __future__ import annotations

import hashlib
import json
import pathlib
import shutil
import subprocess
import sys

import pcbnew
from pinned_source import candidate16_board

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[5]
BOARD = candidate16_board()
PROJECT = ROOT / "hardware/main/rev_b/main_rev_b.kicad_pro"
SOURCE_SHA256 = "60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191"


def mm(value: float) -> int:
    return pcbnew.FromMM(value)


def xy(point: tuple[float, float]) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(mm(point[0]), mm(point[1]))


def main() -> int:
    if hashlib.sha256(BOARD.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise SystemExit("pinned source board hash mismatch; refusing to replay")
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/cc1121_rf_35_bcu")
    out.mkdir(parents=True, exist_ok=True)
    candidate = out / "main_rev_b.kicad_pcb"
    shutil.copyfile(BOARD, candidate)
    shutil.copyfile(PROJECT, out / "main_rev_b.kicad_pro")
    board = pcbnew.LoadBoard(str(candidate))
    net = board.FindNet("/915 MHz RF/CC1121_RF").GetNetCode()
    source, remote = (101.375, 61.75), (103.8, 62.25)
    via_points = ((100.0, 61.75), (104.8, 62.25))

    for point in via_points:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(xy(point))
        via.SetDrill(mm(0.3))
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        for layer in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
            via.SetWidth(layer, mm(0.6))
        via.SetNetCode(net)
        board.Add(via)

    for start, end, layer in (
        (source, via_points[0], pcbnew.F_Cu),
        (via_points[0], via_points[1], pcbnew.B_Cu),
        (via_points[1], remote, pcbnew.F_Cu),
    ):
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(xy(start))
        track.SetEnd(xy(end))
        track.SetLayer(layer)
        track.SetWidth(mm(0.35))
        track.SetNetCode(net)
        board.Add(track)

    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(candidate), board)
    report = out / "drc.json"
    subprocess.run(
        ["kicad-cli", "pcb", "drc", "--severity-all", "--format", "json", "-o", str(report), str(candidate)],
        check=True,
        stdout=subprocess.DEVNULL,
    )
    result = json.loads(report.read_text())
    opens, violations = len(result["unconnected_items"]), len(result["violations"])
    if (opens, violations) != (15, 8):
        raise SystemExit(f"unexpected DRC result: {opens} unconnected, {violations} violations")
    digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
    print(f"PASS rejected-route replay: board SHA-256 {digest}; {opens} unconnected / {violations} violations")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
