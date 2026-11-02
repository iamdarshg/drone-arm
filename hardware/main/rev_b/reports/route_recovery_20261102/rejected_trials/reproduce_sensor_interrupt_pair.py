#!/usr/bin/env python3
"""Replay the rejected paired /ICM_INT1 and /LSM_INT1 route trial."""
from __future__ import annotations

import hashlib
import json
import pathlib
import shutil
import subprocess
import sys

import pcbnew

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[5]
BOARD = ROOT / "hardware/main/rev_b/main_rev_b.kicad_pcb"
PROJECT = ROOT / "hardware/main/rev_b/main_rev_b.kicad_pro"
SOURCE_SHA256 = "60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191"


def mm(value: float) -> int:
    return pcbnew.FromMM(value)


def xy(point: tuple[float, float]) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(mm(point[0]), mm(point[1]))


def main() -> int:
    if hashlib.sha256(BOARD.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise SystemExit("pinned source board hash mismatch; refusing to replay")
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/sensor_interrupt_pair")
    out.mkdir(parents=True, exist_ok=True)
    candidate = out / "main_rev_b.kicad_pcb"
    shutil.copyfile(BOARD, candidate)
    shutil.copyfile(PROJECT, out / "main_rev_b.kicad_pro")
    board = pcbnew.LoadBoard(str(candidate))

    def add_via(net: str, point: tuple[float, float]) -> None:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(xy(point))
        via.SetDrill(mm(0.3))
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        for layer in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
            via.SetWidth(layer, mm(0.6))
        via.SetNetCode(board.FindNet(net).GetNetCode())
        board.Add(via)

    def add_segment(net: str, start: tuple[float, float], end: tuple[float, float], layer: int) -> None:
        track = pcbnew.PCB_TRACK(board)
        track.SetStart(xy(start))
        track.SetEnd(xy(end))
        track.SetLayer(layer)
        track.SetWidth(mm(0.2))
        track.SetNetCode(board.FindNet(net).GetNetCode())
        board.Add(track)

    routes = (
        ("/ICM_INT1", (54.05, 46.6), (53.3, 46.6), (47.5, 83.5), (48.8375, 83.75), pcbnew.In2_Cu),
        ("/LSM_INT1", (54.05, 47.4), (53.3, 47.4), (57.5, 83.5), (58.8375, 83.75), pcbnew.B_Cu),
    )
    for net, pad, source_via, target_via, remote_pad, layer in routes:
        add_via(net, source_via)
        add_via(net, target_via)
        add_segment(net, pad, source_via, pcbnew.F_Cu)
        add_segment(net, source_via, target_via, layer)
        add_segment(net, target_via, remote_pad, pcbnew.F_Cu)

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
    if (opens, violations) != (14, 37):
        raise SystemExit(f"unexpected DRC result: {opens} unconnected, {violations} violations")
    digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
    print(f"PASS rejected-route replay: board SHA-256 {digest}; {opens} unconnected / {violations} violations")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
