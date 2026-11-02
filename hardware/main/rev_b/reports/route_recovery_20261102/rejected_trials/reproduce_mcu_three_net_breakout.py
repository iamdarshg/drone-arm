#!/usr/bin/env python3
"""Replay the rejected three-net U10 breakout from the pinned 16-open board."""
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


def mm(x: float) -> int:
    return pcbnew.FromMM(x)


def xy(pos: tuple[float, float]) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(mm(pos[0]), mm(pos[1]))


def main() -> int:
    if hashlib.sha256(BOARD.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise SystemExit("pinned source board hash mismatch; refusing to replay")
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/mcu_three_net_breakout")
    out.mkdir(parents=True, exist_ok=True)
    candidate = out / "main_rev_b.kicad_pcb"
    project = out / "main_rev_b.kicad_pro"
    shutil.copyfile(BOARD, candidate)
    shutil.copyfile(PROJECT, project)
    board = pcbnew.LoadBoard(str(candidate))

    def via(net: str, pos: tuple[float, float]) -> None:
        item = pcbnew.PCB_VIA(board)
        item.SetPosition(xy(pos))
        item.SetDrill(mm(0.3))
        item.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        for layer in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
            item.SetWidth(layer, mm(0.6))
        item.SetNetCode(board.FindNet(net).GetNetCode())
        board.Add(item)

    def segment(net: str, start: tuple[float, float], end: tuple[float, float], layer: int) -> None:
        item = pcbnew.PCB_TRACK(board)
        item.SetStart(xy(start))
        item.SetEnd(xy(end))
        item.SetLayer(layer)
        item.SetWidth(mm(0.15))
        item.SetNetCode(board.FindNet(net).GetNetCode())
        board.Add(item)

    for net, pad, source_via, target_via in (
        ("/PA_EN", (56.8, 54.95), (56.0, 57.0), (109.5, 59.3625)),
        ("/LNA_EN", (57.2, 54.95), (57.2, 57.0), (111.175, 59.358)),
        ("/HGM", (57.6, 54.95), (58.4, 57.0), (112.0, 59.0)),
    ):
        via(net, source_via)
        via(net, target_via)
        segment(net, pad, source_via, pcbnew.F_Cu)
        segment(net, source_via, target_via, pcbnew.B_Cu)

    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(candidate), board)
    digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
    drc = out / "drc.json"
    subprocess.run(
        ["kicad-cli", "pcb", "drc", "--severity-all", "--format", "json", "-o", str(drc), str(candidate)],
        check=True,
        stdout=subprocess.DEVNULL,
    )
    result = json.loads(drc.read_text())
    opens, violations = len(result["unconnected_items"]), len(result["violations"])
    if opens != 13 or violations not in (47, 48):
        raise SystemExit(f"unexpected DRC result: {opens} unconnected, {violations} violations")
    print(f"PASS rejected-route replay: board SHA-256 {digest}; {opens} unconnected / {violations} DRC violations (archived run: 48)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
