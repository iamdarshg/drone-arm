#!/usr/bin/env python3
"""Replay the rejected diagnostic USB_D+/D- microvia breakout."""
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
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/usb_dp_microvia_pair")
    out.mkdir(parents=True, exist_ok=True)
    candidate = out / "main_rev_b.kicad_pcb"
    shutil.copyfile(BOARD, candidate)
    shutil.copyfile(PROJECT, out / "main_rev_b.kicad_pro")
    board = pcbnew.LoadBoard(str(candidate))

    def add_microvia(net: str, point: tuple[float, float]) -> None:
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(xy(point))
        via.SetDrill(mm(0.1))
        via.SetViaType(pcbnew.VIATYPE_MICROVIA)
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.In1_Cu)
        via.SetWidth(pcbnew.F_Cu, mm(0.3))
        via.SetWidth(pcbnew.In1_Cu, mm(0.3))
        via.SetNetCode(board.FindNet(net).GetNetCode())
        board.Add(via)

    dp = "/RP2354B MCU/USB_DP_MCU"
    dm = "/RP2354B MCU/USB_DM_MCU"
    add_microvia(dp, (60.4, 45.05))
    add_microvia(dm, (60.8, 45.05))
    add_microvia(dp, (56.53, 37.1))
    track = pcbnew.PCB_TRACK(board)
    track.SetStart(xy((60.4, 45.05)))
    track.SetEnd(xy((56.53, 37.1)))
    track.SetLayer(pcbnew.In1_Cu)
    track.SetWidth(mm(0.2))
    track.SetNetCode(board.FindNet(dp).GetNetCode())
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
    if (opens, violations) != (15, 15):
        raise SystemExit(f"unexpected DRC result: {opens} unconnected, {violations} violations")
    digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
    print(f"PASS rejected-route replay: board SHA-256 {digest}; {opens} unconnected / {violations} violations")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
