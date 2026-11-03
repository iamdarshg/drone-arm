#!/usr/bin/env /usr/bin/python3
"""Replay the RF_MOSI/3V3 route and verify geometry, pads, and native DRC."""
from pathlib import Path
import collections, hashlib, json, subprocess
import pcbnew as pcb

ROOT = Path(__file__).resolve().parents[5]
R = Path(__file__).resolve().parent
BASE, SRC, CAND, REPLAY = R / "baseline.kicad_pcb", R / "source.kicad_pcb", R / "candidate.kicad_pcb", R / "replayed.kicad_pcb"
EXPECTED_BASE = "7923ad03efefebfba5251be189821cd3ed78e588c92cb7c62774082a207ed41d"
EXPECTED_SOURCE = "e73000b8edbb97769f9436f48b41fd33304c518db2bf9bf047b1120717e1664f"
EXPECTED_CAND = "b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43"

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

assert sha(BASE) == EXPECTED_BASE, "previous-main board hash mismatch"
assert sha(SRC) == EXPECTED_SOURCE, "source board hash mismatch"
assert sha(CAND) == EXPECTED_CAND, "candidate board hash mismatch"
subprocess.run(["/usr/bin/python3", str(R / "route_mosi.py")], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)

def signatures(path):
    board = pcb.LoadBoard(str(path))
    copper, pads = [], []
    for item in board.GetTracks():
        net = item.GetNetname()
        if isinstance(item, pcb.PCB_VIA):
            pos = item.GetPosition()
            layers = tuple(sorted(item.GetLayerSet().Seq()))
            copper.append(("via", net, pos.x, pos.y, layers, item.GetDrill(), tuple((layer, item.GetWidth(layer)) for layer in layers)))
        else:
            start = (item.GetStart().x, item.GetStart().y)
            end = (item.GetEnd().x, item.GetEnd().y)
            copper.append(("seg", net, item.GetLayer(), *sorted((start, end)), item.GetWidth()))
    for footprint in board.GetFootprints():
        for pad in footprint.Pads():
            pos = pad.GetPosition()
            pads.append((footprint.GetReference(), pad.GetNumber(), pad.GetNetname(), pos.x, pos.y, pad.GetSize().x, pad.GetSize().y))
    return collections.Counter(copper), collections.Counter(pads)

candidate_copper, candidate_pads = signatures(CAND)
baseline_copper, baseline_pads = signatures(BASE)
replay_copper, replay_pads = signatures(REPLAY) if REPLAY.exists() else (None, None)
if replay_copper is None:
    candidate_copper, candidate_pads = signatures(CAND)
    subprocess.run(["/usr/bin/python3", str(R / "route_mosi.py")], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
    replay_copper, replay_pads = signatures(REPLAY)
assert candidate_copper == replay_copper, "replay copper geometry differs from pinned candidate"
assert candidate_pads == replay_pads and sum(candidate_pads.values()) == 657, "pad map/location changed or incomplete"
assert candidate_pads == baseline_pads, "previous-main pad assignments or locations changed"

report = R / "candidate_recheck.drc.json"
subprocess.run(["kicad-cli", "pcb", "drc", "--exit-code-violations", "--severity-all", "--format", "json", "-o", str(report), str(CAND)], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
drc = json.loads(report.read_text())
assert not drc.get("unconnected_items") and not drc.get("violations"), f"DRC failure: {len(drc.get('unconnected_items', []))} opens, {len(drc.get('violations', []))} violations"
print(json.dumps({"source_sha256": sha(SRC), "candidate_sha256": sha(CAND), "replay_sha256": sha(REPLAY), "copper_items": sum(candidate_copper.values()), "pads": sum(candidate_pads.values()), "unconnected": 0, "violations": 0}, indent=2))
