#!/usr/bin/env python3
"""Verify ICM_INT1 route plus J1 clearance adjustment from pinned main."""
import hashlib
import json
import pathlib
import subprocess
import sys
import pcbnew

source, source_pro, routed, routed_path, candidate, candidate_pro, baseline_path, candidate_path, promoted, promoted_pro, comparator = map(pathlib.Path, sys.argv[1:12])

def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
assert sha(source) == "2d4288f022dc560135a40339b6b48d25b85f97292ff4f60da1157408bcabf051"
assert sha(source_pro) == "f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
assert sha(candidate_pro) == sha(source_pro) == sha(promoted_pro)
baseline = json.loads(baseline_path.read_text())
routed_drc = json.loads(routed_path.read_text())
final_drc = json.loads(candidate_path.read_text())
assert len(baseline["unconnected_items"]) == 11 and len(baseline["violations"]) == 4
assert len(routed_drc["unconnected_items"]) == 10 and len(routed_drc["violations"]) == 4
assert len(final_drc["unconnected_items"]) == 10 and len(final_drc["violations"]) == 0

def signature(v):
    items = sorted(((i.get("description"), i.get("pos")) for i in v.get("items", [])),
                   key=lambda x: json.dumps(x, sort_keys=True))
    return json.dumps((v.get("type"), v.get("severity"), v.get("description"), items), sort_keys=True)

assert sorted(map(signature, baseline["violations"])) == sorted(map(signature, routed_drc["violations"]))
assert not any("/ICM_INT1" in repr(item) for item in routed_drc["unconnected_items"])

def footprint_map(board):
    return {fp.GetReference(): (fp.GetValue(), fp.GetFPID().GetLibItemName(),
            round(pcbnew.ToMM(fp.GetPosition().x), 4), round(pcbnew.ToMM(fp.GetPosition().y), 4),
            round(fp.GetOrientationDegrees(), 4)) for fp in board.GetFootprints()}

def pad_map(board):
    return {(fp.GetReference(), p.GetNumber()): (p.GetNetname(),
            (round(pcbnew.ToMM(p.GetSize().x), 4), round(pcbnew.ToMM(p.GetSize().y), 4)),
            int(p.GetShape()), int(p.GetAttribute()))
            for fp in board.GetFootprints() for p in fp.Pads()}

a, b = pcbnew.LoadBoard(str(source)), pcbnew.LoadBoard(str(candidate))
af, bf = footprint_map(a), footprint_map(b)
assert len(af) == len(bf) == 165 and set(af) == set(bf)
for ref in af:
    if ref == "C11":
        assert af[ref][:2] == bf[ref][:2] and abs((bf[ref][3] - af[ref][3]) + 1.0) < 0.001
        assert af[ref][4] == bf[ref][4]
    else:
        assert af[ref] == bf[ref], ref
ap, bp = pad_map(a), pad_map(b)
assert len(ap) == len(bp) == 603 and set(ap) == set(bp)
assert {k for k in ap if ap[k] != bp[k]} == {("J1", n) for n in ("A1", "A12", "B1", "B12")}
assert all(ap[k][0] == bp[k][0] for k in ap)
subprocess.run([sys.executable, str(comparator), str(promoted), str(candidate)], check=True)
print("PASS: /ICM_INT1 closes 11->10 with exactly the four source DRC signatures")
print("PASS: J1 clearance correction clears DRC to zero; 10 opens remain")
print("PASS: only C11 moves 1.0 mm north and four J1 GND land lengths change")
print("PASS: promoted and replayed physical footprint/pad/copper geometry matches")
print("replay", sha(candidate), "DRC", sha(candidate_path))
