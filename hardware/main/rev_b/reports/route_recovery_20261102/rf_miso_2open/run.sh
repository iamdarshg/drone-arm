#!/usr/bin/env bash
set -euo pipefail

repo="$(git rev-parse --show-toplevel)"
report_dir="$(cd "$(dirname "$0")" && pwd)"
board_rel="hardware/main/rev_b/main_rev_b.kicad_pcb"
project_rel="hardware/main/rev_b/main_rev_b.kicad_pro"
source_commit="01e83dab330b11c4914380c022e3ec2d92ed492b"
source_sha="fb2fdc622895e726b569e55fe97b2abd1d14d2c7e81d8e46601e996b89cbd3c3"
project_sha="f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
candidate_sha="fde09ade461c58ad4b3cc1952dca15e961e417dea0e5875055b503114e83be3a"
candidate="${TARGET_BOARD:-$repo/$board_rel}"
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

for stage in source target replay; do
  mkdir -p "$tmp/$stage"
  cp "$repo/$project_rel" "$tmp/$stage/main_rev_b.kicad_pro"
done
test "$(sha256sum "$repo/$project_rel" | awk '{print $1}')" = "$project_sha"
git -C "$repo" show "$source_commit:$board_rel" > "$tmp/source/main_rev_b.kicad_pcb"
test "$(sha256sum "$tmp/source/main_rev_b.kicad_pcb" | awk '{print $1}')" = "$source_sha"
cp "$candidate" "$tmp/target/main_rev_b.kicad_pcb"
test "$(sha256sum "$tmp/target/main_rev_b.kicad_pcb" | awk '{print $1}')" = "$candidate_sha"

/usr/bin/python3 "$report_dir/replay_geometry.py" \
  "$tmp/source/main_rev_b.kicad_pcb" \
  "$tmp/target/main_rev_b.kicad_pcb" \
  "$tmp/replay/main_rev_b.kicad_pcb"
/usr/bin/python3 "$report_dir/compare_geometry.py" \
  "$tmp/target/main_rev_b.kicad_pcb" \
  "$tmp/replay/main_rev_b.kicad_pcb"

for stage in source target replay; do
  kicad-cli pcb drc --severity-all --format json \
    -o "$tmp/$stage/drc.json" "$tmp/$stage/main_rev_b.kicad_pcb"
done

/usr/bin/python3 - "$tmp/source/drc.json" "$tmp/target/drc.json" "$tmp/replay/drc.json" "$report_dir/candidate_drc.json" <<'PY'
import json
import pathlib
import re
import sys

expected = ((3, 0), (2, 0), (2, 0))
for path, counts in zip(map(pathlib.Path, sys.argv[1:4]), expected):
    report = json.loads(path.read_text())
    actual = (len(report["unconnected_items"]), len(report["violations"]))
    print(f"{path.parent.name}: unconnected={actual[0]} violations={actual[1]}")
    if actual != counts:
        raise SystemExit(f"unexpected DRC result in {path}: {actual} != {counts}")

def open_nets(report):
    nets = []
    for row in report["unconnected_items"]:
        labels = {
            match.group(1)
            for item in row["items"]
            if (match := re.search(r"\[([^\]]+)\]", item.get("description", "")))
        }
        if len(labels) != 1:
            raise SystemExit(f"cannot identify one net for an open: {row}")
        nets.append(labels.pop())
    return tuple(sorted(nets))

def violation_signatures(report):
    return tuple(sorted(json.dumps(row, sort_keys=True) for row in report["violations"]))

reports = [json.loads(pathlib.Path(path).read_text()) for path in sys.argv[1:4]]
archive = json.loads(pathlib.Path(sys.argv[4]).read_text())
if open_nets(archive) != open_nets(reports[1]):
    raise SystemExit("fresh DRC open-net set differs from archived candidate report")
if open_nets(reports[1]) != open_nets(reports[2]):
    raise SystemExit("replayed DRC open-net set differs from candidate report")
if violation_signatures(archive) != violation_signatures(reports[1]) or violation_signatures(reports[1]) != violation_signatures(reports[2]):
    raise SystemExit("fresh DRC violations differ from archived candidate or replay")
PY

echo "PASS: pinned 3-open source -> replayed 2-open candidate; /RF_MISO closed, zero DRC violations, pad and copper geometry match."
