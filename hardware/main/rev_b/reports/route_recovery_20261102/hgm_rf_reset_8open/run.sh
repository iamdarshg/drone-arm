#!/usr/bin/env bash
set -euo pipefail

repo="$(git rev-parse --show-toplevel)"
report_dir="$(cd "$(dirname "$0")" && pwd)"
export ROUTE_REPORT_DIR="$report_dir"
board_rel="hardware/main/rev_b/main_rev_b.kicad_pcb"
project_rel="hardware/main/rev_b/main_rev_b.kicad_pro"
source_commit="66fddff96ab557f686b391e49647ca5e8d33e11f"
source_sha="1961847383accbc57f6cbbccd21e6038e966d274d18683395b065b83488742e1"
project_sha="f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
candidate_sha="4e66a77b67e86817e5674b617e021ea9e4a40c889612c00d212bbcd9cd660d4e"
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

/usr/bin/python3 - "$tmp/source/drc.json" "$tmp/target/drc.json" "$tmp/replay/drc.json" <<'PY'
import json
import pathlib
import re
import sys
import os

expected = ((10, 0), (8, 0), (8, 0))
for path, counts in zip(map(pathlib.Path, sys.argv[1:]), expected):
    report = json.loads(path.read_text())
    actual = (len(report["unconnected_items"]), len(report["violations"]))
    print(f"{path.parent.name}: unconnected={actual[0]} violations={actual[1]}")
    if actual != counts:
        raise SystemExit(f"unexpected DRC result in {path}: {actual} != {counts}")

def normalized_violations(path):
    report = json.loads(pathlib.Path(path).read_text())
    return tuple(sorted(json.dumps(row, sort_keys=True) for row in report["violations"]))

def open_nets(path):
    report = json.loads(pathlib.Path(path).read_text())
    nets = []
    for row in report["unconnected_items"]:
        labels = {
            match.group(1)
            for item in row["items"]
            if (match := re.search(r"\[([^\]]+)\]", item.get("description", "")))
        }
        if len(labels) != 1:
            raise SystemExit(f"cannot identify one net for an open in {path}: {row}")
        nets.append(labels.pop())
    return tuple(sorted(nets))

archive = pathlib.Path(os.environ["ROUTE_REPORT_DIR"]) / "candidate_drc.json"
if open_nets(archive) != open_nets(sys.argv[2]):
    raise SystemExit("fresh DRC open-net set differs from archived candidate_drc.json")
if normalized_violations(archive) != normalized_violations(sys.argv[2]):
    raise SystemExit("fresh DRC violations differ from archived candidate_drc.json")
if open_nets(sys.argv[2]) != open_nets(sys.argv[3]):
    raise SystemExit("replayed DRC open-net set differs from candidate DRC")
if normalized_violations(sys.argv[2]) != normalized_violations(sys.argv[3]):
    raise SystemExit("replayed DRC violations differ from candidate DRC")
PY

echo "PASS: pinned 10-open source -> replayed 8-open candidate; pad map and copper geometry match."
