#!/usr/bin/env bash
set -euo pipefail

repo="$(git rev-parse --show-toplevel)"
report_dir="$(cd "$(dirname "$0")" && pwd)"
export ROUTE_REPORT_DIR="$report_dir"
board_rel="hardware/main/rev_b/main_rev_b.kicad_pcb"
project_rel="hardware/main/rev_b/main_rev_b.kicad_pro"
source_commit="899adceeb5b6afe9004e0ebb5df4771a073d8e5c"
source_sha="2356919db26ce9e01417d78d849c0de13541a3827e69a674549cf2245a0b72a3"
project_sha="f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
candidate_sha="b107359cf9adf6bb08e9ecd75ea2e0e8e4bddc3036be4e1fd8c3261f36924656"
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
import sys
import os

expected = ((5, 0), (4, 0), (4, 0))
for path, counts in zip(map(pathlib.Path, sys.argv[1:]), expected):
    report = json.loads(path.read_text())
    actual = (len(report["unconnected_items"]), len(report["violations"]))
    print(f"{path.parent.name}: unconnected={actual[0]} violations={actual[1]}")
    if actual != counts:
        raise SystemExit(f"unexpected DRC result in {path}: {actual} != {counts}")

def normalized(path):
    report = json.loads(pathlib.Path(path).read_text())
    return tuple(sorted(json.dumps(row, sort_keys=True) for row in report[key])
                 for key in ("unconnected_items", "violations"))

archive = pathlib.Path(os.environ["ROUTE_REPORT_DIR"]) / "candidate_drc.json"
if normalized(archive) != normalized(sys.argv[2]):
    raise SystemExit("fresh DRC findings differ from archived candidate_drc.json")
if normalized(sys.argv[2]) != normalized(sys.argv[3]):
    raise SystemExit("replayed DRC findings differ from candidate DRC")
PY

echo "PASS: pinned 5-open source -> replayed 4-open candidate; pad map and copper geometry match."
