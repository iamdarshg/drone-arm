#!/usr/bin/env bash
set -euo pipefail

repo="$(git rev-parse --show-toplevel)"
report_dir="$(cd "$(dirname "$0")" && pwd)"
board_rel="hardware/main/rev_b/main_rev_b.kicad_pcb"
project_rel="hardware/main/rev_b/main_rev_b.kicad_pro"
source_commit="6550f35bca6c005773b10eb444e6b270b8690a24"
source_sha="b107359cf9adf6bb08e9ecd75ea2e0e8e4bddc3036be4e1fd8c3261f36924656"
project_sha="f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e"
candidate_sha="fb2fdc622895e726b569e55fe97b2abd1d14d2c7e81d8e46601e996b89cbd3c3"
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
import sys

expected = ((4, 0), (3, 0), (3, 0))
for path, counts in zip(map(pathlib.Path, sys.argv[1:4]), expected):
    report = json.loads(path.read_text())
    actual = (len(report["unconnected_items"]), len(report["violations"]))
    print(f"{path.parent.name}: unconnected={actual[0]} violations={actual[1]}")
    if actual != counts:
        raise SystemExit(f"unexpected DRC result in {path}: {actual} != {counts}")

archive = json.loads(pathlib.Path(sys.argv[4]).read_text())
def violation_signatures(report):
    return tuple(sorted(json.dumps(row, sort_keys=True) for row in report["violations"]))
if violation_signatures(archive) != violation_signatures(json.loads(pathlib.Path(sys.argv[2]).read_text())):
    raise SystemExit("fresh DRC violations differ from archived candidate report")
PY

echo "PASS: pinned 4-open source -> replayed 3-open candidate; /PA_EN closed, zero DRC violations, all pad and copper geometry matches."
