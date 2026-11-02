#!/usr/bin/env bash
set -euo pipefail
repo="$(git rev-parse --show-toplevel)"
report_dir="$repo/hardware/main/rev_b/reports/route_recovery_20261102"
case_dir="$report_dir/replay_12to11"
rev=5630b1c
expected_source_sha=a568c7faef6fee3a9a45423d4b7e9653e37e862d0f4db0cada9e3a72651a7a5d
expected_project_sha=f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e
promoted_board="${PROMOTED_BOARD:-$repo/hardware/main/rev_b/main_rev_b.kicad_pcb}"
promoted_pro="${PROMOTED_PRO:-$repo/hardware/main/rev_b/main_rev_b.kicad_pro}"
out="$(mktemp -d /tmp/drone-arm-route-12to11.XXXXXX)"
trap 'rm -rf "$out"' EXIT
git -C "$repo" show "$rev:hardware/main/rev_b/main_rev_b.kicad_pcb" > "$out/source.kicad_pcb"
git -C "$repo" show "$rev:hardware/main/rev_b/main_rev_b.kicad_pro" > "$out/source.kicad_pro"
echo "$expected_source_sha  $out/source.kicad_pcb" | sha256sum --check --status
echo "$expected_project_sha  $out/source.kicad_pro" | sha256sum --check --status
/usr/bin/python3 "$case_dir/build_cc_lna_p.py" "$out/source.kicad_pcb" "$out/raw.kicad_pcb"
cp "$out/source.kicad_pro" "$out/raw.kicad_pro"
/usr/bin/python3 "$case_dir/refill_zones.py" "$out/raw.kicad_pcb" "$out/candidate.kicad_pcb"
cp "$out/source.kicad_pro" "$out/candidate.kicad_pro"
kicad-cli pcb drc --severity-all --format json -o "$out/baseline_drc.json" "$out/source.kicad_pcb"
kicad-cli pcb drc --severity-all --format json -o "$out/candidate_drc.json" "$out/candidate.kicad_pcb"
/usr/bin/python3 "$case_dir/verify.py" \
  "$out/source.kicad_pcb" "$out/source.kicad_pro" \
  "$out/candidate.kicad_pcb" "$out/candidate.kicad_pro" \
  "$out/baseline_drc.json" "$out/candidate_drc.json" \
  "$promoted_board" "$promoted_pro" "$report_dir/compare_board_geometry.py"
