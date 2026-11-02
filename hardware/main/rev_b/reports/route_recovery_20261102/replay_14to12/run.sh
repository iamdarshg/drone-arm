#!/usr/bin/env bash
set -euo pipefail
repo="$(git rev-parse --show-toplevel)"
report_dir="$repo/hardware/main/rev_b/reports/route_recovery_20261102"
case_dir="$report_dir/replay_14to12"
rev=762d0ad72744380748cac158e1de42ea7305ae42
out="$(mktemp -d /tmp/drone-arm-route-14to12.XXXXXX)"
promoted_board="${PROMOTED_BOARD:-$repo/hardware/main/rev_b/main_rev_b.kicad_pcb}"
promoted_pro="${PROMOTED_PRO:-$repo/hardware/main/rev_b/main_rev_b.kicad_pro}"
git -C "$repo" show "$rev:hardware/main/rev_b/main_rev_b.kicad_pcb" > "$out/source.kicad_pcb"
git -C "$repo" show "$rev:hardware/main/rev_b/main_rev_b.kicad_pro" > "$out/source.kicad_pro"
/usr/bin/python3 "$case_dir/build_cc_rbias.py" "$out/source.kicad_pcb" "$out/raw_bias.kicad_pcb"
cp "$out/source.kicad_pro" "$out/raw_bias.kicad_pro"
/usr/bin/python3 "$case_dir/refill_zones.py" "$out/raw_bias.kicad_pcb" "$out/bias.kicad_pcb"
cp "$out/source.kicad_pro" "$out/bias.kicad_pro"
/usr/bin/python3 "$case_dir/rotate_fl40.py" "$out/bias.kicad_pcb" "$out/rotated.kicad_pcb"
/usr/bin/python3 "$case_dir/rebuild_rf_local.py" "$out/rotated.kicad_pcb" "$out/rf_local.kicad_pcb"
/usr/bin/python3 "$case_dir/leave_lna_p_open.py" "$out/rf_local.kicad_pcb" "$out/candidate.kicad_pcb"
cp "$out/source.kicad_pro" "$out/candidate.kicad_pro"
kicad-cli pcb drc --severity-all --format json -o "$out/baseline_drc.json" "$out/source.kicad_pcb"
kicad-cli pcb drc --severity-all --format json -o "$out/candidate_drc.json" "$out/candidate.kicad_pcb"
/usr/bin/python3 "$case_dir/verify.py" \
  "$out/source.kicad_pcb" "$out/source.kicad_pro" \
  "$out/candidate.kicad_pcb" "$out/candidate.kicad_pro" \
  "$out/baseline_drc.json" "$out/candidate_drc.json" \
  "$promoted_board" \
  "$promoted_pro" \
  "$report_dir/compare_board_geometry.py"
