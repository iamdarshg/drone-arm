# Issue 23 native PCB variant audit

This report records an exploratory comparison of the source-pinned PCB variants in [pcb-variant-audit-sources.json](pcb-variant-audit-sources.json). It builds on the executable acceptance framework in [drone-arm PR #34](https://github.com/iamdarshg/drone-arm/pull/34) and the native CAD/KiCad evidence gates in [drone-mech PR #16](https://github.com/iamdarshg/drone-mech/pull/16). It does not select a board or grant fabrication authority. Measurements were made with KiCad 9.0.2 (`pcbnew` and `kicad-cli`) from detached checkouts at each manifest SHA. Reports and raw KiCad JSON were generated under `/tmp/issue23-variant-audit/` and are not release artifacts.

| Variant and exact source SHA | Main / ESC outline (mm) | Main / ESC footprints | Main / ESC connectors | Main DRC (violations / unconnected items) | ESC DRC (violations / unconnected items) | ERC main / ESC (severity; CLI exit) | Audit |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- |
| Current-main reference `744994a096ca73e872b396a9b0636c58e7c1afe8` | 140 × 100 / 173.8 × 282.93 | 165 / 687 | 6 / 31 | 7 / 70 | 24 / 499 | 162 warning / 603 warning; 5 / 5 | FAIL |
| PR10 `ceaec7a13e923be62d57398bd69c294f3565774f` | 140 × 100 / 474 × 246 | 165 / 654 | 6 / 21 | 158 / 499 | 207 / 499 | 155 warning / 576 warning; 5 / 5 | FAIL |
| PR11 `1d342cb6b450e3346712bbc2598c41e88b3c0a81` | 140 × 100 / 474 × 246 | 165 / 654 | 6 / 21 | 158 / 499 | 207 / 499 | 155 warning / (154 error + 84 warning); 5 / 5 | FAIL |
| PR17 `569eda582b3a8c4221b308f09e0856dcdec2da21` | 140 × 100 / 173.8 × 282.93 | 165 / 687 | 6 / 31 | 7 / 70 | 24 / 499 | 162 warning / 603 warning; 5 / 5 | FAIL |

The outline envelope is calculated from Edge.Cuts centerline geometry: segment endpoints, swept arc extrema, and circle radii. Graphic stroke bounds are reported separately and include line width: 140.15 × 100.15 mm for the main board; 173.95 × 283.08 mm for baseline/PR17 ESC; 474.15 × 246.15 mm for PR10/PR11 ESC. Main board outlines measure exactly 140 × 100 mm in all variants, matching the CAD placeholder. The ESC baseline/PR17 outline is 173.8 × 282.93 mm; PR10/PR11 is 474 × 246 mm. Each differs from the 150 × 150 mm ESC CAD placeholder by more than the configured 0.05 mm tolerance. The centerline envelopes agree for all eight boards with the native PCB adapter used by PR34. Open or branched Edge.Cuts paths are rejected rather than reported as valid outline envelopes.

PR17's PCB blobs produce the same measurements and native findings as the current-main reference. PR10 and PR11 produce the same geometry and DRC results, with different ERC results. KiCad returned exit code 5 for all native DRC and ERC runs. The audit parses ERC violations from the per-sheet `sheets[*].violations` arrays. Native solid component heights remain blocked because the exporter does not compute 3D solid bounds. The baseline and PR17 MCU pin map each expose six distinct PWM outputs for seven source-declared independent effectors; PR10 and PR11 do not include the pinned MCU pin map. No GPIO or missing output was invented.

The DRC and ERC counts are raw native findings, not adjusted acceptance targets. Baseline/PR17 main DRC includes 4 errors and 3 warnings; ESC DRC includes 8 errors and 16 warnings. PR10/PR11 main DRC includes 4 errors and 154 warnings; their ESC DRC includes 207 warnings. ERC results are all warnings except PR11 ESC, which includes 154 errors and 84 warnings. The CLI JSON reports 499 unconnected items on some boards; that is the count returned by the native report. The source Edge.Cuts graphic stroke envelope is retained as a separate field in the machine-readable results. No component 3D solid-height evidence is available from these audits. These results do not establish physical fit, assembly, electrical safety, first-article function, or fabrication readiness.

## Legacy KiCad launcher baseline probe

The fresh baseline rerun used a clean worktree at `744994a096ca73e872b396a9b0636c58e7c1afe8`. Running `python -m pytest -q` there recorded **23 passed, 2 failed** because both unchanged Windows-targeted tests tried to launch `C:\Program Files\KiCad\9.0\bin\python.exe`, which is unavailable in this Linux runner. Its log is `/workspace/legacy-original-baseline.log`. The earlier `/workspace/issue2-motor-drive-screen/highspeed_edf/verification/evidence/baseline/arm-pytest.log` came from an issue21-framework checkout; the two launcher test files are byte-identical to the baseline744 files, but that earlier log is not attributed to this baseline run. The fresh baseline worktree has no tracked changes; the portable probe below created only an untracked `.superpowers/` directory there. No test assertions or source files were changed.

To check the existing tests with the installed KiCad Python without editing them, an external pytest collection hook at `/workspace/legacy_probe.py` replaced only each collected module's `KICAD_PYTHON` value:

```python
from pathlib import Path

def pytest_collection_modifyitems(items):
    for item in items:
        if hasattr(item.module, "KICAD_PYTHON"):
            item.module.KICAD_PYTHON = Path("/usr/bin/python3")
```

Reproduce from that clean worktree with:

```sh
cd /workspace/legacy-baseline
PYTHONPATH=/workspace pytest -p legacy_probe -q \
  tests/test_prepare_esc_quilter_upload.py \
  tests/test_reconcile_esc_usb_footprints.py
```

The portable-runtime probe recorded **1 passed, 1 failed** (`/workspace/legacy-portable-probe.log`); it left the untracked `.superpowers/` artifacts noted above. The reconciliation test now reaches `pcbnew`, then fails because its configured source PCB `hardware/esc/rev_b/reports/esc_165mm_hybrid_newtop_oldbottom_tight.kicad_pcb` is absent. `git log --all --` for that exact path returns no tracked history. The absent PCB was not created or substituted; the original one-pass/one-missing-file result is retained as a baseline finding.
