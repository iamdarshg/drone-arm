# Issue 23 native PCB variant audit

This report records an exploratory comparison of the source-pinned PCB variants in [pcb-variant-audit-sources.json](pcb-variant-audit-sources.json). It builds on the executable acceptance framework in [drone-arm PR #34](https://github.com/iamdarshg/drone-arm/pull/34) and the native CAD/KiCad evidence gates in [drone-mech PR #16](https://github.com/iamdarshg/drone-mech/pull/16). It does not select a board or grant fabrication authority. Measurements were made with KiCad 9.0.2 (`pcbnew` and `kicad-cli`) from detached checkouts at each manifest SHA. Reports and raw KiCad JSON were generated under `/tmp/issue23-variant-audit/` and are not release artifacts.

| Variant and exact source SHA | Main / ESC outline (mm) | Main / ESC footprints | Main / ESC connectors | Main DRC (violations / unconnected items) | ESC DRC (violations / unconnected items) | ERC main / ESC | Audit |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- |
| Current-main reference `744994a096ca73e872b396a9b0636c58e7c1afe8` | 140.15 × 100.15 / 173.95 × 283.08 | 165 / 687 | 6 / 31 | 7 / 70 | 24 / 499 | PASS / PASS | FAIL |
| PR10 `ceaec7a13e923be62d57398bd69c294f3565774f` | 140.15 × 100.15 / 474.15 × 246.15 | 165 / 654 | 6 / 21 | 158 / 499 | 207 / 499 | PASS / PASS | FAIL |
| PR11 `1d342cb6b450e3346712bbc2598c41e88b3c0a81` | 140.15 × 100.15 / 474.15 × 246.15 | 165 / 654 | 6 / 21 | 158 / 499 | 207 / 499 | PASS / PASS | FAIL |
| PR17 `569eda582b3a8c4221b308f09e0856dcdec2da21` | 140.15 × 100.15 / 173.95 × 283.08 | 165 / 687 | 6 / 31 | 7 / 70 | 24 / 499 | PASS / PASS | FAIL |

The audit found that PR17's PCB blobs produce the same measurements and native checks as the current-main reference. PR10 and PR11 likewise match each other. Every measured board outline differs from the 140 × 100 mm main and 150 × 150 mm ESC CAD placeholders by more than the configured 0.05 mm tolerance. Native solid component heights remain blocked because the exporter does not compute 3D solid bounds. The baseline and PR17 MCU pin map each expose six distinct PWM outputs for seven source-declared independent effectors; PR10 and PR11 do not include the pinned MCU pin map. No GPIO or missing output was invented.

The DRC counts are raw native findings, not adjusted acceptance targets. The CLI JSON reports 499 unconnected items on some boards; that is the count returned by the native report. No component 3D solid-height evidence is available from these audits. These results do not establish physical fit, assembly, electrical safety, first-article function, or fabrication readiness.
