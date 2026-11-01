# GNSS courtyard annotation moved off silkscreen

The current source board had two F.Silkscreen courtyard boundary lines that ran to the right and bottom board edges, plus the `GNSS 50R / 3mm COURTYARD` label overlapping the horizontal line. KiCad 9.0.2 reported two silkscreen-edge violations and one silkscreen overlap.

The candidate moves only these three UUID-identified annotation objects from F.Silkscreen to F.Fab. The label is also moved from `(103.0, 36.0)` to `(103.0, 34.5)` mm to separate it visibly from the fab line. No copper, vias, footprint placement, net assignment, board outline, or project rules changed. An independent top-level object comparison found exactly those three annotation changes. The F.Fab render confirms the annotation remains readable and leaves the physical silkscreen.

## Pinned inputs and result

- Source commit: `481274bad6fda4f1fcfd991c06b134bb563cc651` (`drone-arm/main` before this change).
- Source board SHA-256: `095aa3fd44285b885c2a4ca68c05633b169dd053fbc8407d2160e8585e73336b`.
- Candidate board SHA-256: `28ebb3c19ab3e8f7d3c7ee9acd5ab427a1868142399c27cabd758c7b14c2613f`.
- Project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.
- Tool: KiCad/pcbnew 9.0.2; zones were explicitly refilled before each DRC.
- Paired `kicad-cli pcb drc --severity-all --format json` result: 7 violations → 4, with 3 silk signatures cleared and zero new signatures. Unconnected items remain 42, with the per-net multiset unchanged.
- Reproduce the evidence consistency check with `/usr/bin/python3 hardware/main/rev_b/reports/gnss_courtyard_cleanup/verify_evidence.py`.

The four remaining violations are the USB-C NPTH-to-GND pad clearances at 0.1944 mm against the project’s 0.2000 mm requirement. All 42 open connections remain. This is a silkscreen cleanup only and does not make the board fabrication-ready.
