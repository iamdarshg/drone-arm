# Rejected U10 GND bridge from `f73d222`

A disposable 0.20 mm F.Cu bridge from U10.62 `/GND` `(62.4,45.05)` to the existing same-net via `(60.48,43.5)` reduces unconnected count by one. It is rejected because it shorts `/GND` to `/3V3` and `/VREG_LX` and crosses existing copper.

- Source commit: `f73d22259b932dbd28af640d17c08b85c9716d4c`
- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC: 7 violations / 45 unconnected; [`route_probe_bd9f896_before.json`](route_probe_bd9f896_before.json), SHA-256 `ff0f52414a4113eed39b8facc14b79fe9e2270677266f2fd228d14a4bcfb6f58`.
- Candidate board SHA-256: `37b202e99a6226d9957d59f86abe38ba49f34ddcaab9aed9559e7f17975f8c85`.
- Candidate DRC: 12 violations / 44 unconnected; full JSON [`route_probe_f73d222_u10_gnd.json`](route_probe_f73d222_u10_gnd.json), SHA-256 `ee02417f0657812652a85a95dad9dda03f6b32aff2d6b38ff3128323b60cb3b6`.
- Exact baseline signature diff: [`route_probe_f73d222_u10_gnd_signature_diff.json`](route_probe_f73d222_u10_gnd_signature_diff.json), SHA-256 `e2e042066dba767a3320c99565d6920c8d98a8306fa65ac929aa542a122a1b26`.

Baseline and candidate used KiCad `pcbnew.ZONE_FILLER.Fill(board.Zones())` and KiCad CLI 9.0.2 matching-project DRC (`--severity-all`). A post-save `pcbnew` read verified that the candidate segment is on `/GND`, F.Cu, width 0.20 mm, with the stated endpoints. No PCB change was promoted.
