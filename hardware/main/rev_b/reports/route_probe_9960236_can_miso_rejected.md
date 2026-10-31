# Rejected `/CAN_MISO` route from `9960236`

A disposable `/CAN_MISO` candidate connects U51.12 `(33.5,19.35)` to U10.37 `(61.6,54.95)` using 0.20 mm F.Cu escapes, 0.60/0.30 mm through-vias at `(32.4,19.35)` and `(61.6,56.2)`, and a straight 0.20 mm B.Cu span. It reduces the unconnected count by one but adds 14 DRC signatures, so it was rejected and not promoted.

- Source commit: `9960236fe75eabdf3ae81f02017b9c22dfc5ea91`
- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC: 7 violations / 45 unconnected; [`route_probe_bd9f896_before.json`](route_probe_bd9f896_before.json), SHA-256 `ff0f52414a4113eed39b8facc14b79fe9e2270677266f2fd228d14a4bcfb6f58`.
- Candidate board SHA-256: `d5de4b69d2e6a167e7b6e5fae38b5ea84e90fee24e141c9f978ca52cf0966603`.
- Candidate DRC: 21 violations / 44 unconnected; full JSON [`route_probe_9960236_can_miso_bcu.json`](route_probe_9960236_can_miso_bcu.json), SHA-256 `6c04972680aed40a392953d94d01a3a768e3327b0f6d0d62e28399961d79ff85`.
- Exact baseline signature diff: [`route_probe_9960236_can_miso_bcu_signature_diff.json`](route_probe_9960236_can_miso_bcu_signature_diff.json), SHA-256 `3739fced0af569d534661bdcdbe3b1d4a4101fd829f599cdab8101615ed97076`.

The full-project DRC identifies clearance violations and shorts/crossings with 3V3, CAN_INT, and GND, plus hole-clearance and solder-mask violations. Both source and candidate were zone-refilled with KiCad `pcbnew.ZONE_FILLER.Fill(board.Zones())` and checked using KiCad CLI 9.0.2 with the matching project. A post-save `pcbnew` read confirmed the track segments, layers, widths, vias, endpoints, and `/CAN_MISO` net assignment. No design expectations were modified.
