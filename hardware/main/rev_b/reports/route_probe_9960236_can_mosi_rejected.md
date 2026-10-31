# Rejected `/CAN_MOSI` route from `9960236`

A disposable `/CAN_MOSI` candidate connects U51.11 `(33.5,20.0)` to U10.36 `(61.2,54.95)` using 0.20 mm F.Cu escapes, 0.60/0.30 mm through-vias at `(32.4,20.0)` and `(61.2,56.2)`, and a straight 0.20 mm B.Cu span. It reduces the unconnected count by one but adds 13 DRC signatures, so it was rejected and not promoted.

- Source commit: `9960236fe75eabdf3ae81f02017b9c22dfc5ea91`
- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC: 7 violations / 45 unconnected; [`route_probe_bd9f896_before.json`](route_probe_bd9f896_before.json), SHA-256 `ff0f52414a4113eed39b8facc14b79fe9e2270677266f2fd228d14a4bcfb6f58`.
- Candidate board SHA-256: `27bc7c05de4503f880114c3e9706db781a95af7b2656c6843baa0fc8c3119224`.
- Candidate DRC: 20 violations / 44 unconnected; full JSON [`route_probe_9960236_can_mosi_bcu.json`](route_probe_9960236_can_mosi_bcu.json), SHA-256 `48037db99cef5df25f27562afd7f8592799e22f9aad2a82822a537daf756d404`.
- Exact baseline signature diff: [`route_probe_9960236_can_mosi_bcu_signature_diff.json`](route_probe_9960236_can_mosi_bcu_signature_diff.json), SHA-256 `179b66f5fec09b26d1d7e36dddae64453398c29f82a9b9b48ec68112a8823a6d`.

The new violations include shorts/crossings involving GND, 3V3, RF_RESET_N and other existing copper, plus via hole-clearance and mask-bridge issues. Source and candidate were zone-refilled with KiCad `pcbnew.ZONE_FILLER.Fill(board.Zones())` and checked using KiCad CLI 9.0.2 with the matching project. A post-save `pcbnew` read confirmed the route geometry and `/CAN_MOSI` net assignment. No schematic expectations were altered.
