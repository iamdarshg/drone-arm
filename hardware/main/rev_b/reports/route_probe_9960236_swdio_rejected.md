# Rejected `/RP2354B MCU/SWDIO` routes from `9960236`

Both disposable candidates route only the schematic-confirmed SWDIO net between debug connector J10.2 `(59.0,30.0)` and MCU U10.34 `(60.4,54.95)`. Both reduce the ratsnest by one, but add violations against the pinned full-board baseline, so neither was promoted.

## Pinned inputs and method

- Source commit: `9960236fe75eabdf3ae81f02017b9c22dfc5ea91`
- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- KiCad CLI: 9.0.2; each board was zone-refilled through `pcbnew.ZONE_FILLER.Fill(board.Zones())` and checked with `kicad-cli pcb drc --severity-all --format json` using the matching same-stem project.
- Baseline: 7 DRC violations, 45 unconnected items. Baseline JSON [`route_probe_bd9f896_before.json`](route_probe_bd9f896_before.json), SHA-256 `ff0f52414a4113eed39b8facc14b79fe9e2270677266f2fd228d14a4bcfb6f58`.

| Candidate | Candidate board SHA-256 | Full candidate DRC JSON SHA-256 | DRC / unconnected | Outcome |
|---|---|---|---:|---|
| 0.20 mm straight F.Cu from J10.2 to U10.34 | `d7827170e3f1022486e3de153a2fbcea448eb0b95064bf5418b60392fc84a7eb` | `9573f24eecfd3243b026a06d35c33546cc06f969e27c9fb87d0027ef8d38978f` | 15 / 44 | Rejected: 8 new signatures, including clearance/crossing/short/mask-bridge interactions with 3V3 and GND at MCU escape. Complete JSON: [`route_probe_9960236_swdio_fcu.json`](route_probe_9960236_swdio_fcu.json). Exact baseline signature diff: [`route_probe_9960236_swdio_fcu_signature_diff.json`](route_probe_9960236_swdio_fcu_signature_diff.json), SHA-256 `451673fab5a9a869f54f84e589c471a23a6f47df8c1072db055d250a074e2a02`.
| 0.20 mm F.Cu escapes to 0.60/0.30 mm through-vias at `(59,31.2)` and `(60.4,56.2)`, then a straight 0.20 mm B.Cu span | `020d08f97219424623747458eac338e0778e442291bcc985ceda73127ab6e684` | `9699f8c165c21f17d2814f1f052208672d8906af788374a74b61100ad65c613c` | 16 / 44 | Rejected: 9 new signatures, including shorts with CAN_INT, GND and SENS_SCLK, track crossing, and via hole-clearance violations. Complete JSON: [`route_probe_9960236_swdio_bcu.json`](route_probe_9960236_swdio_bcu.json). Exact baseline signature diff: [`route_probe_9960236_swdio_bcu_signature_diff.json`](route_probe_9960236_swdio_bcu_signature_diff.json), SHA-256 `7accca5f5b91b9a5c8334a74f2f33c7f5ff4a12e8a9b34d66b439251891d32c9`.

## Independent geometry check

KiCad `pcbnew` reopened each saved, refilled candidate and confirmed the added route segments were assigned to `/RP2354B MCU/SWDIO`, connected the stated pad coordinates, and used the declared F.Cu/B.Cu layers and widths. The paired project DRC remained the acceptance gate; both candidates fail it. No board or schematic expectations were changed.
