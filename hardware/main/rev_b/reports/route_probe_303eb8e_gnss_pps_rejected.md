# Rejected `/GNSS_PPS` B.Cu route probe

This temporary probe attempted the low-speed GNSS PPS connection from U10 pad 13 to U30 pad 31. It was not promoted to the PCB.

Source board revision: `303eb8e`. Subsequent documentation commits leave the board bytes unchanged. Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`. Project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The candidate placed 0.60 mm / 0.30 mm through-vias at (53.000, 51.300) mm by U10 and (101.650, 16.800) mm by U30. It used 0.20 mm F.Cu stubs to U10.13 (54.050,51.000) and U30.31 (101.650,15.800), joined by a straight 59.64 mm B.Cu segment. Candidate board SHA-256: `1aff97e32e7885b8fe7d84595d11a72ad8626292705366f3b642504a9d91a9dc`.

KiCad CLI 9.0.2 with the matching project reported 13 violations and 44 unconnected items, versus the zone-refilled baseline of 7 violations and 45 unconnected items. The route removes one airwire but adds six DRC signatures: a clearance violation and short at the U10 breakout against `/3V3`, a B.Cu crossing with `/CAN_INT`, and three B.Cu shorts to existing GND vias near U30. Full candidate signatures are in `route_probe_303eb8e_gnss_pps_direct_bcu.json`.

- Zone-refilled baseline DRC SHA-256: `fd3b89d1e658148e616edf03a371c226593b088083691664ff5839d5f036a513` (`route_probe_zfill_303eb8e_before.json`).
- Candidate DRC SHA-256: `f429e390661e1b6f3373e7b3287ecb6f9ef0fc105ab48e5cc9fab43477f1eb8b` (`route_probe_303eb8e_gnss_pps_direct_bcu.json`).

The source PCB remains unchanged. The candidate is rejected under the no-new-DRC gate.
