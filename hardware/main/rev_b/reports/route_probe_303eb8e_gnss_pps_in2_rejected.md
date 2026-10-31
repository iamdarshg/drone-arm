# Rejected `/GNSS_PPS` In2 route probe

This second temporary `/GNSS_PPS` probe changed the U10 breakout and moved the long segment from B.Cu to In2.Cu. It was not promoted.

Source board revision: `303eb8e`; later commits are documentation-only. Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`. Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The candidate placed through-vias at (51.400, 51.300) mm by U10 and (101.650, 16.800) mm by U30. It connected U10.13 (54.050,51.000) to the first via with a 0.20 mm F.Cu trace, connected the vias with a 0.20 mm In2.Cu trace, and connected the destination via to U30.31 (101.650,15.800) with a 0.20 mm F.Cu trace. Candidate board SHA-256: `870bcc5eef91ed334ca88ba7b0fc9302ab5390ee66a4f1ea84d4e9fcddddb27a`.

KiCad CLI 9.0.2 with the matching project reported 21 violations and 45 unconnected items, versus the zone-refilled baseline of 7 violations and 45 unconnected items. The candidate adds fourteen DRC signatures: seven shorts, two clearances, two solder-mask bridges, two crossings, and one hole-clearance violation. Issues include shorts to `/1V1`, `/3V3`, `/3V3_ADC`, and GND; crossings with `/BARO_SCL` and `/3V3`; and contact with the `/3V3_ADC` filter pad. The unconnected count did not improve. Full candidate signatures are in `route_probe_303eb8e_gnss_pps_in2_candidate.json`.

- Zone-refilled baseline DRC SHA-256: `fd3b89d1e658148e616edf03a371c226593b088083691664ff5839d5f036a513` (`route_probe_zfill_303eb8e_before.json`).
- Candidate DRC SHA-256: `fc99a97390ca5028f55eac7f19bf9456a61550e3e86e488e788cd9762f3dec34` (`route_probe_303eb8e_gnss_pps_in2_candidate.json`).

The source PCB remains unchanged. The candidate is rejected.
