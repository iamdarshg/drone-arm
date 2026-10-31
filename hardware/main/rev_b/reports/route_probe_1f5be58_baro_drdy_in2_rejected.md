# Rejected `/BARO_DRDY` In2 probe from `1f5be58`

After the accepted SYS_5V route, the `/BARO_DRDY` rat item is U10.9 `(54.05,49.4)` to the existing 0.0346 mm F.Cu stub beside U22.7 `(70.7625,82.75)`. I tested a new route on this updated source: 0.20 mm F.Cu escapes from U10.9 to a 0.60/0.30 mm through-via at `(52.8,49.4)` and from a second through-via at `(71.8,82.8)` to the existing same-net stub; a straight 0.20 mm In2.Cu span connects the vias.

## Source and paired DRC

- Source commit: `1f5be58ad0639f80b877378f2c2600df779b837b`
- Source board SHA-256: `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- KiCad pcbnew and CLI: 9.0.2. Both source and candidate were explicitly zone-refilled with `pcbnew.ZONE_FILLER.Fill(board.Zones())` before matching-project `kicad-cli pcb drc --severity-all --format json`.
- Zone-refilled source DRC: 7 violations / 44 unconnected; [`route_probe_1f5be58_before.json`](route_probe_1f5be58_before.json), SHA-256 `155e137e7f9c40bd3e03f3db4b631e42aefbd20034985e2f5eb473dcd0d26b1f`.
- Candidate board SHA-256: `f38863de2d1f71bcc6c200671831bdaa1881013678592dda646c0899331d8922`.
- Candidate DRC: 16 violations / 44 unconnected; [`route_probe_1f5be58_baro_drdy_in2.json`](route_probe_1f5be58_baro_drdy_in2.json), SHA-256 `e6ce917ea2782dd4ff67f537409233e8731459c0bfa3fc2282b5bd042f33ad95`.
- Exact baseline signature diff: [`route_probe_1f5be58_baro_drdy_in2_signature_diff.json`](route_probe_1f5be58_baro_drdy_in2_signature_diff.json), SHA-256 `53dd8f0f31015b341f3864092e7b0f1649dc95fed5be209786b52ae0efeb0324`. It has 9 new signatures and removes none.

This route does not improve the unconnected count and creates additional crossings, clearances and shorts involving `/1V1`; it is rejected. A post-save pcbnew read confirmed the segment/vias have the `/BARO_DRDY` net assignment, stated layers, widths and endpoints. Neither the board nor connectivity expectations were changed.
