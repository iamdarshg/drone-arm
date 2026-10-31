# Rejected `/BARO_DRDY` straight F.Cu probe from `1fd8535`

On the post-SYS_5V source, I tested a 0.20 mm straight F.Cu segment from U10.9 `/BARO_DRDY` `(54.05,49.4)` to the endpoint `(70.787,82.7745)` of the existing 0.0346 mm same-net stub by U22.7. It was generated only on a temporary copy. The route adds shorts/crossings and increases the unrouted count, so it is rejected.

- Source commit: `1fd8535dd91a6477e4c40cce3eb24751060a6188`
- Source board SHA-256: `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- KiCad pcbnew and CLI: 9.0.2. Both source and candidate were explicitly zone-refilled with `pcbnew.ZONE_FILLER.Fill(board.Zones())` before matching-project `kicad-cli pcb drc --severity-all --format json`.
- Baseline: 7 violations / 44 unconnected; [`route_probe_1fd8535_before.json`](route_probe_1fd8535_before.json), SHA-256 `4ca040e7f5b5ecdaf1a8ae8538063e4bb309b05445918080133920b978034376`.
- Candidate board SHA-256: `1ae295c6f4d242df2135c3908fdafaa1329c13f944853a2b7342fe2137725585`.
- Candidate DRC: 20 violations / 46 unconnected; [`route_probe_1fd8535_baro_drdy_fcu.json`](route_probe_1fd8535_baro_drdy_fcu.json), SHA-256 `eb86ffa29fbcc75a48d81bf4b13a596ab17b65ee0e3e1b0ed70f942669240f65`.
- Exact baseline signature diff: [`route_probe_1fd8535_baro_drdy_fcu_signature_diff.json`](route_probe_1fd8535_baro_drdy_fcu_signature_diff.json), SHA-256 `72af659381b45cb097688e9787f1db60eda33bad4f8add9819994d35e16f5322`. It has 13 new signatures and removes none.

The new findings include shorts to `/1V1` and GND, crossings, and mask bridges. A post-save pcbnew read confirmed the route's net, layer, width and endpoints. It is rejected; the source board and connectivity expectations remain unchanged.
