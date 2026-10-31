# Rejected `/GNSS_RX` long B.Cu route probe

Checked against source commit `9960236fe75eabdf3ae81f02017b9c22dfc5ea91` using KiCad 9.0.2. Baseline and candidate were explicitly zone-filled with pcbnew `ZONE_FILLER.Fill(board.Zones())` and run through matching-project CLI DRC with `--severity-all`.

- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `e62198a71066621791af8600058046af78a91e4dabd9f4d4e7952e3b9661f40a`; 7 violations, 45 unconnected items.
- Candidate: U10.12 `/GNSS_RX` F.Cu escape to via `(53.2,50.6)`, direct B.Cu to a via at existing same-net track endpoint `(126.55,13.1)`.
- Candidate board SHA-256: `30f251216aaca5d3a65d77bcb4c6c28ebc843d579eef093659742264693e2a9a`
- Candidate DRC JSON SHA-256: `6ade5fd9e65c53577a608e6d57e2b39d87837f7c195bc7bff75389bdfd0bdc1b`; 19 violations, 44 unconnected items.

Rejected due multiple new shorts to `/1V1`, plus shorts to `/GNSS/GNSS_UART_TX_2V8` and GND, a track crossing, clearance and hole-to-hole violations. No candidate PCB was promoted.
