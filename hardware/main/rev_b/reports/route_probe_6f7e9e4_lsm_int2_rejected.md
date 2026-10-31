# Rejected `/LSM_INT2` direct route probe

Checked against source commit `6f7e9e4882155d7ffb4afbdcda73cbdbe6b7f619` using KiCad 9.0.2. Baseline and candidate were explicitly zone-filled with pcbnew `ZONE_FILLER.Fill(board.Zones())` and run through CLI DRC `--severity-all` with matching project settings.

- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `b64c37b021142361f7556974bb2b2474a12c8a999c2c5a4a2d495973f3366024`; 7 violations, 45 unconnected items.
- Candidate: 0.20 mm F.Cu direct route U10.6 `(54.05,48.2)` to U21.9 `(61.1625,83.25)` on `/LSM_INT2`.
- Candidate board SHA-256: `d420928339f4c2fedd24df235db76590d56f80393e9e695730d32b55d3da1d3a`
- Candidate DRC JSON SHA-256: `8b941b60b74a8d95991d82152f0f1ddd0398765221d4931c0191e426554e14e2`; 27 violations, 44 unconnected items.

Rejected: new crossings and shorts to `/BARO_SCL`, `/LSM_nCS`, and `/SENS_SCLK`, plus clearance and solder-mask violations. No candidate PCB was promoted.
