# Rejected `/GNSS_TX` B.Cu route probe

Checked against source commit `65116e1bf3972cffce22fdfde567c2ad711b19e1` using KiCad 9.0.2. Baseline and candidate were explicitly zone-filled using pcbnew `ZONE_FILLER.Fill(board.Zones())`, then run with matching-project KiCad CLI DRC `--severity-all`.

- Source board SHA-256: `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Refill script SHA-256: `221f0995b7154e7f5e88c69c4b6e01002477a72f4e5873f11e50067866baf8a6`
- Baseline DRC JSON SHA-256: `4891d4e9f5e4f82d1b7e1ab37db8626baa57b1fa191b2fa02891eae72378f2d7`; 7 violations, 44 unconnected items.
- Candidate: U10.11 `/GNSS_TX` `(54.05,50.2)` escaped on F.Cu to a via at `(52.2,50.2)`, B.Cu direct to a via at `(86.0,7.0)` and R30.1 `(86.0,6.1)`.
- Candidate board SHA-256: `7b6ef3356a62c7e48e41c9691bbd76379dde0ba3c0b8fc50e5ac6f339b3a51b2`
- Candidate DRC JSON SHA-256: `f7b2a64ab586f679784d54aceb2f124c269e356c01bed8936743649b077cb6f1`; 18 violations, 43 unconnected items.

Rejected due new crossings, shorts to `/1V1`, `/VREG_AVDD`, `/BARO_SCL` and GND, and two clearance violations. No candidate PCB was promoted.
