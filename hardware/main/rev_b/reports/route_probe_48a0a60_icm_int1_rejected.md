# Rejected `/ICM_INT1` via-route probe

Checked against source commit `48a0a601eed09b6ec6b6ffd6d1f40f00db7797b8` using KiCad 9.0.2. Baseline and candidate were explicitly zone-filled with pcbnew `ZONE_FILLER.Fill(board.Zones())` and checked by CLI DRC `--severity-all` using matching project settings.

- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `379794193a9bbe51e3ad1a9414abfaba53eb9ba19a2833b70a5d674593a4bfe5`; 7 violations, 45 unconnected items.
- Candidate: U10.2 `/ICM_INT1` F.Cu escape to a via at `(53.1,46.6)`, B.Cu direct to a via on the existing same-net track endpoint `(48.7628,83.6753)`.
- Candidate board SHA-256: `efcdce25d0c86fe5d1dab4b900f128d2163fc8f4d7e0ba95ad804b37643d3151`
- Candidate DRC JSON SHA-256: `7156e69bf0c58712dd3894e72eedb15ee961c6f28e2e7a7ed5773564fe33b34a`; 18 violations, 44 unconnected items.

Rejected due new crossings, clearances/shorts involving GND and `/BARO_SCL`, and a hole-clearance violation. No candidate PCB was promoted.
