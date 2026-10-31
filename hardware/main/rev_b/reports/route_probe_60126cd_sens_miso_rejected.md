# Rejected `/SENS_MISO` escape probe

Checked against source commit `60126cd023d9476ebe59372ba0dc196097e0e5e6` using KiCad 9.0.2. Source and candidate were explicitly zone-filled with pcbnew `ZONE_FILLER.Fill(board.Zones())`; CLI DRC ran with `--severity-all` and the matching project.

- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `2fbb3808080e265ff8e3e8cd7cd4015010e3bd59489fd92897a868762fcb106a`; 7 violations, 45 unconnected items.
- Candidate: U10.79 `/SENS_MISO` F.Cu breakout to a via at `(55.6,44.0)`, B.Cu direct to a via at the existing same-net F.Cu track endpoint `(58.8375,82.25)`.
- Candidate board SHA-256: `3fec2b6b485bda84fcc31eb66cbbe53e643492872a1bc2f195775454fe93fc04`
- Candidate DRC JSON SHA-256: `9a5ca902f5842c7f145fa3bf9be9fb574823200bcc51bf463686541532b46a7e`; 18 violations, 44 unconnected items.

Rejected due new shorts among `/SENS_MISO`, `/1V1`, `/3V3`, `/CAN_INT` and `/SENS_MOSI`, along with crossings and clearance errors. No candidate PCB was promoted.
