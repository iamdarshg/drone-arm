# Rejected `/CAN_SCK` In2.Cu route probe

Checked against source commit `775761659892fa4251d5aafc4b39adb27f93c59f` with KiCad 9.0.2. Both boards were explicitly zone-filled with pcbnew `ZONE_FILLER.Fill(board.Zones())`, then checked with matching-project CLI DRC `--severity-all`.

- Source board SHA-256: `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `f7a80a7f69ae1bf282f4e34208a815f174c5521998de42f6f3a83efbdfdab74a`; 7 violations, 44 unconnected.
- Candidate: U51.10 `/CAN_SCK` `(33.5,20.65)` via `(34.4,20.65)` on F.Cu, In2.Cu direct to via `(58,56)`, then F.Cu to U10.28 `(58,54.95)`.
- Candidate board SHA-256: `9d7cd6ea288cbdd2b410eec2cf5c28cb00231ab27c4257d5b16e5bb2d3d70010`
- Candidate DRC JSON SHA-256: `f5cfd128b54522cb6fc87999b233a0c3045ef279be55b2072ba234b6bd501bc8`; 18 violations, 44 unconnected.

Rejected: no unrouted reduction, with new crossings and shorts/clearance errors to `/1V1`, `/3V3`, GND and `/BARO_SCL`. No candidate PCB was promoted.
