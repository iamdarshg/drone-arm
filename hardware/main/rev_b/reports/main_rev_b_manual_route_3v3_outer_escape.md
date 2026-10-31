# Rev-B hand-route: outer `/3V3` MCU island connection

Added a short three-segment F.Cu connection between two existing `/3V3` copper islands above U10. The path connects the existing track endpoint at (59.520, 43.500) mm to the existing `/3V3` track at (61.600, 44.300) mm, routing around the nearby GND via at (60.480, 43.500) mm. Width is 0.20 mm. The board net assignment confirms both copper islands and all new segments are `/3V3`.

Source revision: `3bc0dff` (`docs: pin rejected GNSS RX route probe`). Source board SHA-256: `db8fa1ca44907ad5b70b7010bdb67fe06d80dcd6715aea2a02662682ddb47291`.

Native KiCad CLI 9.0.2 DRC was run with the matching `main_rev_b.kicad_pro` and refilled zones on both source and candidate boards:

- Source: 7 violations, 55 unconnected items.
- Candidate: 7 violations, 54 unconnected items.
- The seven violation signatures, including item descriptions and coordinates, are identical before and after.

The unconnected `/3V3` track pair at (59.600, 45.050) mm and (61.600, 44.300) mm is no longer reported after connection; other U10 `/3V3` branches remain unrouted. This local route does not make the board fabrication-ready.

Candidate board SHA-256: `b5ed4d4bd2c50b7d58569a7f7ede820350f8c8303ed4ffd1e1a52966bd65e022`.

DRC report SHA-256 values:

- Before: `be13794147edf3fdb2b4db2bf23a14be3671a21661eb3e5fdfb164e833cc67e7` (`main_rev_b_manual_route_3v3_outer_escape_before.json`).
- After: `996f3dc514c7707e0d02d5a86c2744035d3899090b5e62ac92ee3398a1154106` (`main_rev_b_manual_route_3v3_outer_escape_after.json`).
