# Rev-B hand-route: east-side `/3V3` via escape

Added a 0.20 mm, two-segment F.Cu route from the existing `/3V3` track endpoint at (64.700, 50.200) mm down the east side of U10 to (64.700, 52.480), then east to the existing `/3V3` through-via at (65.500, 52.480). This keeps the route west of the nearby C36 GND via at (65.500, 51.520). Native board data confirms both endpoint islands and the new segments are `/3V3`.

Source revision: `c09b900` (`pcb: add south edge rev b 3v3 route`). Source board SHA-256: `c57705d0db354edef32f842bd0d3c3ee37f60dcdad16ff8fe1f2400aa13f524c`.

Project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. Before and after reports were generated with the matching project and KiCad CLI 9.0.2.

- Source DRC: 7 violations, 49 unconnected items.
- Candidate DRC: 7 violations, 48 unconnected items.
- Full signatures match on violation type, description, item descriptions and coordinates. Findings remain four hole-clearance, two silk-edge-clearance and one silk-overlap.

Candidate board SHA-256: `4a1f8245cfa35ef64fc68c8c71ecac06c5a688d4fb1ff72d09840d08ede830e2`.

- Before DRC SHA-256: `09f85151cd36768942e34ff328cb715ff389f6e477b2c107ff97f4173472d27b` (`main_rev_b_manual_route_3v3_east_via_escape_before.json`).
- After DRC SHA-256: `d94a38d287460034a6b6177c8589b08c4031a556030b410c1c67ccedfb333953` (`main_rev_b_manual_route_3v3_east_via_escape_after.json`).

Unrouted items and existing DRC findings remain; this route does not make the board fabrication-ready.
