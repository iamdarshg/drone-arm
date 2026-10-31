# Rev-B hand-route: U10 pad 41 `/3V3` outer escape

Added a 0.20 mm, six-segment F.Cu route from U10 pad 41 at (63.950, 53.800) mm to the existing `/3V3` through-via at (65.500, 52.480) mm. The path exits east to (64.900, 53.800), continues to (67.000, 53.800), rises to (67.000, 50.800), returns west to (64.700, 50.800), descends to (64.700, 52.480), then connects to the via. This outer loop clears the nearby C37 GND pad and GNSS power via. Native board data confirms the pad, via and added segments are `/3V3`.

Source revision: `683edd8` (`pcb: route rev b 3v3 east via escape`). Source board SHA-256: `4a1f8245cfa35ef64fc68c8c71ecac06c5a688d4fb1ff72d09840d08ede830e2`.

Project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. KiCad CLI 9.0.2 generated both reports with the matching project and refilled zones.

- Source DRC: 7 violations, 48 unconnected items.
- Candidate DRC: 7 violations, 47 unconnected items.
- Full violation signatures match on type, description, item descriptions and coordinates. The seven baseline findings remain four hole-clearance, two silk-edge-clearance and one silk-overlap.

Candidate board SHA-256: `bd28ebb9f6ad98b3a90b655ec510726b83dcc103681e15505549f8667bd6588b`.

- Before DRC SHA-256: `520c5569a79a859d8dde0e854014b6befb53e360a67e1ff9c1928119b70481e8` (`main_rev_b_manual_route_3v3_pad41_outer_escape_before.json`).
- After DRC SHA-256: `b7cfd526c0b34c147a4c46b8e0d888a0f6f1f496feb22b2e52d8dd4652139e24` (`main_rev_b_manual_route_3v3_pad41_outer_escape_after.json`).

Other unrouted connections and existing DRC findings remain; this route does not make the board fabrication-ready.
