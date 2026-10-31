# Rev-B hand-route: west-edge `/3V3` branch

Added a 0.20 mm, three-segment F.Cu connection from U10 pad 5 at (54.050, 47.800) mm to the existing `/3V3` track endpoint at (56.800, 44.200) mm. The trace leaves the west side of U10 through (53.300, 47.800), then runs north outside the package to (53.300, 44.200) and east to the existing track. Native board data confirms pad 5, the existing track and the new segments are all `/3V3`.

Source revision: `bb5f651` (`pcb: route northwest rev b 3v3 link`). Source board SHA-256: `40b348a94fbb2d32955f2ed9a9f42196874b5ad0f0901c2edac0d67106fc3e04`.

Project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. Native KiCad CLI 9.0.2 DRC was run with this matching project and refilled zones.

- Source: 7 violations, 52 unconnected items.
- Candidate: 7 violations, 51 unconnected items.
- Full signatures match across type, description, item descriptions and item coordinates. The seven findings remain four hole-clearance, two silk-edge-clearance and one silk-overlap.

Candidate board SHA-256: `bdb6ae9f74475b2a45e081731598a395edead2d2cc26bf848634d63c44c431f0`.

- Before DRC SHA-256: `d728c48912102e491c039811222f010fa6f8ad390a4a17bb489b9b81908e32dd` (`main_rev_b_manual_route_3v3_west_escape_before.json`).
- After DRC SHA-256: `8985c9fd29e0f6be9dc8e048bfe1c93eb2308d9fd2294e9d08102c824b6e014d` (`main_rev_b_manual_route_3v3_west_escape_after.json`).

Unrouted items and existing DRC findings remain; this local route does not make the board fabrication-ready.
