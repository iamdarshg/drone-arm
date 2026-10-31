# Rev-B hand-route: U10.10 `/1V1` decoupling via

Added a `/1V1` connection from U10 pad 10 at (54.050, 49.800) mm to the existing C11.1 via at (51.520, 47.000) mm. On F.Cu, the route leaves pad 10 west to a new through-via at (53.200, 50.400) mm using two 0.20 mm segments through (53.200, 49.800). The new via is 0.60 mm in diameter with a 0.30 mm drill. A 0.20 mm B.Cu track connects it to the existing C11.1 via. Native board data assigns both pads, both vias and all route segments to `/1V1`.

Source revision: `961e9d2` (`pcb: route rev b RF reset on back copper`). Source board SHA-256: `8b03f2bce12a3f7ea0780387364c46ed9426d75bc0912c7fa64e553a01fd9bc5`.

Project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. KiCad CLI 9.0.2 generated both reports using the matching project and refilled zones.

- Source DRC: 7 violations, 46 unconnected items.
- Candidate DRC: 7 violations, 45 unconnected items.
- Full signatures match on violation type, description, item descriptions and coordinates. Findings remain four hole-clearance, two silk-edge-clearance and one silk-overlap.

Candidate board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`.

- Before DRC SHA-256: `41a281357816bffcba7c5b893762a04fc3255220605d125d1ecd3af129b13d65` (`main_rev_b_manual_route_1v1_u10_pad10_via_before.json`).
- After DRC SHA-256: `bed3ac7979919b210ddd22d289720b855d0233db3a1df6f1824ccbb5ae0d874b` (`main_rev_b_manual_route_1v1_u10_pad10_via_after.json`).

Other unrouted items and existing DRC findings remain; the board is not fabrication-ready.
