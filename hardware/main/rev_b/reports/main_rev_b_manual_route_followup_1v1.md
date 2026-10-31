# Main Rev-B manual route follow-up: local 1V1 connection

Starting point: `b60256a` (`pcb: route additional local rev b 3v3 links`). This change was made in an isolated worktree.

Added one 0.20 mm F.Cu connection from U10 pin 32 to the nearby existing `/1V1` via at (59.52, 56.50) mm. The schematic netlist confirms U10 pin 32 (DVDD) is on `/1V1`. The route uses two short segments and was followed by a full zone refill.

Native KiCad 9.0.2 DRC with the board project rules was run before and after this batch:

- Before: 7 violations, 63 unconnected items.
- After: 7 violations, 62 unconnected items.
- Violation types and counts are unchanged: 4 hole-clearance, 2 silk-edge-clearance, and 1 silk-overlap.

No fabrication-readiness claim is made. Other unrouted nets and existing DRC findings remain.

## Source hashes

- Before board: `5ea8653e9f085fe1cab9d9230fc4c0edee7b4f03a772124c2b3849145a7bc3c5`
- After board: `ad073a8f42b771ec0897f18fdd37b4a6429acf39a53bd0444ec73ba34befe314`
- Project: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Schematic: `63761ab21083500d55c03a3fea0e243885a5e627e4d424463861cafc6663aae9`

Full native DRC reports are in the adjacent `main_rev_b_manual_route_followup_1v1_before.json` and `main_rev_b_manual_route_followup_1v1_after.json` files.
