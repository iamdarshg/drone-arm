# Rev-B hand-route: northeast `/3V3` island connection

Added a 0.20 mm, three-segment F.Cu route between two existing `/3V3` islands around the north-east edge of U10. The path runs from the existing stub at (62.520, 43.500) mm north to (62.520, 42.500), east to (64.700, 42.500), then south to (64.700, 46.200) mm, joining the existing `/3V3` trace there. It avoids the adjacent C33 GND pad at (63.480, 43.500) mm. All endpoints and segments are assigned `/3V3` in the native board model.

Source revision: `263ca77` (`pcb: connect local rev b 3v3 islands`). Source board SHA-256: `b5ed4d4bd2c50b7d58569a7f7ede820350f8c8303ed4ffd1e1a52966bd65e022`.

Native KiCad CLI 9.0.2 DRC with the matching `main_rev_b.kicad_pro` and refilled zones:

- Source: 7 violations, 54 unconnected items.
- Candidate: 7 violations, 53 unconnected items.
- All seven baseline violation signatures are identical before and after.

This route connects a previously separate `/3V3` trace island. Other unrouted items and existing DRC findings remain; the board is not fabrication-ready.

Candidate board SHA-256: `cf8df6593ca0426729adec0e43d534455ed6a062dcb18043d669acd2274a48fe`.

DRC report SHA-256 values:

- Before: `dd2c62a46668cfb9661fc559f2f663cc5ac68e6e7457b4bfc83480eebe021054` (`main_rev_b_manual_route_3v3_north_east_escape_before.json`).
- After: `8a003ba1bbf963219ada959ab2ff8a821775bd8ea45e01473cebef05b7db7d90` (`main_rev_b_manual_route_3v3_north_east_escape_after.json`).
