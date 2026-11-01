# Accepted `/ICM_nCS` In1.Cu route from `f01ef6f`

This route connects U10.80 `/ICM_nCS` to the existing sensor-side same-net route on the Rev-B board at source commit `f01ef6f555f099a2b2ce08c278534e448b387889`. Source board SHA-256: `2bcb4d6ba0293d280306393d070ccc16a0b6755d0d34b77cf89aefac7d4e638a`. Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The 0.20 mm F.Cu escape runs from U10.80 `(55.2,45.05)` to a 0.60/0.30 mm F.Cu/B.Cu through-via at `(55.2,46.2)`. A 0.20 mm In1.Cu detour then runs `(55.2,46.2)` → `(55.2,62.5)` → `(52.5,65.2)` → the existing `/ICM_nCS` via at `(51.1777,80.9098)`. The route skirts the `/RP2354B MCU/MAIN_LED_A` In1.Cu track by staying east of it until below its endpoint.

Baseline and candidate were explicitly zone-refilled with pcbnew 9.0.2 `ZONE_FILLER.Fill(board.Zones())` using `route_probe_zone_fill.py` (SHA-256 `221f0995b7154e7f5e88c69c4b6e01002477a72f4e5873f11e50067866baf8a6`), then checked with KiCad CLI 9.0.2 `pcb drc --severity-all --format json` and the matching project. Baseline: **7 violations / 43 unconnected**, JSON SHA-256 `bbce54e369bca7504ab480bfb484fb7c83d22f986761c333c6cc85e812609bdc`. Candidate: **7 violations / 42 unconnected**, JSON SHA-256 `e6959f689be79e23f515a014bef96b9998ec06b0f18c0013d93ac54b90870156`. All seven DRC signatures match exactly; the U10.80 `/ICM_nCS` open is resolved.

An independent read-only pcbnew comparison confirmed the candidate adds exactly four `/ICM_nCS` tracks and one correctly sized through-via, with no removed or modified board objects; the new In1.Cu endpoint joins the existing same-net via. The zone-refilled candidate board SHA-256 is `095aa3fd44285b885c2a4ca68c05633b169dd053fbc8407d2160e8585e73336b`.

This removes one unconnected item. The board still has 42 unconnected items and 7 DRC findings; it is not fabrication-ready.
