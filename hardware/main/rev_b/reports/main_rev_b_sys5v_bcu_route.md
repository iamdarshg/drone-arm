# Rev-B `/SYS_5V` B.Cu route

This commit adds one manually routed `/SYS_5V` connection on the Rev-B board. The route connects D2.2 to the existing same-net F.Cu track at `(16.65,71.5)` through two new through-vias. The lower via is at the track endpoint; it was not present on the source board.

## Source and candidate pins

- Exact source commit: `f5f7c5211a292cf59dfdfe130782a00d265f542c`
- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Candidate board SHA-256: `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee`
- Matching `main_rev_b.kicad_pro` SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- KiCad pcbnew and CLI: `9.0.2`
- Both baseline and candidate were explicitly zone-filled with pcbnew `ZONE_FILLER.Fill(board.Zones())`, then run through CLI DRC with `--severity-all` and matching project context. Refill script SHA-256: `221f0995b7154e7f5e88c69c4b6e01002477a72f4e5873f11e50067866baf8a6`.

## Route geometry

- D2.2 `/SYS_5V` `(18.65,31.0)` to new F.Cu/B.Cu via `(18.65,32.0)` using 0.4 mm F.Cu copper.
- New 0.4 mm B.Cu path: `(18.65,32.0)` → `(18.65,68.0)` → `(18.0,68.0)` → `(18.0,71.5)` → `(16.65,71.5)`.
- New F.Cu/B.Cu via at `(16.65,71.5)` joins the existing `/SYS_5V` F.Cu segment there, which is 0.5 mm wide.
- Both vias are 0.8 mm diameter with 0.4 mm drills. The dogleg detours around the existing GND via at `(16.825,69.0)`.

Native pcbnew inspection confirms the new tracks and both vias are `/SYS_5V`; the destination via joins the existing same-net F.Cu stub. An independent read-only verification also confirmed continuity and repeated the matching-project DRC.

## DRC evidence

| Board | Violations | Unconnected items | JSON SHA-256 |
| --- | ---: | ---: | --- |
| Zone-filled source | 7 | 45 | `436c1181e9b9bb489a1b198616468aa7c6f2c1abb1a1a16106c4a816746fa959` |
| Zone-filled candidate | 7 | 44 | `c1a6e05cd62178e634a422dc183599ea48da7f3f5d17489bfcf1a6eb63f64da6` |

All seven baseline DRC violation signatures are unchanged. The route removes one unconnected item and introduces no new DRC violations. This is a routing result only; it does not establish a current-capacity, fabrication, or system-readiness rating.
