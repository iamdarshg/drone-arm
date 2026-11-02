# PCB routing recovery — 2026-11-02

This records an incremental routing recovery on the `fc80ded19dd070b476866e84de6c872ee3480d89` source board. It is not a fabrication release.

## Reproduction and result

The source board and matching project are pinned by Git revision `fc80ded19dd070b476866e84de6c872ee3480d89`; the board SHA-256 is `28ebb3c19ab3e8f7d3c7ee9acd5ab427a1868142399c27cabd758c7b14c2613f` and the project SHA-256 is `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. The current candidate board SHA-256 is `60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191`. The prior 17-open intermediate DRC is retained as `candidate_drc_17_open.json`.

KiCad CLI DRC was run with the matching `.kicad_pro` beside each board and `--severity-all --format json`. Baseline: 42 unconnected items and 4 violations. Current candidate: 16 unconnected items and the same 4 violations. The four are J1 USB-C NPTH-to-GND hole-clearance findings (0.1944 mm measured against 0.2000 mm required); their normalized descriptions, item references, and coordinates are unchanged. The current candidate DRC JSON SHA-256 is `599987f963bb5e4ef78207ea8404ea081d902bc48fcc35f3795597bfdb196abd`.

The immutable JSON reports are `source_drc.json` and `candidate_drc.json`. Run `python3 verify.py` from this repository root to reconstruct the baseline board and 17-open intermediate from pinned Git objects, replay the 3V3 route with KiCad `pcbnew`, run paired DRC on the source, replay, and promoted board, verify the hashes, and compare normalized DRC outcomes. The replay also checks that footprint and pad mappings are unchanged.

## Routing changes and provenance

The starting copper geometry was transplanted from the historical `main_rev_b_routed_clean.kicad_pcb` artifact at revision `c25a070`; its source artifact SHA-256 was `4f98e74884cfd0b2b088cee07953942f7e79154c7e78ece1c7269e2bbffc29a4`. The current target footprints, pad assignments, zones, and net table were retained. Additional local changes:

- `/915 MHz RF/CC1190_BIAS`: joined U41 pad 14 to R50 pad 1.
- `/RF_RESET_N`: joined the R44-to-U40 local segment; the U10 endpoint remains unconnected.
- `/Power and USB/USB_DN_CONN`: joined J1 A7 to B7 using standard through-vias and B.Cu; adjusted nearby VBUS and GND copper to clear the route and maintain DRC.
- `/3V3`: joined the long F.Cu airwire using a standard through-via at `(27.273, 24.598)` and a 0.25 mm B.Cu segment to the existing `/3V3` via at `(49.1875, 58.589)`.

The historical copper transplant reduced some open connections but also introduced new airwires. Relative to the pinned baseline, 30 original airwires were resolved, while 4 new ones remain: `/3V3` (1), `/915 MHz RF/CC1121_RF` (1), `/915 MHz RF/CC_RBIAS` (1), and `/RF_RESET_N` (1). The candidate has 16 open airwires total; their net counts are recorded in `candidate_drc.json`.

## Limits

This candidate is **not fabrication-ready**. It retains 16 unconnected items and 4 hole-clearance violations. No fabrication outputs were generated or approved. Separate standard-via escape experiments for `/915 MHz RF/CC1121_RF` were rejected because they introduced additional DRC violations; they are archived under `rejected_trials/`. No physical board or hardware test was performed.
