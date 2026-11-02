# PCB routing recovery — 2026-11-02

This records an incremental routing recovery on the `fc80ded19dd070b476866e84de6c872ee3480d89` source board. It is not a fabrication release.

## Reproduction and result

The source board and matching project are pinned by Git revision `fc80ded19dd070b476866e84de6c872ee3480d89`; the board SHA-256 is `28ebb3c19ab3e8f7d3c7ee9acd5ab427a1868142399c27cabd758c7b14c2613f` and the project SHA-256 is `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. The candidate board SHA-256 is `49b03f98c942f625f46f670fc420dda754b739f362e6fb5d65ce8c36f3a3f88f`.

KiCad CLI DRC was run with the matching `.kicad_pro` beside each board and `--severity-all --format json`. Baseline: 42 unconnected items and 4 violations. Candidate: 17 unconnected items and the same 4 violations. The four are J1 USB-C NPTH-to-GND hole-clearance findings (0.1944 mm measured against 0.2000 mm required); their normalized descriptions, item references, and coordinates are unchanged. The current candidate DRC JSON SHA-256 is `09ab425900f233dbaadf6c4cd4d53b693759f559f8079841bdd8a24f46d6f603`.

The immutable JSON reports are `source_drc.json` and `candidate_drc.json`. Run `python3 verify.py` from this repository root to reconstruct the baseline board from the pinned Git object, run DRC on both boards with the matching project, verify these hashes and compare normalized DRC outcomes.

## Routing changes and provenance

The starting copper geometry was transplanted from the historical `main_rev_b_routed_clean.kicad_pcb` artifact at revision `c25a070`; its source artifact SHA-256 was `4f98e74884cfd0b2b088cee07953942f7e79154c7e78ece1c7269e2bbffc29a4`. The current target footprints, pad assignments, zones, and net table were retained. Additional local changes:

- `/915 MHz RF/CC1190_BIAS`: joined U41 pad 14 to R50 pad 1.
- `/RF_RESET_N`: joined the R44-to-U40 local segment; the U10 endpoint remains unconnected.
- `/Power and USB/USB_DN_CONN`: joined J1 A7 to B7 using standard through-vias and B.Cu; adjusted nearby VBUS and GND copper to clear the route and maintain DRC.

The historical copper transplant reduced some open connections but also introduced new airwires. Relative to the pinned baseline, 30 original airwires were resolved, while 5 new ones appeared: `/3V3` (2), `/915 MHz RF/CC1121_RF` (1), `/915 MHz RF/CC_RBIAS` (1), and `/RF_RESET_N` (1). The candidate has 17 open airwires total; their net counts are recorded in `candidate_drc.json`.

## Limits

This candidate is **not fabrication-ready**. It retains 17 unconnected items and 4 hole-clearance violations. No fabrication outputs were generated or approved. A separate standard-via escape experiment for `/915 MHz RF/CC1121_RF` removed that airwire but was rejected because it raised DRC violations from 4 to 12; it is not included in the candidate. No physical board or hardware test was performed.
