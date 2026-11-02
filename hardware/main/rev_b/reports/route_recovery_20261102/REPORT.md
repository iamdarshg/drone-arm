# PCB routing recovery — 2026-11-02

This report records incremental routing recovery on the pinned source board. It is not a fabrication release.

## Reproduction and result

The source board is pinned at `fc80ded19dd070b476866e84de6c872ee3480d89`, with board SHA-256 `28ebb3c19ab3e8f7d3c7ee9acd5ab427a1868142399c27cabd758c7b14c2613f`. The matching project SHA-256 is `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. The historical 16-open candidate board SHA-256 is `60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191`. The 15-open candidate is pinned at `b6761f85473a1e9a8e3a1b335f4974d40afb5deb`, board SHA-256 `f7deda79edbd6ac1a3a3eceba10fed34b283b704f1a61a1fa6a347e5fe153d15`. The current board SHA-256 is `63b00e91cc1ea46e7e539b383c0deb6c7ac4bdfa833dd14cdd59936aef90e0ef`.

KiCad CLI DRC used the matching `.kicad_pro` with `--severity-all --format json`. Results: source 42 unconnected / 4 violations; historical candidate 16 / 4; previous candidate 15 / 4; current candidate 14 / 4. The four remaining findings are J1 USB-C NPTH-to-GND hole-clearance violations: 0.1944 mm measured against the unchanged 0.2000 mm rule. Their normalized descriptions, references, and coordinates match the baseline. Current DRC report SHA-256: `10e5deac975ca4da513e5dd2c3e17377d4a84885832f86b2267890a8298591f8`.

The immutable reports are `source_drc.json`, `candidate_drc_16_open.json`, `candidate_drc_15_open.json`, and `candidate_drc.json`. Run `python3 verify.py` here to reconstruct the pinned boards, replay the accepted routes, rerun DRC, compare violation signatures, and compare the replayed candidate's footprint, pad, and copper geometry with the promoted board.

## Routing changes and provenance

The starting copper geometry was transplanted from `main_rev_b_routed_clean.kicad_pcb` at revision `c25a070`; source artifact SHA-256 `4f98e74884cfd0b2b088cee07953942f7e79154c7e78ece1c7269e2bbffc29a4`. The current target footprints, pad assignments, zones, and net table were retained.

- `/915 MHz RF/CC1190_BIAS`: joined U41 pad 14 to R50 pad 1.
- `/RF_RESET_N`: joined the R44-to-U40 local segment; the U10 endpoint remains open.
- `/Power and USB/USB_DN_CONN`: joined J1 A7 to B7 using standard through-vias and B.Cu; nearby VBUS and GND copper was adjusted to clear the route.
- `/3V3`: joined the long F.Cu airwire using a standard through-via at `(27.273, 24.598)` and a 0.25 mm B.Cu segment to the existing `/3V3` via at `(49.1875, 58.589)`.
- `/1V1`: moved C11 0.2 mm north and extended its `/1V1` and GND stubs, then connected the open branch with a 0.25 mm In2.Cu route from a standard through-via at `(50.49, 41.5)` to the existing via at `(52.9125, 49.6018)`.
- U10 `/3V3`: moved C39-C41 0.2 mm south, rebuilt the local `/1V1` trunk and GND ties, then added a 0.20 mm F.Cu dogleg at y=55.6 mm joining U10 pads 29 and 24 below the south pad row. This closes one open connection. All 165 footprints and pad-to-net assignments are unchanged.

The historical copper transplant resolved 30 original airwires and introduced 4 new ones: `/3V3`, `/915 MHz RF/CC1121_RF`, `/915 MHz RF/CC_RBIAS`, and `/RF_RESET_N`. The later `/1V1` and U10 `/3V3` routes each closed one additional connection. Current unconnected items and their net counts are recorded in `candidate_drc.json`.

## Limits

This candidate is **not fabrication-ready**. It retains 14 unconnected items and 4 hole-clearance violations. No fabrication outputs were generated or approved. Additional RF, USB, sensor, GNSS, and MCU route experiments that increased DRC were rejected and archived under `rejected_trials/`. No physical board or hardware test was performed.
