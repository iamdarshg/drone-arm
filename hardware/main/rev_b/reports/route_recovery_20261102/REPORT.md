# PCB routing recovery — 2026-11-02

This report records incremental routing recovery on the pinned source board. It is not a fabrication release.

## Reproduction and result

The original source board is pinned at `fc80ded19dd070b476866e84de6c872ee3480d89`, with board SHA-256 `28ebb3c19ab3e8f7d3c7ee9acd5ab427a1868142399c27cabd758c7b14c2613f`. The matching project SHA-256 is `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. The historical 16-open candidate board SHA-256 is `60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191`. The 15-open candidate is pinned at `b6761f85473a1e9a8e3a1b335f4974d40afb5deb`, board SHA-256 `f7deda79edbd6ac1a3a3eceba10fed34b283b704f1a61a1fa6a347e5fe153d15`. The pinned 14-open candidate is commit `762d0ad72744380748cac158e1de42ea7305ae42`, board SHA-256 `63b00e91cc1ea46e7e539b383c0deb6c7ac4bdfa833dd14cdd59936aef90e0ef`. The pinned 12-open board at commit `5630b1c` has SHA-256 `a568c7faef6fee3a9a45423d4b7e9653e37e862d0f4db0cada9e3a72651a7a5d`. The pinned 11-open board at commit `c453474` has SHA-256 `2d4288f022dc560135a40339b6b48d25b85f97292ff4f60da1157408bcabf051`. The current board SHA-256 is `1961847383accbc57f6cbbccd21e6038e966d274d18683395b065b83488742e1`.

KiCad CLI DRC used the matching `.kicad_pro` with `--severity-all --format json`. Results: source 42 unconnected / 4 violations; historical candidate 16 / 4; previous candidate 15 / 4; pinned candidate 14 / 4; pinned candidate 12 / 4; pinned candidate 11 / 4; current candidate 10 / 0. The four findings on the pinned 11-open board were J1 USB-C NPTH-to-GND hole-clearance violations: 0.1944 mm measured against the unchanged 0.2000 mm rule. The current board shortens duplicated GND lands J1 A1, B12, A12, and B1 from 0.60 × 1.15 mm to 0.60 × 1.10 mm. The official GCT USB4105 drawing calls for 12 × 1.15 mm lands and gives ±0.15 mm general tolerance for two-decimal dimensions (https://gct.co/files/drawings/usb4105.pdf; PDF SHA-256 `fb331fbabee8392ed2937ed757c1610cb0f174b84625147c0b580a18eea8c0e5`). Pad numbers, positions, shapes, and GND nets remain unchanged. Archived 11-open/4-finding DRC report SHA-256: `4bbd41af3ffea172d9dc6e848f57ff6782011776d70d5b7e6de6056e5779ede9`. Current 10-open/zero-violation DRC report SHA-256: `e7657b83f349c3023bd2ddbe93990cbef34c1f06329bf5fc8158648f0f544a0b`.

The archived reports are `source_drc.json`, `candidate_drc_16_open.json`, `candidate_drc_15_open.json`, `candidate_drc_14_open.json`, `candidate_drc_12_open.json`, `candidate_drc_11_open_4_violations.json`, and `candidate_drc.json`. Run `python3 verify.py` here to reconstruct the pinned stages, replay accepted routes, rerun DRC, compare violation signatures, and compare replay geometry with the promoted board.

## Routing changes and provenance

The starting copper geometry was transplanted from `main_rev_b_routed_clean.kicad_pcb` at revision `c25a070`; source artifact SHA-256 `4f98e74884cfd0b2b088cee07953942f7e79154c7e78ece1c7269e2bbffc29a4`. The current target footprints, pad assignments, zones, and net table were retained.

- `/915 MHz RF/CC1190_BIAS`: joined U41 pad 14 to R50 pad 1.
- `/RF_RESET_N`: joined the R44-to-U40 local segment; the U10 endpoint remains open.
- `/Power and USB/USB_DN_CONN`: joined J1 A7 to B7 using standard through-vias and B.Cu; nearby VBUS and GND copper was adjusted to clear the route.
- `/3V3`: joined the long F.Cu airwire using a standard through-via at `(27.273, 24.598)` and a 0.25 mm B.Cu segment to the existing `/3V3` via at `(49.1875, 58.589)`.
- `/1V1`: moved C11 0.2 mm north and extended its `/1V1` and GND stubs, then connected the open branch with a 0.25 mm In2.Cu route from a standard through-via at `(50.49, 41.5)` to the existing via at `(52.9125, 49.6018)`.
- U10 `/3V3`: moved C39-C41 0.2 mm south, rebuilt the local `/1V1` trunk and GND ties, then added a 0.20 mm F.Cu dogleg at y=55.6 mm joining U10 pads 29 and 24 below the south pad row. This closes one open connection. All 165 footprints and pad-to-net assignments are unchanged.
- `/915 MHz RF/CC_RBIAS`: replaced the local `/VDD_CC112` F.Cu bridge with a project-sized 0.30/0.10 mm F.Cu-to-In1 microvia and routed U40 pad 14 around R40 pad 2.
- `/915 MHz RF/CC1121_RF`: rotated FL40 180 degrees in place and rebuilt the local RF/GND tracks, connecting FL40 pad 2 to FL41 pad 5. The rotation changes pad positions but not pin numbers, pad sizes, or net assignments.
- `/915 MHz RF/CC_LNA_P`: shifted the `/CC_LNA_N` standard via from `(97.3142, 61.2381)` to `(99.3, 61.0)`, adjusted the local `/CC_TRX_SW` route, and joined U40 pad 19 to FL40 pad 7 with a 0.35 mm F.Cu trace. All 165 footprints and 603 pad/net/size mappings remain unchanged. Pin-to-pin trace width and RF impedance have not been validated.
- `/ICM_INT1`: moved C11 1.0 mm north and rebuilt its `/1V1` and GND stubs plus the local `/3V3_ADC` In2 chord, then routed U10 pad 2 to U20 pad 4 with two standard through-vias and a 0.20 mm B.Cu path along the board’s lower edge. This closes one open net. Independent verification confirms C11 is the only moved footprint and no unrelated tracks changed.
- J1 USB-C clearance: shortened only the land lengths of duplicated GND pads A1, B12, A12, and B1 from 1.15 mm to 1.10 mm, within the GCT drawing general tolerance. The replay verifies only these four pad sizes change in addition to the ICM route.

The historical copper transplant resolved 30 original airwires and introduced 4 new ones: `/3V3`, `/915 MHz RF/CC1121_RF`, `/915 MHz RF/CC_RBIAS`, and `/RF_RESET_N`. The later `/1V1`, U10 `/3V3`, `/CC_LNA_P`, and `/ICM_INT1` routes each closed one additional connection. The remaining 10 opens are `/GNSS_RX`, `/HGM`, `/LNA_EN`, `/LSM_INT1`, `/PA_EN`, `/RF_GPIO2`, `/RF_MISO`, `/RF_MOSI`, `/RF_RESET_N`, and `/RP2354B MCU/USB_DP_MCU`. Current unconnected items and their net counts are recorded in `candidate_drc.json`.

## Limits

This candidate is **not fabrication-ready**. It retains 10 unconnected items. The J1 land-size adjustment is within the GCT drawing tolerance, but solder process and connector-land yield have not been validated on a physical assembly. The 0.30/0.10 mm microvia is permitted by project rules and the layer stackup, but a board house's blind-microvia capability has not been verified. The latest RF route has not been impedance-simulated. No fabrication outputs were generated or approved. Additional RF, USB, sensor, GNSS, and MCU route experiments that increased DRC were rejected and archived under `rejected_trials/`. No physical board or hardware test was performed.
