# USB DP, LNA enable, and GNSS RX routing update

This composite candidate starts from board commit `cb58843b73e473eefb7716f31310c84c9affa228` (PCB SHA-256 `4e66a77b67e86817e5674b617e021ea9e4a40c889612c00d212bbcd9cd660d4e`) and project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

Candidate PCB SHA-256: `2356919db26ce9e01417d78d849c0de13541a3827e69a674549cf2245a0b72a3`. Fresh native KiCad 9.0.2 DRC reports **5 unconnected nets and 0 violations**, down from 8/0. The committed `candidate_drc.json` SHA-256 is `7bd687d2e98c4f7782d835792c986b80c92a8f6390a56a22dccc540db78ed9ac`. It closes `/RP2354B MCU/USB_DP_MCU`, `/LNA_EN`, and `/GNSS_RX`.

The remaining open nets are `/LSM_INT1`, `/PA_EN`, `/RF_GPIO2`, `/RF_MISO`, and `/RF_MOSI`. All 165 footprints and 657 physical pads are preserved. The changed copper nets relative to the pinned 8-open board are `/3V3`, `/GND`, `/GNSS_RX`, `/LNA_EN`, and `/RP2354B MCU/USB_DP_MCU`.

USB DP escapes U10 through two stacked microvia transitions and uses In2.Cu. Its local `/3V3` bridge moves to B.Cu through standard vias at `(55.0,40.8)` and `(64.0,39.2)`; nearby local GND copper is cleared and zones are refilled. LNA_EN routes from U10 pad 26 through a standard via at `(58.2,53.1)`, an obstacle-aware In2.Cu path, and a second standard via near `(111.2,57.3)`. GNSS_RX escapes U10 pad 12 through an F.Cu-to-In1 microvia at `(53.9,50.6)`, continues through an In1/In2 microvia pair, then routes on In2 to an In2/In1/F transition at the R32 end. The existing GNSS_PPS geometry is unchanged. All routes pass native DRC after zone refill.

Run `bash run.sh` with KiCad 9 CLI and `pcbnew` installed to retrieve the pinned source, replay changed-net copper, compare all pad instances and complete track/via geometry, and rerun native DRC on source, target, and replay. This candidate is **not fabrication-ready**: five nets remain open, microvia manufacturability is unconfirmed, and RF/USB impedance, assembly yield, and physical operation are unvalidated. No fabrication outputs or hardware tests were performed.
