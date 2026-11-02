# HGM and RF reset routing update

This incremental route update starts from board commit `66fddff96ab557f686b391e49647ca5e8d33e11f` (PCB SHA-256 `1961847383accbc57f6cbbccd21e6038e966d274d18683395b065b83488742e1`) and project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The candidate PCB SHA-256 is `4e66a77b67e86817e5674b617e021ea9e4a40c889612c00d212bbcd9cd660d4e`. Fresh native KiCad 9.0.2 DRC found **8 unconnected items and 0 violations**, compared with 10 unconnected items and 0 violations on the pinned source. The committed report `candidate_drc.json` has SHA-256 `33fb8fc21784d6dc935354c9bedfaf6e00cb2b7a6a9df0e2ddb560a3f9cc2329`. It closes `/HGM` and `/RF_RESET_N`. The eight remaining open nets are `/GNSS_RX`, `/LNA_EN`, `/LSM_INT1`, `/PA_EN`, `/RF_GPIO2`, `/RF_MISO`, `/RF_MOSI`, and `/RP2354B MCU/USB_DP_MCU`.

All 165 footprint references and 603 pad assignments, positions, sizes, shapes, and attributes are unchanged. Copper geometry changes are limited to `/1V1`, `/3V3`, `/HGM`, `/ICM_INT2`, `/LSM_nCS`, `/RF_RESET_N`, and `/RP2354B MCU/XIN`. The cleanup merges exact-collinear spans and removes near-zero pad-fanout fragments; it does not change component placement.

Run `bash run.sh` with KiCad 9 CLI and the `pcbnew` Python module installed. The script retrieves the pinned source board from git, checks the project and board hashes, replays only the changed-net copper from the committed candidate, compares the complete pad map and copper geometry, and reruns native DRC on source, candidate, and replay. To validate a candidate before promotion, set `TARGET_BOARD=/absolute/path/to/candidate.kicad_pcb`.

This remains a routing candidate, **not fabrication-ready**: eight nets are still open. The routes use F.Cu-to-In1 and In1-to-In2 microvias, and the board house's ability to manufacture this via structure has not been confirmed. RF impedance, assembly yield, electrical behavior, and physical operation have not been tested. No fabrication files were released and no hardware was tested.
