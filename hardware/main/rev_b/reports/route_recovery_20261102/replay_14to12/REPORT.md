# Replayed partial routing: 14 to 12 open items

This isolated candidate closes `/915 MHz RF/CC_RBIAS` and `/915 MHz RF/CC1121_RF` without adding DRC findings. The pinned source is commit `762d0ad72744380748cac158e1de42ea7305ae42`, PCB SHA-256 `63b00e91cc1ea46e7e539b383c0deb6c7ac4bdfa833dd14cdd59936aef90e0ef`, and project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The replay removes the two F.Cu `/VDD_CC112` bridge segments beside U40 and reconnects the pad-15 branch through a project-sized 0.30/0.10 mm F.Cu-to-In1 microvia. A 0.20 mm F.Cu route connects U40 pad 14 `/CC_RBIAS` around the R40 GND pad. The local RF cluster then rotates FL40 180° in place and rebuilds the affected `/CC_LNA_N`, `/CC_TRX_SW`, `/CC_TXPA`, ground, and `/CC1121_RF` topology. The exploratory `/CC_LNA_P` direct trace is discarded because it introduces DRC errors, so that net remains open.

Native KiCad DRC reports **14 to 12 unconnected items** and **4 to 4 violations**. All four findings are unchanged J1 NPTH-to-GND clearances at 0.1944 mm against the unchanged 0.2000 mm rule. The candidate does not alter project rules or any pad net/size assignments. It preserves 165 footprints and 603 pad/net/size mappings; FL40 is the only footprint whose orientation changes.

The 0.30/0.10 mm microvia is configured in the project rules and the F.Cu-to-In1 layer transition matches the four-layer stackup. A board house's blind-microvia capability has not been specified or verified. The standard 0.60/0.30 mm through-via control at this escape location adds five clearance violations. This partial route is therefore not a fabrication release, and the board remains unrouted on 12 connections.

From the repository root, run `hardware/main/rev_b/reports/route_recovery_20261102/replay_14to12/run.sh`. The runner materializes the pinned source, applies each routing stage in separate pcbnew processes, refills copper zones, reruns native DRC on source and candidate, checks baseline signatures and pad mappings, and compares replayed geometry with the promoted board.
