# LSM interrupt routing update

This candidate starts from commit `899adceeb5b6afe9004e0ebb5df4771a073d8e5c` (PCB SHA-256 `2356919db26ce9e01417d78d849c0de13541a3827e69a674549cf2245a0b72a3`) and project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

Candidate PCB SHA-256: `b107359cf9adf6bb08e9ecd75ea2e0e8e4bddc3036be4e1fd8c3261f36924656`. Fresh native KiCad 9.0.2 DRC reports **4 unconnected nets and 0 violations**, down from 5/0. The committed `candidate_drc.json` SHA-256 is `a4661958274ad1dac3bc655488673b44d4619df49f35e3cbb9b16b8a10ca3fd2`. The `/LSM_INT1` connection is now complete.

The remaining open nets are `/PA_EN`, `/RF_GPIO2`, `/RF_MISO`, and `/RF_MOSI`. Only `/3V3`, `/LSM_INT1`, and `/SENS_SCLK` copper changes relative to the pinned source. All 165 footprints and 657 physical pad instances are preserved. The route escapes U10 pad 4 with an F.Cu-to-In1 microvia, transitions In1-to-In2 at `(51.5,49.0)`, returns In2-to-In1 at `(50.6,56.6)`, and reaches U21 pad 4 through a stacked transition at `(58.1,83.9)`. Local `/3V3` and `/SENS_SCLK` fanout copper is rerouted to create a legal U10 escape. MAIN_LED_A and ICM_INT2 remain unchanged.

Run `bash run.sh` with KiCad 9 CLI and `pcbnew` installed to retrieve the pinned source, replay the changed-net copper, compare every physical pad and full copper geometry, and rerun native DRC on source, target, and replay. This candidate is **not fabrication-ready**: four nets remain open, microvia manufacturing capability is unconfirmed, and electrical/assembly performance is untested. No fabrication outputs or physical tests were performed.
