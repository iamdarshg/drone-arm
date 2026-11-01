# Rejected peripheral control-net via probes from `2a780a7`

Both candidates were created on disposable copies of the board at source commit `2a780a7` (docs-only successors to the same board hash `095aa3fd44285b885c2a4ca68c05633b169dd053fbc8407d2160e8585e73336b`). The matching project SHA-256 is `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. Source and candidates were explicitly zone-refilled using `pcbnew.ZONE_FILLER.Fill(board.Zones())`; KiCad CLI 9.0.2 ran `pcb drc --severity-all --format json` with the matching project. The paired baseline was 7 violations / 42 unconnected; baseline JSON SHA-256 `a8472d31d92c4001aac62631a9f2091c8b2a92055c3c30e6ffc2655697a37f54`.

| Net and topology | Candidate board SHA-256 | Candidate DRC JSON SHA-256 | Result |
| --- | --- | --- | --- |
| `/PA_EN`: U10.25 `(56.8,54.95)` F.Cu to via `(56.8,56.2)`; existing track endpoint `(109.5,57.51)` F.Cu to via `(108.3,57.51)`; 0.20 mm B.Cu span | `eb9dd37591864e18669ed99daa9ea8fb2d782cdf201132f6a038eaa8e4a0fe6d` | [`route_probe_2a780a7_rf_pa_en_candidate.json`](route_probe_2a780a7_rf_pa_en_candidate.json), `8ac6541d181dbbc50295f86c7537bb7988b275414a2667fdba90484c83dfb9a2` | 26 violations / 41 unconnected; rejected for new shorts, crossings, clearances, holes and mask bridges. |
| `/RF_GPIO2`: U10.22 `(55.6,54.95)` F.Cu to via `(55.6,56.2)`; U40.4 `(91.5625,60.75)` F.Cu to via `(90.2,60.75)`; 0.20 mm In2.Cu span | `72048a163b1b2df749d8816255036a15aefd658e5af26d9ca0a49ad9057ab4b5` | [`route_probe_2a780a7_rf_gpio2_candidate.json`](route_probe_2a780a7_rf_gpio2_candidate.json), `2856ccb4aa8b936ffc6b418285001072dc536c7c35b54524a7608832f448bc99` | 16 violations / 42 unconnected; rejected for shorts to `/1V1`, `/CAN_INT`, `/ICM_nCS`, RF supply and GND, plus crossings and clearances. |

Neither route reduced the unconnected count without new DRC signatures. No board, netlist, or design expectation was changed.
