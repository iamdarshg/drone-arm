# Rev-B manual route attempts (rejected)

Source board revision: drone-arm `main` commit `bb6c001`; source board SHA-256 `a2690937d50b35190720988a20ec890c3bdf230c3e4b6c68d7fff5a461043c74`. Baseline DRC was run with a same-stem copy of `main_rev_b.kicad_pro` and board SHA above: 7 violations, 56 unconnected items (DRC JSON SHA-256 `109eb4e459c8b04d83110491f168b38ce904ec496d423e39e0691bf3c348b173`). All candidate files and reports are temporary under `/tmp`; no source board was changed.

| Candidate | Candidate board SHA-256 | Matching-project DRC | Result |
|---|---|---:|---|
| BARO_DRDY west escape + In1 corridor | `2f666b31ac5024b49f0b956516b694c55c2d1b45eb327851ec928cd5b43fe36f` | 41 violations, 55 unconnected (`7cd43c40af80937bae5e62cdf637474cea418875867e480e1aa4cd1f4e396eaf`) | Rejected: crossings/clearance along shared sensor corridor. |
| `/915 MHz RF/CC_LNA_P` direct F.Cu | `1ca6994eaf314a3d87f4715afb282897da1ea95d8ed71260bd3e08d8105e80cd` | 12 violations, 55 unconnected (`eab995e432db7d8f29175eeda9f3dfdea1739be99f6897d6d8a9153adcf4f63b`) | Rejected: shorts/crossings into adjacent RF nets, zone clearance and mask bridge. |
| ICM_INT1 west breakout + In2 | `5ca6a64c9b6f8e6cadf416cfc8fdf7dfe3ebd5410483c0ef7ea754f47ef89ae2` | 34 violations, 55 unconnected (`925bfa919af4cfde3efe6ec0de7759efb49b689aba122e44fa41815720fa9594`) | Rejected: shorts/clearance to 3V3_ADC, GND, VREG_AVDD, and sensor-region objects. |
| SYS_5V D2 to existing left-edge stub, F.Cu | `1e30aca3790172fb5f787c5fb6e501fc92db588a6e8d20c280769e7c0d82d510` | 11 violations, 55 unconnected after zone refill (`07977638234ddd1ae35fdd47384a69ce8229d25d8b96be2a9f27a1f1a88e7f1c`) | Rejected: crosses CHASSIS and shorts R5 GND pad/via at (16.825,69.0). Handed off to separate power-route agent to try alternate layers. |
| SWCLK J10.1 to U10.33, vias + In2 | `62ee118e282670f0405fed6cf41b12bdd258cfe5a85e322ab045af36917ac055` | 20 violations, 55 unconnected after zone refill (`89932af2a3eb9dcc5ebe6688fa7cf60df49dce8ba4ee5945a7d1e980bc67e08e`) | Rejected: U10-side escape overlaps C39 pads and nearby 1V1/SENS_SCLK copper. |

The source board remains at 56 unrouted connections and 7 baseline findings. No candidate passed acceptance, so none was committed or pushed.
