# Rejected routes from the 17-open candidate

The sensor, power, and copper-restore trials below started from the same candidate board SHA-256 `49b03f98c942f625f46f670fc420dda754b739f362e6fb5d65ce8c36f3a3f88f` and matching project file. The standard-via CC1121_RF trial used a separate 17-open candidate copy with SHA-256 `b18d76ab8195415007f6defbf070c47ebebfc0bffa62bf7923f36e9fb1644111`. Every trial was rejected because its DRC count exceeded the applicable 4-violation baseline. The JSON reports are preserved here; their `unconnected_items` and `violations` arrays provide observed counts and detailed failure locations.

| Target | Trial | Unconnected | DRC violations | Trial board SHA-256 | DRC report |
|---|---|---:|---:|---|---|
| `/3V3` | direct short | 16 | 7 | `23ad6a3532a2609b3abb3d076dce3bfe9d1c4df3069fa9faa98780dddd503222` | `try3v3short.json` |
| `/3V3` | dogleg | 16 | 14 | `d0272e7f841c9fd1e46a549754ff0ff15b452556574161111695979ea27e2e31` | `try3v3out.json` |
| `/3V3` with local XIN and `/1V1` reroute | three-net escape | 17 | 15 | `2a31aae1d4f8a2033cd2c5235e229973c44c235aba576c65f00fd6c32b96a3d1` | `three_net_3v3_escape.json` |
| `/1V1` | endpoint vias | 16 | 19 | `ab81c7b5743e5cfe0513aa290c737cae7f6eb21c88ec3dbcc74e0f24e6497dd8` | `try1v1v.json` |
| `/1V1` | B.Cu/via route | 16 | 15 | `0b5180145dac8eb6d11abc61acecb596905c04970a6951327a1bb1fd1980bdcc` | `try_1v1.json` |
| `/GNSS_RX` | direct F.Cu | 16 | 38 | `38018a608dd5ce3c8aee0f0637ccc5e5975d92be9f82afb81722ae703f0ea43a` | `gnss.json` |
| `/HGM` | B.Cu direct | 16 | 16 | `f41db5dab368d5b3099c5df69430f61dff713e40bad42335e2ce0d1da9d20d06` | `hgm_direct_bcu.json` |
| `/HGM` | F.Cu escape + B.Cu | 16 | 22 | `20981924ddd3952032485a34237cca8b9441fc5b96da12a2ca8740fe92d6347b` | `hgm_escape3.json` |
| `/LNA_EN` | B.Cu direct | 16 | 16 | `c5be332cfdec03a8adbb061c956af16fa4a05ad8996fe75c4f4d7d83e01498eb` | `lna_en_direct_bcu.json` |
| `/PA_EN` | B.Cu direct | 16 | 19 | `9d115ade08a6574042633cb64a9813c47ddd5c276ddf498573b06fada21140bc` | `pa_en_direct_bcu.json` |
| `/ICM_INT1` | In1.Cu with vias | 16 | 20 | `84b1787fd130c68e3a555fa0a2234a88fb64e97e10adf462672c9033256abe9f` | `icm.json` |
| `/LSM_INT1` | direct F.Cu | 16 | 54 | `237530c5e581a8885fbf60e45124e6e473f77a83aaa396da23541b8d94075ab5` | `lsm_fcu.json` |
| `/915 MHz RF/CC1121_RF` | standard-via escape | 17 | 9 | `b18d76ab8195415007f6defbf070c47ebebfc0bffa62bf7923f36e9fb1644111` | `cc1121_rf_via.json` |
| `/915 MHz RF/CC_RBIAS` | restore baseline copper | 18 | 15 | `0c91b660b3b486274ed28f9dcc7afa239bc785d52aef6d219324f4c65c1a7f0d` | `cc_rbias_restore.json` |
| `/RF_RESET_N` | restore baseline copper | 20 | 32 | `6ff4ed34368afec03171bbee53f327573013815b7d1cb00dc6c189966ef6c6fe` | `rf_reset_restore.json` |

The sensor and power trials were freshly replayed against the 17-open board with a matching `.kicad_pro`. The `/CC1121_RF` candidate was also tested at 17 opens; although that DRC report shows 9 violations, it is a different trial than the earlier 19-open/12-violation attempt (board hash `d0c8cf741c9866557626f31fa389054c2d76097bde7b4c94e7eb047aed66cf1e`). These DRC snapshots are diagnostic evidence; the candidate board and the pinned source-to-candidate verification remain in the parent directory. Trial board blobs are not included in Git; their hashes are listed so any separately retained copy can be identified.

The three-net `/3V3` escape moved the local XIN escape and rerouted `/1V1` with standard through-vias before joining the `/3V3` endpoints. It reduced no total airwires and added 11 DRC violations; the new `/1V1` via conflicts with C41/GND/MCU copper, and the `/3V3` segment violates C40 clearance. This points to a broader teardown of the local MCU/capacitor routing cluster if that pad-29 connection is to be pursued.
