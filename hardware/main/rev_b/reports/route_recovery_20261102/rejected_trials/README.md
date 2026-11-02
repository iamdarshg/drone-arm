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

## Fresh trials from the 16-open candidate

These additional trials start from the promoted 16-open board SHA-256 `60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191`.

| Target | Trial | Unconnected | DRC violations | Trial board SHA-256 | DRC report |
|---|---|---:|---:|---|---|
| `/1V1` | direct F.Cu | 15 | 6 | `4cb29f616a494c381f992775f395aedb78dfcc8de5cb6bac1022589785af4249` | `one_v_one_direct.json` |
| `/1V1` | via + B.Cu | 15 | 8 | `8eb9cc7e0eced67e980277e40d67cdefdcf935fe04b4e3ba8c4f6da4a91e2faf` | `one_v_one_bcu.json` |
| `/1V1` | south B.Cu detour | 15 | 11 | `42bf5106f9ffd73f2cfd41d9a557fc280ed6578b9ae4e6d99552ac9f6d0e00d2` | `one_v_one_bcu_detour.json` |
| `/1V1` | via + In1.Cu | 15 | 7 | `b333a9a9a97a132a73593aa3eccac2a22def2e11b90e9f8b95e558b650bfe25e` | `one_v_one_in1.json` |
| `/RF_MISO`, `/RF_MOSI`, `/RF_GPIO2` | coordinated three-net via bundle | 13 | 79 | `53f29bd35313c696df1db117e4ce7f43ad7b1356cdce00b82921f17e316a7e39` | `rf_spi_bundle.json` |
| `/RP2354B MCU/USB_DP_MCU` | standard via pair | 15 | 12 | `06668e6561a5695989a5cc445f99fcc66befb79c30768453f2dff5e9feca9304` | `usb_dp_via.json` |
| `/RP2354B MCU/USB_DP_MCU` | direct F.Cu | 15 | 11 | `4de87dd11de9d75f71c720b0b09ee89eee492b4b93c260235a10769daab849cd` | `usb_dp_direct.json` |
| `/3V3` pad 29 with `/1V1` reroute | alternate south via | 15 | 11 | `ca3308be084ea5c7dd9c10c8dd4780ff64862194e004cb3ddaf652ce946da786` | `three_net_3v3_pad29.json` |
| `/GNSS_RX` with GNSS_PPS via relocation | standard-via B.Cu | 18 | 29 | `39e2c69e44d4e93ea380349cd174224e9f0249382b7e60e9015831636b1a176a` | `gnss_rx_via_and_pps_relocate.json` |
| `/915 MHz RF/CC_RBIAS` | direct 0.25 mm F.Cu trace | 15 | 8 | `ef73f2615c46bae057b6d48d18780ea9b741efa949c8f76acff05f0403caef7f` | `cc_rbias_direct_probe.json` |

The `/1V1` direct track crosses and shorts `/BARO_SDA`; its via alternatives conflict with VREG_AVDD, SENS_MISO, and `/3V3_ADC`. The bundle closes its three intended airwires but splits other connections and creates many shorts, crossings, and clearance errors. None was promoted.

The USB_DP direct route shorts/crosses `/3V3` and GND around C31/C32. The standard-via route conflicts with USB_DM, `/3V3_ADC`, SWDIO/SWCLK, and GNSS_RESET_N; its archived KiCad DRC JSON records 12 violations. The pad-67 escape is constrained by adjacent USB_DM and `/3V3` pads/tracks and an exposed GND pad. A coordinated MCU USB breakout reroute would be required; no USB impedance or fabrication readiness is claimed.

The alternate pad-29 `/3V3` trial moved the `/1V1` destination via to `(54.2, 58.7)`; the via itself fit, but its F.Cu tie crosses MCU RUN and GND. The source escape also conflicts with GND near y=57.08. It closed one airwire but added seven violations.

The `/GNSS_RX` trial relocated the adjacent GNSS_PPS escape via and used standard through-vias for a long B.Cu route. It added two open airwires overall and 25 new DRC violations, including shorts to `/1V1`, GND, GNSS backup/reset, VREG_AVDD, BARO_SCL, and LSM_INT2; the relocated GNSS_PPS via also conflicts with `/3V3` and USB_DM.

The direct `/CC_RBIAS` trace closed U40 pad14 to R40 pad1 but added a VDD_CC112 crossing/short, a GND short at the R40 pad2 branch, and a solder-mask bridge. U40's escape is blocked by VDD_CC112 and neighboring RF_nCS/CC_PA copper; R40's 0.38 mm pad1-to-GND-pad2 gap is narrower than the 0.50 mm needed for this board's 0.25 mm trace and clearances.

The separate attempt to escape U40 pad15 (`/VDD_CC112`) toward the open pad14 `/CC_RBIAS` lane also failed. Four standard-via placements at `(95.25,64.0)`, `(95.25,64.1)`, `(95.25,64.3)`, and `(96.0,64.3)` all retained 16 opens and raised DRC to 8, 7, 9, and 11 violations, respectively. Nearby `/CC_PA` copper and NC pad16 leave no legal via corridor; the y=64.0 placement has only 0.1135 mm clearance to pad16 and 0.0789 mm to `/CC_PA`, below the 0.125 mm board requirement. The y=64.3 and east placements create additional `/CC_PA` conflicts. This coordinated local escape is rejected; none of these boards was promoted. `verify.py` checks the byte hashes and counts of all four reports.
