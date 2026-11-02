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

These additional trials start from the historical 16-open board SHA-256 `60da183ad175be4e1db9daa57a705714a84dd0d7b1f0b2a942f6beeee3a9a191`, pinned at Git revision `7f9a9eff504ad5ebc7f33bd1af25ee743d9381c1`. Replayers materialize that immutable board from Git so they remain valid after the current board advanced to 15 opens.

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

For the MCU USB pair, the project assigns both nets to the Default net class with 0.25 mm track width and default diff-pair values of 0.20 mm width / 0.25 mm gap, but has no explicit `diff_pair_dimensions`, no USB length/skew limits, and no stackup/dielectric impedance model. The diagnostic paired microvia breakout used 0.3 mm laser vias with 0.1 mm drills from F.Cu to In1.Cu; it reduced opens to 15 but produced 15 violations, including a 0.100 mm via-to-via clearance, crossings, and shorts. It is not an acceptable route or a fabrication claim. Replay it with `/usr/bin/python3 reproduce_usb_dp_microvia_pair.py`; the archived DRC is `usb_dp_microvia_pair.json`.

The direct `/CC_RBIAS` trace closed U40 pad14 to R40 pad1 but added a VDD_CC112 crossing/short, a GND short at the R40 pad2 branch, and a solder-mask bridge. U40's escape is blocked by VDD_CC112 and neighboring RF_nCS/CC_PA copper; R40's 0.38 mm pad1-to-GND-pad2 gap is narrower than the 0.50 mm needed for this board's 0.25 mm trace and clearances.

The separate attempt to escape U40 pad15 (`/VDD_CC112`) toward the open pad14 `/CC_RBIAS` lane also failed. Four standard-via placements at `(95.25,64.0)`, `(95.25,64.1)`, `(95.25,64.3)`, and `(96.0,64.3)` all retained 16 opens and raised DRC to 8, 7, 9, and 11 violations, respectively. Nearby `/CC_PA` copper and NC pad16 leave no legal via corridor; the y=64.0 placement has only 0.1135 mm clearance to pad16 and 0.0789 mm to `/CC_PA`, below the 0.125 mm board requirement. The y=64.3 and east placements create additional `/CC_PA` conflicts. This coordinated local escape is rejected; none of these boards was promoted. `verify.py` checks the byte hashes and counts of all four reports.

Additional isolated RF probes against the same 16-open board confirmed the RF region needs coordinated rerouting. The `/CC_LNA_P` direct and detoured F.Cu paths all closed one airwire but introduced shorts, narrow clearances, crossings, or solder-mask bridges. A standard via pair to B.Cu produced RF_LNA_P-to-RF_LNA_N/GND shorts. A direct `/CC1121_RF` bridge between FL40 and FL41 crossed `/CC_LNA_P`, breached clearances, and bridged NC-pad masks.

| Target | Trial | Unconnected | DRC violations | Trial board SHA-256 | DRC report |
|---|---|---:|---:|---|---|
| `/CC_LNA_P` | direct F.Cu, 0.20 mm | 15 | 9 | `4886bf1a57f26f26ba39991bffc1171dabfda6d80311ee88cfd8e0a2d6d2398f` | `cc_lna_p_direct.json` |
| `/CC_LNA_P` | F.Cu detour via y=63.0 mm | 15 | 28 | `eeaa6dc8618c89c9836c3d796d2d7a2bca957ca744646586a595d64a8c73cc56` | `cc_lna_p_detour_y0.json` |
| `/CC_LNA_P` | F.Cu detour via y=63.4 mm | 15 | 19 | `84655552387ca4f9388919a0679d476fb4d879ae04526da5fb1f248fdd5b7406` | `cc_lna_p_detour_y1.json` |
| `/CC_LNA_P` | F.Cu detour via y=60.8 mm | 15 | 29 | `b51a1087d70d96921f70657c1a886104de2218d9bd1f7ab7d437bf0317ce5391` | `cc_lna_p_detour_y2.json` |
| `/CC_LNA_P` | F.Cu detour via y=60.4 mm | 15 | 33 | `a9ad74373cb231ba1f5d63feb8911421459b49afce92f7affd665671b3ba490` | `cc_lna_p_detour_y3.json` |
| `/CC_LNA_P` | partial F.Cu detour via y=63.0 mm | 15 | 26 | `233fab69a01faf76bcee7fccf6e4ae18bb825b7ee2d7c61d5bed220cc92b5129` | `cc_lna_p_detour_y4.json` |
| `/CC_LNA_P` | standard via pair + B.Cu | 15 | 26 | `40767c128768945136832b66858fc5c509e1757aa62956002e4e6054519c6889` | `cc_lna_p_via_pair.json` |
| `/CC1121_RF` | direct F.Cu, 0.20 mm | 15 | 10 | `41d34016611a14ce756e50a52c001f1c8f2b37d11cd776d7cf69c1d1ab825ac0` | `cc1121_rf_direct.json` |

These DRC snapshots were generated from isolated copies of the pinned board and matching project. `verify.py` checks the report content hashes and observed counts; listed board hashes identify the exact trial boards, which were retained in the temporary investigation workspaces rather than promoted.

A coordinated U10 breakout trial routed `/PA_EN`, `/LNA_EN`, and `/HGM` through paired F.Cu/B.Cu escapes. It closed those three airwires (16 to 13 opens) but raised DRC violations from 4 to 48 in the archived run. Replays produce 47 or 48 violations while preserving the same open count; one shorting-items report varies with KiCad's connectivity report ordering. The U10 south-edge fanout conflicts with `/1V1`, GND, XIN, and adjacent RF copper; the proposed 0.15 mm traces also violate the 0.20 mm minimum. The remote `/PA_EN` escape hits the nearby track cluster. This trial is rejected. Its report is `mcu_three_net_breakout.json`; run `/usr/bin/python3 reproduce_mcu_three_net_breakout.py` (the system KiCad Python binding) to reconstruct the candidate. Candidate board SHA-256 from the original run: `d6fcdf35e8792dbeb664cf601cd1d78d94893130ce219743a53f4950b2af20ae`.

A legal-width `/PA_EN` probe used a 0.25 mm F.Cu escape from U10 pad25 to a 0.70/0.30 mm through-via at `(56.8,55.6)`, then a 0.25 mm B.Cu link to a second 0.70/0.30 mm via at `(109.5,58.1135)` on the existing track. It reduced opens to 15 but raised violations to 18. Source-via alternatives across B.Cu, In1.Cu, and In2.Cu all hit `/1V1`, `/3V3`, GND, CAN, XIN, or RF copper. This trial is rejected; its DRC report is `pa_en_via_bcu.json`, and `/usr/bin/python3 reproduce_pa_en_via_bcu.py` replays it.

Sensor interrupt routes were tested individually and as a pair using legal 0.20 mm traces and standard through-vias. A single `/ICM_INT1` detour from U10 pad2 `(54.05,46.6)` to U20 pad4 `(48.8375,83.75)` reduced opens to 15 but produced 12 violations; the source via/escape conflicts with `/ICM_INT2`, MAIN_LED_A, `/3V3_ADC`, VREG_AVDD, and SENS_MISO. The paired `/ICM_INT1` and `/LSM_INT1` route reduced opens to 14 but produced 37 violations. It used In2.Cu for `/ICM_INT1` and B.Cu for `/LSM_INT1`; occupied layer corridors and source escapes caused shorts and clearance failures. These routes are rejected. The archived report hashes are checked by `verify.py`, and `/usr/bin/python3 reproduce_sensor_interrupt_pair.py` replays the paired result.

| Target | Trial | Unconnected | DRC violations | Trial board SHA-256 | DRC report |
|---|---|---:|---:|---|---|
| `/ICM_INT1` | B.Cu detour | 15 | 12 | `d6659c1b5da22d6c4a512c4b56a8cca04d20da4dcbfbccd0857cfe2ecc9a5487` | `icm_int1_detour.json` |
| `/ICM_INT1`, `/LSM_INT1` | In2.Cu + B.Cu paired route | 14 | 37 | `5ee302183f7a91f55e919e512377ff90425fb0dd723d059b748199f371d6778c` | `sensor_interrupt_pair_in2_bcu.json` |

The long `/GNSS_RX` airwire was also routed from U10 pad12 via a standard-via pair and In2.Cu, keeping the adjacent `/GNSS_PPS` geometry fixed. It reduced opens to 15 but produced 41 violations; the In1.Cu alternative produced 48. The route crosses existing `/1V1`, GNSS UART, `/3V3_ADC`, `/SYS_5V`, and GND, and the U10 escape is constrained by `/GNSS_PPS`. These are rejected. The In2 replay is `/usr/bin/python3 reproduce_gnss_rx_in2.py`.

| Target | Trial | Unconnected | DRC violations | Trial board SHA-256 | DRC report |
|---|---|---:|---:|---|---|
| `/GNSS_RX` | In2.Cu via pair; preserve GNSS_PPS | 15 | 41 | `c35b44154ee949b54a5cc8942e6828f8348837102ed6ef5700568b75c0965982` | `gnss_rx_in2_keep_pps.json` |
| `/GNSS_RX` | In1.Cu via pair; preserve GNSS_PPS | 15 | 48 | `951b234ed4b3f3c16cfb4859229e8f03d355050393fc9e10f63ea3078d91fe1f` | `gnss_rx_in1_keep_pps.json` |

The `/RF_RESET_N` connection from U10 pad20 to the existing MCU-region track was tested with a 0.20 mm F.Cu breakout, a standard via at `(53.15,54.65)`, and a 0.20 mm B.Cu segment to `(84.0,64.29)`. It reduced opens to 15 but produced 13 violations. Nearby `/RF_nCS`, `/ICM_INT2`, `/ICM_nCS`, GND, `/RF_GPIO3`, XIN, and STATUS_LED routing blocks a clean source escape or B.Cu chord. The route is rejected; `/usr/bin/python3 reproduce_rf_reset_bcu.py` replays the DRC result.

The best legal-width `/CC1121_RF` probe used two 0.60/0.30 mm through-vias and a 0.35 mm F.Cu/B.Cu route from FL40 pad2 `(101.375,61.75)` to FL41 pad5 `(103.8,62.25)`. It closed one airwire but resulted in 8 total violations (the 4 baseline USB hole-clearance findings plus 4 new RF/GND conflicts): the F.Cu escape crosses `/CC_TXPA`, the source via shorts `/CC_LNA_N` on In2.Cu, and the destination via/track shorts GND. The direct 0.35 mm trial is rejected; `/usr/bin/python3 reproduce_cc1121_rf_35_bcu.py` replays it. This only checks nominal trace width against the `RF_50R` class; no stackup/solver evidence verifies 50 Ω impedance.

| Target | Trial | Unconnected | DRC violations | Trial board SHA-256 | DRC report |
|---|---|---:|---:|---|---|
| `/CC1121_RF` | 0.35 mm via pair + B.Cu trunk | 15 | 8 | `d8e67873b49e7dd87d8830711d7f7379616798571c1e4b624ab106e85dbe2237` | `cc1121_rf_35_bcu.json` |

The `/CC_RBIAS` route was tested at 0.35 mm although its project assignment remains Default (0.25 mm), to use a conservative width for the RF cluster. The best west-side F.Cu dogleg from U40 pad14 through `(94.35,64.1)` and `(94.35,66.6)` to R40 pad1 closed the airwire but produced 8 total violations. It shorts and crosses `/VDD_CC112`, has only 0.0374 mm clearance to the `/VDD_CC112` via, and bridges the mask to U40 pad13. R40's pad-to-pad gap is too narrow for the selected trace and clearance. It is rejected; `/usr/bin/python3 reproduce_cc_rbias_35_west.py` replays it.

| Target | Trial | Unconnected | DRC violations | Trial board SHA-256 | DRC report |
|---|---|---:|---:|---|---|
| `/CC_RBIAS` | 0.35 mm west F.Cu dogleg | 15 | 8 | `10605eaa837639c0bfc361509a1021e0494d31d7512752ccf5cb92ac0ba3ab2f` | `cc_rbias_35_west.json` |
