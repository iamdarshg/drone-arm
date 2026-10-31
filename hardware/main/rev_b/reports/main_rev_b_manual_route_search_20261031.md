# Rev-B hand-route search log — 2026-10-31

All candidates used KiCad CLI 9.0.2 with the same-stem `main_rev_b.kicad_pro`. The baseline board is SHA-256 `a2690937d50b35190720988a20ec890c3bdf230c3e4b6c68d7fff5a461043c74`; the baseline DRC JSON is retained as `main_rev_b_manual_route_can_int_parent_drc.json` (SHA-256 `50d5c7f114131dbf497d692b2d6d7a814e433dc5850e3a4caff4bb643fff735b`). Baseline: 7 violations, 56 unconnected items.

Rejected candidates remained in temporary directories and were not copied to the board. Their board hashes and complete project-aware DRC JSON reports are recorded here for audit. Candidate DRC reports are copied into `manual_route_search_20261031/`.

| Candidate | Candidate board SHA-256 | DRC SHA-256 | Violations / unconnected | Outcome |
|---|---|---|---:|---|
| BARO_SDA, F.Cu breakout + In2 detour | `d701222c29fb290e1615861e9c26203211e25ee87acdf4d48593604140eaace5` | `be757c6c3f5be56bde25d93206b10af4a860d45a9056ae550710a2618f231856` | 32 / 55 | Rejected: short/clearance conflicts with BARO_SCL, VREG_AVDD and sensor GND pad/zone. |
| BARO_SDA, F.Cu breakout + B.Cu detour | `0c1747a6209881358e72b68d11aa2250085b3c209a34f5863723c49116cbc1c6` | `1668ae5d3ef05720f88e754787dc20f744439b85cc2717cea6ebba0f3ac35ac6` | 39 / 55 | Rejected: overlaps BARO_SCL, SYS_5V and GND copper/vias. |
| SENS_MISO, direct B.Cu route | `4f032c1056146cc21fb9963a17f92889dea3716e495b8315ed02de2b1b59a33e` | `9521758e2fc906c443e7300908ba14a929ff7835af3e492be9edf74f59a3286d` | 23 / 55 | Rejected: B.Cu conflicts/clearances in the sensor and MCU corridor. |
| GNSS_RESET_N, direct B.Cu route | `346c2b1f8b399bdd5c2ea95fb4287859450c23f58570be0b7556a9ee749e77a7` | `a46be1da05f1ace7e29fde99fcf4d7920b4ea4b8bccf98c57293ca5604c6b996` | 29 / 55 | Rejected: short to 3V3 at MCU breakout and GND via on B.Cu. |
| CAN_INT, direct B.Cu route | `5a2e7131ddeeaaaf4849001c7b9266eb186775d9545e228a6d11268cc4fb5227` | `68bfbd10f1a4af3f826ae0a286b2fbbac5f77ba0c64ad1165f9968e340a57303` | 20 / 55 | Rejected: source via conflict and 3V3 via crossing. |
| CAN_INT, detour 1 | `34b7df89d9e40e1f790b06d4c146f5e723c726e3f7763d19415d019d5e541add` | `6d7bb15bb2fedfee26381ea178dcf14f498527169f062b03f95e59485982fb88` | 17 / 55 | Rejected: remaining B.Cu crossings/clearances around 3V3 via. |
| CAN_INT, detour 2 | `fb4dec6de59640644fb4276b802449b68355bcb71a12626326c1f655990705df` | `70dc2cf5fe41b1dca23624d38b092f1e83b1bc4c44e130bfd2a83445a7c0acf1` | 14 / 55 | Rejected: short/clearances at MCU breakout and GND vias. |
| CAN_INT, detour 3 | `5723f85a3853f11e149c84fe0da7b5a05f6161d58e82bcfc09bd824e97478162` | `2837f878338a83d5c9c4765bddf6fdee3804ea62fd822d1332e026dac9efa1bd` | 10 / 55 | Rejected: 3 remaining clearance/short issues. |
| CAN_INT, detour 4 | `5a5debff55a979995cfdb2957174b20bbca6ac289c9c6d2d4a822edc3c8cc015` | `9d208d5d581577ec6db6704a69f35dd3246a3714e023ef7f7aa3e72914decf40` | 9 / 55 | Rejected: two remaining clearance issues. |
| CAN_INT, detour 5 | `8272f78bd285130499c92a915db78a66504e668baafcd4b91c1743e2e0b448d8` | `139664053778896a5fb1be02f9f9864ad513942ed06eb3a7d82b8be6c7743485` | 8 / 55 | Rejected: via shorted the STATUS_LED track. |
| **CAN_INT, accepted final route** | `db8fa1ca44907ad5b70b7010bdb67fe06d80dcd6715aea2a02662682ddb47291` | `1529a6fc5673dc30eea612d67ed906133572ad780acfdfe7aa2f0216ed32d04c` | **7 / 55** | **Accepted and pushed. All seven violation signatures match baseline; unconnected count falls by one.** |

The accepted route is documented in `main_rev_b_manual_route_can_int_bcu.md`. The accepted board was independently rechecked with matching-project DRC after promotion: 7 violations and 55 unconnected items; signature comparison against baseline returned identical. The board is still not fabrication-ready.
