# Rejected hand-route probes from `60126cd`

These three disposable routing probes were made from the same unmodified Rev-B source board. None was promoted. The board is still incomplete and not fabrication-ready.

## Pinned inputs and procedure

- Source commit: `60126cd023d9476ebe59372ba0dc196097e0e5e6`
- Source board: `hardware/main/rev_b/main_rev_b.kicad_pcb`, SHA-256 `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project: `hardware/main/rev_b/main_rev_b.kicad_pro`, SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline, explicitly zone-refilled via KiCad `pcbnew.ZONE_FILLER.Fill(board.Zones())`, then checked with `kicad-cli pcb drc --severity-all --format json` (KiCad CLI 9.0.2): 7 violations, 45 unconnected items.
- Baseline DRC JSON: [`route_probe_bd9f896_before.json`](route_probe_bd9f896_before.json), SHA-256 `ff0f52414a4113eed39b8facc14b79fe9e2270677266f2fd228d14a4bcfb6f58`.
- Each candidate was built from a fresh copy of the pinned source, had zones refilled, and was checked with the same CLI invocation and matching project. The candidate JSON files preserve the full DRC output.

## Attempts

| Net / geometry | Candidate board SHA-256 | Candidate DRC JSON SHA-256 | DRC / unconnected | Result |
|---|---|---|---:|---|
| `/CAN_SCK`, straight 0.20 mm F.Cu from U51.10 `(33.5,20.65)` to U10.28 `(58.0,54.95)` | `228e5ce3eb6398ab7038861ab108c55a91b039409c01417d01740805cf7ff57f` | `dfbe1fb26edadd2184cd1bc1625c8b1c09167228df297d947b2138e05002c883` | 24 / 44 | Rejected: 17 new DRC signatures, including shorts to HGM, BARO_SCL and GND, track crossings, a clearance error and mask bridges. Full result: [`route_probe_60126cd_can_sck_fcu.json`](route_probe_60126cd_can_sck_fcu.json); baseline-diff signatures: [`route_probe_60126cd_can_sck_fcu_signature_diff.json`](route_probe_60126cd_can_sck_fcu_signature_diff.json), SHA-256 `97b672225b0f00ab57597d50734f7f8a37900dceaedd022a6a4737100830ebfe`. |
| `/CAN_SCK`, 0.20 mm F.Cu escapes to 0.60/0.30 mm through-vias at `(32.4,20.65)` and `(58,56.2)`, then a straight 0.20 mm B.Cu span | `fc6babf2ab1c00da04e285812c9adf6267ea27498eaf7bbd9e8533754ed04cc0` | `94672aff9fb1535652d24bb0ab74d2d0f7cacd53833838497f214291ebb56d29` | 25 / 44 | Rejected: 18 new DRC signatures, including shorts/crossings/clearances involving 1V1, BARO_SCL, GND, MCU XIN, and hole clearances at the selected via locations. Full result: [`route_probe_60126cd_can_sck_bcu.json`](route_probe_60126cd_can_sck_bcu.json); baseline-diff signatures: [`route_probe_60126cd_can_sck_bcu_signature_diff.json`](route_probe_60126cd_can_sck_bcu_signature_diff.json), SHA-256 `0e4c05de6f6c9f4d1652e6f43b5b4e3ba7b8090adee5e0beddd603d363884675`. |
| `/LSM_INT1`, straight 0.20 mm F.Cu from U10.4 `(54.05,47.4)` to U21.4 `(58.8375,83.75)` | `303f73e9e550895a82f04679bbb9c56ac6428d9cfbb76165ae7964580101ddd4` | `88252f864bd9649e83f430c03bfbcbd9581c295d61e06531385a479bea7f64b1` | 33 / 44 | Rejected: 26 new DRC signatures, including shorts/crossings to 3V3, BARO_SCL, 3V3_SENS, SENS_MOSI and MCU signals plus clearance and mask-bridge violations. Full result: [`route_probe_60126cd_lsm_int1_fcu.json`](route_probe_60126cd_lsm_int1_fcu.json); baseline-diff signatures: [`route_probe_60126cd_lsm_int1_fcu_signature_diff.json`](route_probe_60126cd_lsm_int1_fcu_signature_diff.json), SHA-256 `5199844899f9f4ed94630146c47c19ff3377293e656da7f7305ca98e25916351`. |

## Independent route check

The candidate boards were reopened with KiCad's Python `pcbnew` API after save and zone refill. It confirmed the `/CAN_SCK` F.Cu track endpoints and net assignment; for the via attempt it confirmed the 0.20 mm B.Cu span, the two through-vias, and the F.Cu escapes. It confirmed the `/LSM_INT1` F.Cu segment endpoints and net assignment. The full-project DRC results independently show each segment reduced the unconnected count by one, but every candidate adds errors over the unchanged seven-signature baseline. These were intentionally rejected; no board or netlist expectation was modified.
