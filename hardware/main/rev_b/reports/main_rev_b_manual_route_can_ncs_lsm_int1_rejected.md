# Rev-B hand-route probes: CAN_nCS and LSM_INT1 (rejected)

These were temporary candidate boards only. Neither candidate was accepted or added to the Rev-B PCB.

Source repository revision: `bce94d9` (`docs: pin rev b manual route candidate evidence`), including accepted CAN_INT route `6b9dba0`. Source board SHA-256: `db8fa1ca44907ad5b70b7010bdb67fe06d80dcd6715aea2a02662682ddb47291`.

The baseline used KiCad CLI 9.0.2 with a matching-stem `main_rev_b.kicad_pro` and zone refill. It reported 7 violations and 55 unconnected items; report SHA-256: `0484d21eefb48749a57a3162a0ffa71a5b66105a838a68ce3d92acf37ac904c5`.

| Candidate | Temporary board SHA-256 | DRC result and report SHA-256 | Decision |
|---|---|---|---|
| `/CAN_nCS`: U10.38 breakout to (61.8,58.5), B.Cu detour, destination via at (35.8413,18.7) on existing U51.13 trace | `eb9f5bda9160706a2901944116f752f5d9a6832fa59cd027f141976f60108057` | 27 violations, 54 unconnected; `8ef679a0c10039e42fccd09728a3e51936a3fe1a841819272c14c1287ca4f135` | Rejected: new path shorts/crosses accepted `/CAN_INT` copper at multiple points and conflicts with `/1V1`, `/3V3`, `/SENS_SCLK`, and GND near the U10 escape. |
| `/LSM_INT1`: U10.4 breakout to (52.8,47.4), B.Cu path via (50.8,49.4), (50.8,74), (51.8,76), (55.5,79.7), (57,81.2), (58.8,83), ending at a via at (58.8,85.2) connected to U21.4 | `060e28b2816b1fc7d829f653646d3aaa82b61a20e2018e435a8baca16a85a2dd` | 17 violations, 54 unconnected; `9ebcead66a7eed6a14ae5d88c28ed37f0cc9323c88de324ad648f2bcea095dd0` | Rejected: new route shorts/clears C11 GND, `/3V3_ADC`, `/SYS_5V`, `/SENS_MISO`, and sensor-power copper. |
| `/RP2354B MCU/RUN`: R12.2 breakout via at (49.86,61.0), B.Cu path to U10.35 via at (61.8,58.5), with an F.Cu south/east MCU escape | `72e9135fdbb37210ec03fa68ee631a49dffbabb53e24f48d4c2f5c5e433654f8` | 11 violations, 54 unconnected; `9712b8cde41b31cfd8bec48d23385b27343eb563eed8e9284910e3658b48ff95` | Rejected: U10 escape crosses and shorts the accepted `/CAN_INT` route; new via also violates clearance to `/RP2354B MCU/STATUS_LED`. |

All three routes reduced the unconnected count by one but introduced DRC violations. The source PCB remains unchanged at 55 unconnected items and 7 existing violations; it is not fabrication-ready.

## Follow-up probes from `bee8715`

The source main PCB is still byte-for-byte the same board (SHA-256 above), with matching-project baseline DRC at 7 violations and 55 unconnected items. Two further `/RP2354B MCU/USB_DP_MCU` alternatives were tested as temporary candidates:

| Candidate | Temporary board SHA-256 | DRC result and report SHA-256 | Decision |
|---|---|---|---|
| Direct F.Cu segment R10.2 (56.53,37.1) to U10.67 (60.4,45.05) | `16a6684dad908107682f8b85ac91f71ca4d246f56682bc7da52d62cf778a3298` | 10 violations, 54 unconnected; `87eb22ddbe8d06be8d66a3407ef7831f68a2a1fdf2df98cfd701ae71aafae89f` | Rejected: trace shorts the existing `/3V3` track and C32 power pad. |
| B.Cu route using vias at (56.53,38.2) and (60.4,43.5) | `27f4df0c75755f67a149ff934fe3cc04fa2453468229a1b154560d0d73f75a97` | 16 violations, 55 unconnected; `2b1f945b4985373fd00867b97537ec20f6508c50f111f8bf094f82f28bdf3e7d` | Rejected: U10-side via and escape short C32's GND pad/via and violate drill clearance. |

Neither USB_DP candidate met the no-new-DRC gate. The paired USB_DM connection remains unrouted, so no USB differential-pair completion is claimed. The accepted source PCB remains unchanged.

## Follow-up probe from `980f77c`: sensor SPI

The source board still has SHA-256 `db8fa1ca44907ad5b70b7010bdb67fe06d80dcd6715aea2a02662682ddb47291` and baseline DRC 7 violations / 55 unconnected items. A temporary `/SENS_MOSI` route from U10.78 via (56,42) to the existing sensor-bus via at (58.8456,81.5875), using B.Cu, produced board SHA-256 `24732c7d1d467c379ba11a6065813ae8afacdc30c9afff8337fcb31369410d90` and DRC JSON SHA-256 `0f70069dfea3b0a948a961e7cc8e64cc0d7fdd7603bcb4c3cac2bff181bc6a85`. Matching-project KiCad DRC reported 12 violations and 54 unconnected items. Rejected: the route crosses `/CAN_INT` and `/SYS_5V` on B.Cu and violates clearance at nearby `/3V3` and GND vias.

## Follow-up probe from `7381d8c`: LSM_INT2 sensor interrupt

With the matching project and zone refill, the source-board baseline reports 7 violations / 55 unconnected items (DRC JSON SHA-256 `1e54259d5cf1c3cb004e0ee558754d8331bd830cfccee412a77cc245d825be2d`). The temporary `/LSM_INT2` candidate from U10.6 via an outer escape and a B.Cu detour to U21.9 has board SHA-256 `6472a2a398b634b19e1a7573afe709a4142839025fa0620cc16450372c867150`; its DRC JSON SHA-256 is `049d913136f09c57907296b4d392ce3b2e3e02132343b4eedd60d48ae5f4763f`. It reports 20 violations / 55 unconnected items, so it does not reduce the unrouted count. Rejected: it shorts C11 GND, `/SYS_5V`, and `/SENS_MOSI`; it also crosses `/1V1` and violates clearances to `/3V3_ADC` and GND vias.
