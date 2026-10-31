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
