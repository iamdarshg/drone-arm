# Rev B manual route draft

Source board: `main_rev_b.kicad_pcb` at `1979c9884be8491186d7f1f65319340081ab4d1a`.

Added two 0.20 mm F.Cu traces on `/3V3`:

- U10 pin 69 (`QSPI_IOVDD`, 59.600, 45.050 mm) to U10 pin 68 (`USB_OTP_VDD`, 60.000, 45.050 mm), length 0.400 mm.
- U10 pin 69 to C32 pad 1 (59.520, 43.500 mm), length 1.552 mm.

The schematic netlist places both MCU pins and C32 pad 1 on `/3V3`. The route follows the local pad geometry on F.Cu. The two MCU pads are adjacent, and the C32 link exits pin 69 northward; the zone was refilled before final DRC.

KiCad CLI DRC with the board project rules:

- Before: 7 violations, 70 unconnected items.
- After: 7 violations, 68 unconnected items.
- The existing violations remain four J1 hole-clearance findings, two silk-to-edge findings, and one silk overlap. No new violations were introduced.

Schematic parity reported 191 issues both before and after. Those are pre-existing and unchanged. This draft only routes two local `/3V3` connections; it does not establish fabrication readiness.
