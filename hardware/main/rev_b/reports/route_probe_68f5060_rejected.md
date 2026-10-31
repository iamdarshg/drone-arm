# Rev-B manual route probes on `68f5060`

These are rejected, report-only routing experiments. The board source remains unchanged by these probes. Each candidate was created from the source board at commit `68f5060` using KiCad `pcbnew` 9.0.2 and checked with KiCad CLI 9.0.2 using the matching `main_rev_b.kicad_pro`; none passes the acceptance gate (no new DRC signatures and fewer unconnected items).

## Source pinning

- Source commit: `68f5060ae7863ff2992dd648783f1c8c69d9542b`
- Source board: `hardware/main/rev_b/main_rev_b.kicad_pcb`, SHA-256 `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project: `hardware/main/rev_b/main_rev_b.kicad_pro`, SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Tool: KiCad CLI / pcbnew `9.0.2`
- Baseline DRC JSON `route_probe_68f5060_before.json`, SHA-256 `cb5cb2a4204d94eba4741c3aab7a10ec04a742e0b237003dfdd0a961a4bb04a5`; 7 violations, 45 unconnected items.

| Probe | Candidate board SHA-256 | DRC JSON | Result |
| --- | --- | --- | --- |
| `/VREG_LX`, straight F.Cu U10.63→L1.1 | `c46928bf9b0c05020e63a2a4f9d9c66766ef6205afa9704bc25b710a0e0a7be7` | `route_probe_68f5060_vreg_lx_straight.json` (`31d12ee17dd56d5ec1534afd5bfb9e8152bd3d27b8e95d5f5c0001d550223fb9`) | 12 violations, 44 unconnected. Rejected: new `/VREG_LX`–`/VREG_AVDD` short, track crossing, clearance and mask-bridge violations. |
| `/VREG_LX`, F.Cu escape and two-via B.Cu detour | `2f60d9c659293bdc077a838a71c651ad1ea957bcdeaf8312ebeaccc225035d3c` | `route_probe_68f5060_vreg_lx_via_escape.json` (`055cbee0d19449e2e622f67bd45029befa6acfbd8632f9a4d7424336119c7111`) | 26 violations, 44 unconnected. Rejected: new track crossings, `/VREG_LX`–`/3V3` shorts, clearance/hole-clearance/zone and solder-mask violations. |
| `/RP2354B MCU/USB_DP_MCU`, straight F.Cu U10.67→R10.2 | `5ec43a11227014031e51eb15fb8c57a53279d3a35aee8bc6ac4cdf13f0b5b2b1` | `route_probe_68f5060_usb_dp_straight.json` (`4e4699f1efd8592136482dad305fb58a339006ae6fbfe4f4003842b6ab0d278e`) | 15 violations, 44 unconnected. Rejected: new shorts to `/3V3`, crossing, zone clearance, and solder-mask-bridge violations. |

Only DRC JSON reports and this source-pinned record are retained. No rejected candidate PCB was promoted.
