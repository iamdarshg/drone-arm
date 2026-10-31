# Rejected `/BARO_DRDY` B.Cu route probe

This temporary probe attempted the low-speed barometer data-ready line from U10 pad 9 to U22 pad 7. It was not promoted to the PCB.

Source board revision: `303eb8e`. Later commit `9f9d394` adds route-probe documentation only and leaves the board bytes unchanged. Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`. Project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The candidate used 0.60 mm / 0.30 mm through-vias at (52.200, 49.400) mm by U10 and (71.500, 82.750) mm by U22. It connected U10.9 to the first via and the second via to U22.7 with 0.20 mm F.Cu segments, then joined the vias with a straight 0.20 mm B.Cu trace. Candidate board SHA-256: `42c8327f0a119d9f27f214c595b748f83769c29907d4563cd53c3d0813f17ab1`.

KiCad CLI 9.0.2 with the matching project reported 16 violations and 44 unconnected items on the candidate, versus the zone-refilled baseline of 7 violations and 45 unconnected items. Although one airwire was removed, the candidate adds nine DRC signatures: five shorts, a hole-clearance violation at the `/3V3_ADC` pad, a POWER clearance violation near `/1V1`, a B.Cu crossing with `/SYS_5V`, and a solder-mask bridge. The shorts involve `/1V1`, `/3V3_ADC`, `/VREG_AVDD`, and GND. The exact full candidate signatures are retained in `route_probe_303eb8e_baro_drdy_direct_bcu.json`.

- Zone-refilled source DRC SHA-256: `fd3b89d1e658148e616edf03a371c226593b088083691664ff5839d5f036a513` (`route_probe_zfill_303eb8e_before.json`).
- Candidate DRC SHA-256: `59564c26b948bb16bf43fc6904d32e97f685bd1445f5d5728c82d081b4758a15` (`route_probe_303eb8e_baro_drdy_direct_bcu.json`).

The source PCB remains unchanged. The candidate is rejected because it damages the surrounding power/ground routing and violates the no-new-DRC gate.
