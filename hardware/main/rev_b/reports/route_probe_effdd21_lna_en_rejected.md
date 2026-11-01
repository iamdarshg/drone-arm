# Rejected `/LNA_EN` MCU-to-RF via probe

Tested from source commit `effdd21` on a disposable board copy. PCB hash remained `095aa3fd44285b885c2a4ca68c05633b169dd053fbc8407d2160e8585e73336b`; matching project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. Source and candidate were explicitly zone-refilled using `pcbnew.ZONE_FILLER.Fill(board.Zones())`, then checked with KiCad CLI 9.0.2 `pcb drc --severity-all --format json` using the matching project. Baseline: 7 violations / 42 unconnected, JSON SHA-256 `9a2f05ebd71ef3e1162e7a39c2976db1226d37ba0f4b5f5da08d170932beda64`.

Candidate board SHA-256: `4068d2f533daa8af8f397230c1f2bd596d0b3a2f026f3c1b4ec8d5e78864b0d2`. It escapes U10.26 `(57.2,54.95)` to a 0.60/0.30 mm through-via at `(57.2,56.2)`, escapes the existing `/LNA_EN` track endpoint `(110.8204,58.1896)` to a through-via at `(112.0,58.2)`, and joins them with a 0.20 mm B.Cu span. Candidate JSON: [`route_probe_effdd21_lna_en_candidate.json`](route_probe_effdd21_lna_en_candidate.json), SHA-256 `b2f16de30770690da9e24b2d15a46efcea681c4a260f7f221402f12ff3d042cb`.

The candidate reduces unconnected items to 41, but adds numerous DRC signatures: shorts to `/1V1`, `/3V3`, GND, `/915 MHz RF/CC_EXTCLK`, and RF supply; crossings, clearances, solder-mask and hole-clearance errors. It is rejected. No board or design expectation was changed.
