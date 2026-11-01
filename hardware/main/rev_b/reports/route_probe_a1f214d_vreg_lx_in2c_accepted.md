# Accepted `/VREG_LX` In2.Cu route from `a1f214d`

This route connects U10.63 `/VREG_LX` to L1.1 on the board at source commit `a1f214d26921eba633dce8dbe3c8767fe787a2e0`. Source board SHA-256: `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee`. Matching `main_rev_b.kicad_pro` SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The 0.20 mm route leaves U10.63 `(62.0,45.05)` on F.Cu to a 0.60/0.30 mm through-via at `(62.0,47.2)`, crosses In2.Cu to a second 0.60/0.30 mm through-via at `(67.9,42.5)`, then returns to F.Cu and joins L1.1 `(66.9375,42.5)`. This south escape keeps the first via clear of the existing `/RP2354B MCU/XIN` In1.Cu track.

Both source and candidate were explicitly zone-refilled with pcbnew 9.0.2 `ZONE_FILLER.Fill(board.Zones())`, then checked with KiCad CLI 9.0.2 `pcb drc --severity-all --format json` using the matching project. The baseline reports **7 violations / 44 unconnected**; the candidate reports **7 violations / 43 unconnected**. All seven DRC signatures are identical. Baseline JSON SHA-256: `a363c30e743eab778ef356c7668a98ce850800b12b292d4808e97c7dd0db7450`; candidate JSON SHA-256: `6f309fd5ad9cb757e8880582edb591435673ef871ffbf2060d1451ec6f0fa94e`.

The accepted zone-refilled board SHA-256 is `2bcb4d6ba0293d280306393d070ccc16a0b6755d0d34b77cf89aefac7d4e638a`. The final DRC was rerun on the committed board file with its matching project; it reports the same counts and signatures. The route removes one unconnected item and adds no DRC signature. Remaining board DRC findings and unconnected items remain; this route does not establish fabrication readiness or electrical qualification.
