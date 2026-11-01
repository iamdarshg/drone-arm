# GND stitch via at R50 pad 2: reject

This source-pinned audit evaluates one temporary `/GND` through-via placement. It is not part of the routed board and must not be promoted to the fabrication layout.

## Pinned inputs and tools

- Source revision: `b72c327` (`docs: bound GND via clearance search`).
- Source board, `hardware/main/rev_b/main_rev_b.kicad_pcb`: SHA-256 `095aa3fd44285b885c2a4ca68c05633b169dd053fbc8407d2160e8585e73336b`.
- Matching project, `hardware/main/rev_b/main_rev_b.kicad_pro`: SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.
- Temporary candidate board SHA-256: `966611a9c8caa4b6bc4974038a4666de791177b1ed7cb81f35518b42c918792b`.
- KiCad CLI and `pcbnew`: 9.0.2 (`9.0.2+dfsg-1`). Both boards were explicitly zone-refilled with `pcbnew.ZONE_FILLER.Fill(board.Zones())`; re-saving preserved each board hash.
- Matching-project DRC command for each copied board: `kicad-cli pcb drc --severity-all --format json --output <result.json> <main_rev_b.kicad_pcb>`.

The source and candidate DRC JSON are retained beside this report as `route_probe_gnd_via_r50_pad_source_drc.json` and `route_probe_gnd_via_r50_pad_candidate_drc.json`. Their SHA-256 values are respectively `e1fb9fc19a819c94cfaf8af0f30edec17d40f9f5f0430d265ad0aca0cf908cda` and `685bf0b70e2e832bbc62e867132a2f339443ce604ba3037ef4a5ed40b5ea6140`.

## DRC result

The explicitly refilled source reports 7 violations and 42 unconnected items. The candidate reports 7 violations and 41 unconnected items. Comparing the complete violation signatures (type, description, item descriptions, and coordinates) gives the same SHA-256 signature digest for both: `438757221a0ce8be318f24a5683ebdb531e6be562744225d32e33ebd5fdda1fb`. There are zero new or removed violation signatures. The candidate therefore removes one airwire while passing the project's current DRC checks.

## Placement review and decision

The candidate adds a 0.6 mm copper / 0.3 mm drill `/GND` through-via at `(111.836329, 65.720839)` mm. It overlaps the copper region of R50 pad 2 (`/GND`, center `(111.5, 65.49)` mm, 0.54 × 0.64 mm pad); the via center is only 0.016 mm beyond the pad's right copper edge, so the via annulus extends approximately 0.284 mm into the pad's copper envelope. The via also lies inside the R50 F.Courtyard rectangle (approximately x=111.03–111.97 mm, y=65.07–66.93 mm). The same-net overlap explains why DRC reports no new clearance violation. DRC acceptance does not establish solderability: no filled/capped-via specification or assembly-process evidence is available to rule out solder wicking or an assembly obstruction.

**Decision: reject and do not promote.** Keep the production PCB unchanged. The one-airwire improvement does not justify placing an unqualified open via in the component pad/courtyard area. This is a routing and assembly review only; it does not establish board fabrication readiness.
