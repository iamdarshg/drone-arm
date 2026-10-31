# Rejected `/3V3_ADC` hand-route probe

This experiment was run against main commit `9f9d394d7d9ef0e8b8daeadaea4dfa45a3771b26` with KiCad 9.0.2. Both baseline and candidate boards were explicitly zone-filled with pcbnew 9.0.2 `ZONE_FILLER.Fill(board.Zones())`, saved, and checked with KiCad CLI DRC using the matching project and `--severity-all`.

- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching `main_rev_b.kicad_pro` SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Zone-filled baseline board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Baseline DRC JSON SHA-256: `5319d3b5995743d8ebd2226b5308715b8ca216d7695130f1f9b01971c00c7de5`; 7 violations, 45 unconnected items.
- Candidate route: 0.20 mm F.Cu from U10.59 `/3V3_ADC` at (63.95,46.60) directly to existing same-net F.Cu track at (50.8132,48.5257).
- Candidate board SHA-256: `6b378fbc40d5577b54d9c7057f120a72b8337d8a9597b24aeef6ce91c9a7d527`
- Candidate DRC JSON SHA-256: `fb5df1ebb08057e09eb0d9b550e634ed4d8a9b4d22ba130498a280c52328bd9b`; 12 violations, 45 unconnected items.

Rejected: the count did not fall, with new shorts to `/BARO_SCL` and `/3V3`, power-clearance and solder-mask bridge violations. No candidate PCB was promoted. Full JSON evidence is adjacent.
