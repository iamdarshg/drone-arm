# Rejected routes checked against latest main `ef7bdbe`

Both candidates use the exact latest main board at source commit `ef7bdbe86f63b4bbfa7bbc80d64e791eeaa2627a`. Baseline and candidates were all explicitly zone-filled with pcbnew 9.0.2 `ZONE_FILLER.Fill(board.Zones())`, then checked with KiCad CLI 9.0.2 `--severity-all` using the matching project settings. Baseline: 7 violations, 45 unconnected items.

- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `59fb44afea1e62d2ee20df68288709861ba3d6692465b25b57fc73eb391ab8a9`
- Zone-fill script SHA-256: `221f0995b7154e7f5e88c69c4b6e01002477a72f4e5873f11e50067866baf8a6`

| Candidate | Candidate board SHA-256 | Candidate DRC JSON SHA-256 | Result |
| --- | --- | --- | --- |
| `/1V1`, U10.51 outward F.Cu escape and via at (66.3,47.9), B.Cu detour to existing `/1V1` via at (53.2,50.4) | `008a5953d5b7e1e25eb6870414e4ffd637b59280a0f6566aa913d71d70dddf79` | `1ab30975f4e5e17ecbd22b3aad2609fcd075927512b1d0780068658858ecd268` | 12 violations, 44 unconnected. Rejected: four new power clearance violations and one B.Cu crossing with `/CAN_INT`; no shorts remained on this revised candidate. |
| `/3V3_ADC`, U10.59 straight F.Cu to existing same-net track at (50.8132,48.5257) | `73985e5199d40323f00f85d21103ae9d1fb76bd9d5201cfe855373b4fee90fe6` | `ba712837fcd4134db142e4ae2900bea5fea9ff56a7163db991393cf18646f700` | 12 violations, 45 unconnected. Rejected: new shorts to `/BARO_SCL` and `/3V3`, clearance and solder-mask violations; no connectivity reduction. |

The complete before/after DRC JSON files accompany this note. Neither PCB candidate was promoted.
