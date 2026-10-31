# Rejected `/915 MHz RF/CC_TRX_SW` route probe

Checked against source commit `bb14a26a6dcaef2ddd89583d50d25f01a800e625` using KiCad 9.0.2. Baseline and candidate were explicitly zone-filled with pcbnew `ZONE_FILLER.Fill(board.Zones())`; CLI DRC used `--severity-all` and the matching project.

- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `08fe932dbd12aaa0a949249f77f3afc92dc3af3db00ee4a22a8494e1107f4e00`; 7 violations, 45 unconnected items.
- Candidate: 0.20 mm F.Cu straight route from U40.18 `(96.4375,62.25)` to FL40.8 `(102.625,61.75)` on `/915 MHz RF/CC_TRX_SW`.
- Candidate board SHA-256: `520c9f63c612be69b4a3e064306d9e5620fbe2ed25c2d8c0a1e26687baba82bd`
- Candidate DRC JSON SHA-256: `766f9023195013c61e2aff57f6d06dc04c561d6e6dbab85c79c234591f9ec66c`; 13 violations, 44 unconnected items.

Rejected: new shorts to `/915 MHz RF/CC1121_RF` and `/915 MHz RF/CC_TXPA`, a crossing, and a solder-mask bridge. No candidate PCB was promoted; DRC says nothing about controlled impedance or RF performance.
