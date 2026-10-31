# Rejected `/915 MHz RF/CC_LNA_P` route probe

Source commit `bd9f8966f0b2ee2312226c7d92385e274afa8996` was checked with KiCad pcbnew and CLI 9.0.2. Baseline and candidate were explicitly zone-filled with `ZONE_FILLER.Fill(board.Zones())`; CLI DRC used `--severity-all` and the matching project file.

- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `ff0f52414a4113eed39b8facc14b79fe9e2270677266f2fd228d14a4bcfb6f58`; 7 violations, 45 unconnected items.
- Candidate: 0.20 mm F.Cu straight route from U40.19 `(96.4375,61.75)` to FL40.7 `(102.625,62.25)` on `/915 MHz RF/CC_LNA_P`.
- Candidate board SHA-256: `4c4072244834a4af506d6a8d565c0f186fc61b261023714b90ef6c5fda96eab8`
- Candidate DRC JSON SHA-256: `acb236b207bd0024cde9f94b2814d7e11c86956291e2ae2fc5b2d5bf26117559`; 12 violations, 44 unconnected items.

Rejected due new shorts to `/915 MHz RF/CC1121_RF`, `FL40` NC, and `/915 MHz RF/CC_TXPA`, a track crossing, and solder-mask bridge. The route was not promoted. This DRC result does not establish controlled impedance or RF performance.
