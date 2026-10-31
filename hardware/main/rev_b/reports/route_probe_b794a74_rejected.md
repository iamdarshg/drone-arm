# Rev-B manual route probes on `b794a74`

Report-only rejected candidates. KiCad pcbnew/CLI 9.0.2. Each candidate uses the exact main board and project context at this commit. Baseline: 7 violations and 45 unconnected items. Both candidates reduce unconnected items by one but add DRC violations, so neither is suitable to promote.

- Source commit: `b794a745f9969771ebe576138cda65bc8f2d623f`
- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline DRC JSON SHA-256: `b5e3ba6ef15db3d6d76b661fd71001d9692914307aa8d8191abe80e5cbef5e76`

| Probe | Candidate board SHA-256 | DRC JSON SHA-256 | Result |
| --- | --- | --- | --- |
| `/RP2354B MCU/RUN` straight F.Cu U10.35→R12.2 | `55da4ab68988cab8ac91429277f7acbe9504f3340502e6bf78f2345258f3c0a6` | `92a7de3e4cc7dc1e19993752643581cd55907296c4d97dd16586046fdaec4619` | 17 violations, 44 unconnected. Added crossings, power clearance, a GND short, zone clearance and mask bridges. |
| `/VREG_AVDD` straight F.Cu U10.61→existing track at (51.51,41.50) | `d77cc178848da958280555728dfc39cbb3a9939e9c7ba63b23197da01f7b3c50` | `c903dfe924268d75fe05c43928877b98522ca30dd9eb519acfd0323b9df03b6b` | 20 violations, 44 unconnected. Added crossings, power clearances, GND short, zone clearance and mask bridges. |

Only source-pinned JSON evidence is retained. No candidate PCB was promoted.
