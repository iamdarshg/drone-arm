# Controlled zone-refill DRC recheck for rejected Rev-B route probes

This follow-up normalizes the DRC procedure for earlier route experiments. Every source and candidate board was zone-filled with the same KiCad pcbnew 9.0.2 API (`ZONE_FILLER.Fill(board.Zones())`), saved, then run through KiCad CLI 9.0.2 DRC with the same matching `main_rev_b.kicad_pro` settings and `--severity-all`. The baseline was freshly zone-filled with this same procedure. The source board contents match the hash listed below across the earlier probe bases, even though documentation commits have advanced main.

- Recheck source commit: `303eb8e` (documentation-only; board bytes unchanged)
- Source board SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Zone-fill script SHA-256: `221f0995b7154e7f5e88c69c4b6e01002477a72f4e5873f11e50067866baf8a6`
- Zone-filled baseline file SHA-256: `a21b010eba158e7b3ecfeb0bfdb8c98de9c8d62212f9f7958ffbba0e6b945824` (serialization remained byte-identical)
- Zone-filled baseline DRC JSON SHA-256: `fd3b89d1e658148e616edf03a371c226593b088083691664ff5839d5f036a513`; 7 violations, 45 unconnected items.

| Candidate | Zone-filled candidate board SHA-256 | DRC JSON SHA-256 | Violations / unconnected | Result |
| --- | --- | --- | --- | --- |
| VREG_LX straight F.Cu | `d49724180cedbbc9b1f59ec8f187b111918433476a6af6ff86fd9aff832d43ce` | `4331084bb6af6add5aa20e2649f7de1e6093db0b486e9c0abfda3fe2f8c0fafe` | 11 / 45 | Rejected; added crossings and /VREG_LX–/VREG_AVDD short. Zone fill did not reveal a routable version. |
| VREG_LX two-via B.Cu escape | `6305a71d99a48bb87870e6a87aa4a20e7984074b5af063f7e810c00e2aa97be4` | `8e85dab305e5ada0c203cb13b55eb6f2251b5f0cf3da8a69be3c9bf4e80fc751` | 12 / 44 | Rejected; new crossings and short/clearance issues remain. |
| USB_DP straight F.Cu | `fefad464df6f49505654740f5e909e5425c963cf44365340661414f3a6c0a550` | `c1ba8a953b1cf3d74e5bbb439f97f24d42a801b2f2dbff8392beffaf071de9b9` | 14 / 44 | Rejected; new shorts/crossings/clearances remain. |
| RUN straight F.Cu | `88d5b1f96e2d563a2e5df0d1f881412ce47cd02391eac9a2e8111dc0f9d39ba2` | `c1f35f9faef10e476e20434b2b491b0bb5aa72d09f45da3e49900cd38a52e236` | 16 / 45 | Rejected; crossings and GND short remain; no connectivity reduction after refill. |
| VREG_AVDD straight F.Cu | `e0a321e109443bf2fcb1f500f22b22c43bdd2ea8e726de9daa7898b71b348797` | `8756df9bae4489ffad29847d9cdf9e08a2bd0a6534a3ac60bd5f9aef10f1273c` | 19 / 44 | Rejected; crossings and GND short remain. |
| XIN via escape | `7667d6f3da1e0da7d875f9a323235229e694c0aa01da2a94e04e8d828c195f5e` | `9e601c25c31a8a91eeb9e30e1286cd2b7fb52a5d3234cf9bfe22d6e47ddab170` | 21 / 44 | Rejected; crossings and shorts to /3V3 and /XOUT remain. |
| XOUT straight F.Cu | `49f777d0a9c1a8332a8cf8c3f3414c3a20a40e0e45ad6d070b3a1ea72f56f244` | `14edb49b9eba661bd9030f91c4cc9d180251249366b6644975033296e54e4e89` | 22 / 44 | Rejected; crossings and shorts to /1V1, /GNSS_PWR_EN and /3V3 remain. |
| BARO_SDA straight F.Cu | `74e75582e4db394d36d00b11533e1afe8216961fae77af1274e329fed0908f57` | `6be3b3fb7321916195b2aee311fce79060ae2596b4022eb594502e1cc2f79c35` | 28 / 45 | Rejected; new crossings, shorts to /3V3_SENS and GND, and clearance/mask errors; no connection reduction. |
| ICM_INT2 straight F.Cu | `29499be588870b0502681b0778d0dadaa5c5ef755796b4f499399a13da770bca` | `f1b4bbaa6e61895d76feb85814831cc754dbfe0a62fbf11ba111c352189765fc` | 38 / 44 | Rejected; new crossings and shorts to /1V1, /3V3 and /ICM_nCS remain. |

Earlier reports in this directory contain the original no-explicit-refill DRC runs. Those results are preliminary only; the table and attached JSONs above are the matched, zone-refilled disposition. No rejected candidate PCB was promoted.
