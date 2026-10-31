# Rev-B manual route: BARO_SCL edge escape

Added a legal package-edge breakout and In2.Cu route for `/BARO_SCL`:

- Source: U10 pad 7 at (54.050, 48.600 mm)
- F.Cu escape: 0.20 mm wide, 1.050 mm from the pad to a via outside the package at (53.000, 48.600 mm)
- New via: through via, 0.60 mm diameter, 0.30 mm drill
- In2.Cu route: 0.20 mm wide; waypoints (54.800,50.400), (54.800,57.200), (64.600,67.000), (64.600,77.000), (66.800,79.200), (66.800,80.800), (68.7126,82.7126) mm; terminates at the existing `/BARO_SCL` via (68.7126,83.7749 mm)
- Parent revision: `cd8a797`; parent board SHA-256: `590efd4db73facf1f743e57832c707f5a688532e790f00a1a0f9e95c11fb15f2`
- Result board SHA-256: `a2690937d50b35190720988a20ec890c3bdf230c3e4b6c68d7fff5a461043c74`

The native board model confirms the source pad, new via, In2.Cu segments, and destination via are all assigned `/BARO_SCL`. KiCad CLI 9.0.2 DRC with the matching Rev-B project file reported 7 violations before and after with identical signatures; unconnected items fell from 57 to 56. The board still has 56 unrouted connections and 7 existing violations, and is not fabrication-ready.

DRC report SHA-256 values: parent `30fd195649daf616fd2001bcc481e7c2313962f41a2fb7dc15cf17b1b23aaa12`; candidate `ab9a83393b0fa8e54b546c3f4ac4dbdf429c9d9b9b4f8e8723a4928a0b2c1f46`.
