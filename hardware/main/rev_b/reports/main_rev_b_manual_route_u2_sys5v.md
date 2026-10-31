# Rev-B manual route: U2 SYS_5V input branch

Added a via transition and B.Cu link to connect the existing U2 SYS_5V input trace to the SYS_5V through-via at U2 pad 4:

- Net: `/SYS_5V`
- Start: existing F.Cu trace endpoint at (27.8625, 73.600 mm)
- New via: through via at (27.8625, 73.600 mm), 0.60 mm diameter, 0.30 mm drill
- B.Cu segment: 0.50 mm wide, to existing `/SYS_5V` via at (30.1375, 76.950 mm)
- Parent revision: `6043198`; parent board SHA-256: `9f0b1cafff47ec27b3988e247a1b1e6173e0750b3d28bd43ce45aaebc8e99bb9`
- Result board SHA-256: `2ff21565e387e612a01f0691dfe4778e9c80572b8f9e8b9c3418e3aea74db1a3`

The existing F.Cu track, both vias, and the added B.Cu track are assigned `/SYS_5V` in the native board model. KiCad CLI 9.0.2 native DRC reported 7 violations before and after with identical violation signatures; unconnected items fell from 60 to 59. The board still has 59 unrouted connections and 7 violations, and is not fabrication-ready.

DRC report SHA-256 values: parent `55328098f8b63fbb2467962040d1bf2f111ab2ace4d010bac22094a46125696b`; candidate `1a97c9c6343068e14da1d7f79f290cafd14e8e3f09c38b01672b6d1277d342b1`.
