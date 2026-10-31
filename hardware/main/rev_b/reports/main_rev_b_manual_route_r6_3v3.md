# Rev-B manual route: R6 3V3 branch

Added one F.Cu connection on the Rev-B main board:

- Net: `/3V3`
- Endpoints: R6 pad 1 (33.490, 82.000 mm) to the existing `/3V3` through-via (36.500, 76.000 mm)
- Track: 0.20 mm wide, straight segment
- Parent revision: `696891f`; parent board SHA-256: `ad073a8f42b771ec0897f18fdd37b4a6429acf39a53bd0444ec73ba34befe314`
- Result board SHA-256: `6f6f0aa7c7dd1a867c1b1df92b3aa7283acf98e8bb33352413e5a1323e9e7284`

KiCad CLI 9.0.2 native DRC on temporary parent and candidate board copies reported 7 violations before and after, with identical violation signatures. Unconnected items fell from 62 to 61. The `/3V3` net assignment on both endpoints was checked in the native board model. This route does not change the existing DRC findings; the board still has 61 unrouted connections and 7 violations, and is not fabrication-ready.

DRC report SHA-256 values: parent `08468db2fb538ed46145b066a517b35ed855e12ec4065e234716152ead7f28aa`; candidate `d447db374bada589791361bd50660c2bd3a2bf91155e8ff4ac75c07ec97cfe41`.
