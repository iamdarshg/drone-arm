# Rev-B manual route: R4 1V1 decoupling branch

Added one F.Cu connection on the Rev-B main board:

- Net: `/1V1`
- Endpoints: R4 pad 1 (50.490, 41.500 mm) to the existing `/1V1` through-via at (51.520, 47.000 mm)
- Track: 0.20 mm wide, straight segment
- Parent revision: `e1ef03e`; parent board SHA-256: `2ff21565e387e612a01f0691dfe4778e9c80572b8f9e8b9c3418e3aea74db1a3`
- Result board SHA-256: `62504a4f5fe0941d51098c290abc99ccec300db94e563cb3a7adcf237fa74458`

The native board model confirms both endpoints are assigned `/1V1`. KiCad CLI 9.0.2 native DRC, run with the matching Rev-B project file, reported 7 violations before and after with identical signatures; unconnected items fell from 59 to 58. The board still has 58 unrouted connections and 7 violations, and is not fabrication-ready.

DRC report SHA-256 values: parent `f7550ecf35814fe930607627d15b3f92003c8e653944536195bcc183f43b23b0`; candidate `7beca8b1988ddd34c444765137df03cef4bbf70827dc71117f41fb0997afcfa4`.
