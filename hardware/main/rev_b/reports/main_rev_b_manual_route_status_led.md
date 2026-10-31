# Rev-B manual route: status LED control

Added an F.Cu route on the Rev-B main board:

- Net: `/RP2354B MCU/STATUS_LED`
- Endpoints: U10 pad 40 (62.800, 54.950 mm) to R13 pad 1 (52.410, 60.100 mm)
- Track: 0.20 mm wide; routed around the MCU decoupling capacitors and adjacent `MAIN_LED_A` via
- Parent revision: `8aba025`; parent board SHA-256: `62504a4f5fe0941d51098c290abc99ccec300db94e563cb3a7adcf237fa74458`
- Result board SHA-256: `590efd4db73facf1f743e57832c707f5a688532e790f00a1a0f9e95c11fb15f2`

The native board model confirms both endpoints are assigned to `/RP2354B MCU/STATUS_LED`. KiCad CLI 9.0.2 native DRC, run with the matching Rev-B project file, reported 7 violations before and after with identical signatures; unconnected items fell from 58 to 57. The board still has 57 unrouted connections and 7 violations, and is not fabrication-ready.

DRC report SHA-256 values: parent `8807af047bafb149afa897ff2b6ff843e52360a97db23d223be2277ce5172a68`; candidate `c821b3e049d04f0fd3b3a5bc2be0882e75bfee09bb5670e11819d28ef1c9f886`.
