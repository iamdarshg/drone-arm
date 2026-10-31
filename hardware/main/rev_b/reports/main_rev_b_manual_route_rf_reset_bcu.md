# Rev-B manual route: RF_RESET_N on B.Cu

This change completes the open `/RF_RESET_N` connection from U10.20 to the existing RF-section through-via. The source is `drone-arm` main commit `11003919c6390bbf9a71f1bef5a15eb7a8e315c0`; source board SHA-256 `bd28ebb9f6ad98b3a90b655ec510726b83dcc103681e15505549f8667bd6588b`; project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

The native board model contains this continuous same-net path:

- U10.20 `/RF_RESET_N`, F.Cu at (54.050, 53.800) mm to a 0.20 mm F.Cu track ending at a new through-via at (59.000, 53.800) mm.
- New through-via: 0.60 mm diameter, 0.30 mm drill, F.Cu-to-B.Cu through stack.
- 0.20 mm B.Cu route: (59.000,53.800) → (64.500,53.800) → (64.500,60.000) → (68.000,64.000) → existing `/RF_RESET_N` via at (83.3787,64.3687) mm.
- The destination via connects to the existing In1.Cu and F.Cu `/RF_RESET_N` route.

KiCad CLI 9.0.2 with the matching Rev-B project reports 7 DRC violations before and after, with identical violation signatures. Unconnected items fall from 47 to 46. The complete before/after JSON reports are retained beside this file. Their SHA-256 values are `8ac6fcf029fd208e0ea53a517000f7c9b816d953213a984b422f756eb3ca36df` (parent) and `8ac99a99f1d6d71d5a98d0de178dce60936aa10b5181f015815410ea21449fdf` (candidate). Candidate board SHA-256: `8b03f2bce12a3f7ea0780387364c46ed9426d75bc0912c7fa64e553a01fd9bc5`.

Independent read-only verification confirmed the native pad, via, track, and destination continuity, and reproduced the 7/7 identical DRC signatures and 47→46 unconnected count. The board still has 46 unrouted connections and 7 existing DRC findings; this route does not establish fabrication readiness.
