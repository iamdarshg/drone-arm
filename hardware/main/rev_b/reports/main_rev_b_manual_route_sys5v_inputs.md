# Rev-B manual route: SYS_5V regulator inputs

Added one F.Cu connection on the Rev-B main board between the SYS_5V input island at C3 and the existing SYS_5V via at U4:

- Net: `/SYS_5V`
- Endpoints: C3 pad 1 (122.000, 17.775 mm) to existing through-via at U4 pad 1 (124.8625, 26.050 mm)
- Track: 0.50 mm wide, straight segment
- Parent revision: `5bf6e4f`; parent board SHA-256: `6f6f0aa7c7dd1a867c1b1df92b3aa7283acf98e8bb33352413e5a1323e9e7284`
- Result board SHA-256: `9f0b1cafff47ec27b3988e247a1b1e6173e0750b3d28bd43ce45aaebc8e99bb9`

Both endpoints are on `/SYS_5V` in the native board model. KiCad CLI 9.0.2 native DRC reported 7 violations before and after with identical violation signatures; unconnected items fell from 61 to 60. The route joins existing regulator input copper and does not alter the established DRC findings. The board still has 60 unrouted connections and 7 violations, so it is not fabrication-ready.

DRC report SHA-256 values: parent `7788edb3e8b157d0c25af9a5598e4059ec5cd918eefbd1fd34eaf8e46960b6a1`; candidate `1c7db5345675b425eb17360769d90232201a7f38784b5d7e063efcebce677bde`.
