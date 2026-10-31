# Rejected `/CAN_MISO` route topologies

Five temporary `/CAN_MISO` routing topologies were evaluated against the Rev-B board as of source commit `1f5be58ad0639f80b877378f2c2600df779b837b`. Baseline and each candidate were explicitly zone-filled with pcbnew 9.0.2 `ZONE_FILLER.Fill(board.Zones())`, then checked using KiCad CLI 9.0.2 `--severity-all` and the matching project file. None meets the DRC acceptance gate.

- Source board SHA-256: `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Refill script SHA-256: `221f0995b7154e7f5e88c69c4b6e01002477a72f4e5873f11e50067866baf8a6`
- Baseline DRC JSON SHA-256: `598c9b6f6e91d7beebc42142ab2336b0e04f729be040f4683be30f7ac935361d`; 7 violations, 44 unconnected items.

All trials connected the same endpoints: U51.12 `(33.5,19.35)` to U10.37 `(61.6,54.95)`. Both endpoint via escapes use 0.6/0.3 mm vias and 0.2 mm F.Cu traces. The variant changes only the inter-via layer/path.

| Topology | Candidate board SHA-256 | Candidate DRC JSON SHA-256 | Result |
| --- | --- | --- | --- |
| Straight B.Cu | `ac6f4218dc4e3559f7d0f154b83424442f150a2b26f10ed42892b21ca87188cb` | `6ddd0bbc5ab3fb5bcd012cc71455c5ba04aec2fed8abc8e15ed8e272bceef317` | 11 violations / 43 unconnected; added crossings and clearance error. |
| Straight In1.Cu | `634a1e8617026bbb47ab455c22d912f85d25548b649136014a052e2df1279ec3` | `b1dae2661b7e184c76b3874cd2b2d1159dd9738b4c1248da86bb5b054eb806ee` | 12 / 44; added crossings and clearance error; no connectivity reduction. |
| Straight In2.Cu | `bf21b1bf7c2de6130d2b9bf54ea6e9cb2438b86640ae56b15fd8a94991f52810` | `965c8eb7a0d4458aae9299aded3f4515c447cedea9c56d892d269f37393bafe0` | 9 / 43; added USB_DP crossing and CAN_INT via clearance. |
| In2 detour around USB_DP | `5af9b59690c648d789bb621370c5ec9844054ad73e811480660a6163b2ccb4aa` | `58ef264774d804e1000c9ddf79d7e262007110bc267400f5934eaef62ee5bcae` | 11 / 43; added shorts to CAN_MOSI/GND and mask bridge. |
| In2 alternate detour | `b485e2eeafedd706cbdbf38c77f8c0b6442bbccbd93b1545b9b9434f28ca6a09` | `e8b7f7ec80b68ff65861add62dfcac27faf439e007d0bdf79d28acd031ebbc58` | 10 / 43; added shorts to CAN_INT and RF_RESET_N. |

The matching before/after DRC JSONs are included. These candidates were rejected and no CAN_MISO board geometry was promoted.
