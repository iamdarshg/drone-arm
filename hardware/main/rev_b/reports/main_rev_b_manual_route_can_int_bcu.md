# Rev-B manual route: CAN_INT on B.Cu

Added a short F.Cu breakout from U10 pad 39 (`/CAN_INT`) to a new through-via, then a hand-routed B.Cu path to the existing `/CAN_INT` through-via at (27.2547, 24.4203) mm. The existing via continues the net on In1.Cu and F.Cu.

- Source pad: U10.39 at (62.400, 54.950) mm, F.Cu.
- New breakout: 0.20 mm F.Cu to a 0.60 mm / 0.30 mm through-via at (61.200, 57.300) mm.
- B.Cu track width: 0.20 mm. Waypoints from source via to existing via: (55.800, 51.000), (55.800, 46.000), (55.800, 43.000), (51.000, 38.000), (50.000, 39.500), (47.000, 37.000), (45.000, 35.000), (40.000, 29.000), (37.000, 30.500), (30.500, 23.500), (29.400, 23.500), (27.2547, 24.4203) mm.
- Parent board SHA-256: `a2690937d50b35190720988a20ec890c3bdf230c3e4b6c68d7fff5a461043c74`.
- Candidate board SHA-256: `db8fa1ca44907ad5b70b7010bdb67fe06d80dcd6715aea2a02662682ddb47291`.

KiCad CLI 9.0.2 DRC was run in the matching Rev-B project context for both the parent and candidate. The result fell from 56 to 55 unconnected items. The seven existing violation signatures are identical before and after; no new DRC violation was introduced. The complete JSON reports are retained alongside this note. Candidate board and report hashes should be checked against the revision containing this route.

The native board model confirms the new U10 pad breakout, via, and B.Cu segments all carry `/CAN_INT`; the B.Cu chain terminates at the existing `/CAN_INT` through-via. Remaining board status is 55 unconnected items and seven existing violations. The board is not fabrication-ready.

Rejected exploration included BARO_SDA, SENS_MISO, GNSS_RESET_N, and earlier CAN_INT topologies; each added DRC findings and remained temporary. Only this final CAN_INT candidate met the no-new-violations gate.
