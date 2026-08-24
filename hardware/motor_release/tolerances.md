# Tolerances And Critical Fits

## General

- Unspecified dimensions: ISO 2768-m (medium).
- All rotating parts balanced after final assembly: ISO 21940 G6.3 at rated speed.

## Critical fits

| Feature | Fit / tolerance | Rationale |
| --- | --- | --- |
| Stator OD to housing bore | H7/g6 press with adhesive | heat transfer and torque reaction |
| Lamination stack length | 56.0 (+0.3 / -0.0) mm | axial magnet-stator alignment |
| Bearing shaft seats | k5 | standard rotating inner ring fit |
| Bearing housing bores | H7 | outer ring clearance fit with clamp |
| Prop adapter thread | M8x1.25 6H/6g, or per prop spec | clamped, not keyed |
| Shaft runout at prop seat | <= 0.02 mm TIR | vibration limit |
| Air gap (nominal 0.50 mm) | concentricity budget keeps local gap >= 0.35 mm | avoids pull-in under UMP |
| Magnet pocket depth | 6.5 (+0.05/-0.0) mm | bond line control |
| Magnet-to-magnet circumferential pitch | +/- 0.15 mm at rotor OD | cogging and balance |

## Thermal growth verification

At continuous rating the optimizer's expansion model predicts rotor growth of
about 0.07 mm radially at operating temperature. The 0.35 mm minimum local gap
budget above already includes this case plus bearing radial clearance.
