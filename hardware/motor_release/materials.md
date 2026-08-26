# Materials Specification — Superseded Custom Motor

> **Superseded 2026-08-26:** These materials specify the former custom motor,
> not the purchased Flycci FA4119 platform. See
> [FA4119_PROPULSION_DECISION.md](FA4119_PROPULSION_DECISION.md). In particular,
> do not purchase the custom stator steel, back iron, bell, shaft or magnets for
> the current prototype.

## Stator laminations

- Grade: M-330-35A (or manufacturer-proposed equivalent, e.g. NO-20/NO-27 class)
- Thickness: 0.35 mm, insulated both sides (C5 or better stacking insulation)
- Stacking factor >= 0.95, welded or riveted stack, burr <= 0.03 mm
- Core loss target at 1.5 T / 400 Hz <= 20 W/kg (electrical frequency at 13 krpm is ~433 Hz)

## Rotor back iron and bell

- Back iron: same M-330-35A lamination pack bonded to the bell
- Bell/housing: 6061-T6 aluminum, hard anodized inside magnet pockets optional
- Magnets bonded with high-temperature structural epoxy (see magnet_specification.md) plus a 0.5 mm steel retaining ring rated to the 16.7 krpm proof speed

## Shaft

- Primary: 42CrMo4 quenched and tempered to 900-1100 MPa UTS
- Alternate: 17-4PH H900 if corrosion resistance prioritized
- Bearing seats ground to tolerance per tolerances.md; runout <= 0.02 mm TIR at prop adapter

## Windings

- Conductor: copper, class H enamel (polyamide-imide topcoat), 3 x 3.15 mm parallel strands per turn
- Slot liners: aramid paper (Nomex 410) 0.25 mm
- Varnish: vacuum pressure impregnation (VPI), class H silicone-modified or polyester-imide resin

## Permanent magnets

See magnet_specification.md. Summary: N42SH NdFeB, Ni-Cu-Ni plated.
