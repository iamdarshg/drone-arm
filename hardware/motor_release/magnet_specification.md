# Permanent Magnet Specification

## Grade and geometry

- Material: sintered NdFeB, grade **N42SH** (Br = 1.24-1.30 T design window; the optimizer used 1.244 T)
- SH temperature class: max continuous operating temperature 150 C, with intrinsic coercivity Hcj >= 20 kOe retained to that temperature
- Magnet radial thickness: 6.5 mm
- Pole count: 8 (arc-pitched to give approximately 0.85 pole arc to pole pitch ratio)
- Axial length: 56.0 mm (matches stator stack)

## Thermal and demagnetization verification

- Continuous thermal FEM gives magnet-region temperature <= 113.5 C at 45 C ambient with 60 A continuous: 36.5 C of margin to the SH 150 C limit.
- Burst case (90 A / 30 s) transient adds under 15 C: still at least 20 C of margin.
- Demagnetization check: peak armature reaction at 90 A burst keeps the operating point above the knee for N42SH at 130 C; manufacturer to confirm with FEA of the final magnet shape.

## Retention

1. Bond line: structural epoxy, 0.10 mm nominal, cured per adhesive datasheet.
2. Mechanical backup: 0.5 mm minimum steel retaining ring or bell lip crimp designed for hoop stress at 16,659 rpm proof speed with safety factor >= 2 against ring yield.
3. Balance: rotor assembly balanced to ISO 21940 G6.3 after magnet installation.

## Acceptance

- Each magnet batch certified for Br, Hcj, and SH-class temperature coefficient.
- Sample pull test: bond plus retention system survives 1000x centrifugal equivalent acceleration at room temperature on a spin fixture.
