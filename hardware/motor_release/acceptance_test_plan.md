# Acceptance Test Plan

Every motor (or sample per sampling plan noted) must pass these tests before
shipment. Flight release additionally requires the system-level tests at the end.

## A. Winding and insulation (100% of units)

1. Resistance: measure phase-to-phase cold at 25 C reference; accept 19.7 mOhm +/- 10%.
2. Inductance: LCI bridge at 1 kHz and 10 kHz; record values, compare across units +/- 15%.
3. Insulation: 500 VDC megger, >= 100 MOhm phase-to-frame.
4. Hi-pot: 1.2 kVAC 1 s phase-to-frame, no breakdown.

## B. Back-EMF and magnetics (sampling: first article + every 20th)

1. Spin test at 3000 rpm: line-to-line RMS BEMF per krpm within +/- 7% of model.
2. BEMF waveform capture; THD < 8%; confirm balanced phases within 2%.
3. Pole/slot verification by flux mapping or low-voltage spin scope check.

## C. Mechanical (first article + every 50th)

1. Balance grade verification G6.3 at rated speed on balancing machine.
2. Overspeed spin test: 16,700 rpm for 60 s, room temperature, post-test dye-penetrant on retaining ring and bell.
3. Shaft runout <= 0.02 mm TIR at prop seat after overspeed.
4. Air gap feeler check at 4 quadrants: all readings >= 0.35 mm.
5. Magnet retention pull-off sample test per magnet_specification.md.

## D. Thermal (first article + annual requalification)

1. Thermocouples: 3 x winding end turns, 1 x housing, 1 x bearing front.
2. Continuous soak: design point load to thermal steady state with 10 m/s airflow; winding must stabilize below 130 C at 45 C ambient.
3. Burst transient: 90 A for 30 s from steady state; peak winding temperature below 150 C.
4. Bearing temperature below 90 C throughout.

## E. Performance (every unit, production dyno)

1. Efficiency map spot points at 25/50/75/100% of rated speed at 60 A: measured shaft efficiency recorded; acceptance threshold defined after first-article dyno correlation (analytical estimate is ~96% electromagnetic only).
2. No-load current at 13,000 rpm within manufacturer-correlated limit.
3. Cogging torque qualitative check; audible noise at rated speed recorded.

## F. System-level (flight release gate, on aircraft-integrated motor)

1. ESC FOC autocalibration completes cleanly with this motor (resistance, inductance, flux observer).
2. Locked-rotor, phase-open, and phase-short fault response per REV_B_ARCHITECTURE.md validation list.
3. Regeneration behavior verified against battery charge-acceptance limits.
4. EMI scan with motor running at cruise power alongside GNSS and 915 MHz radio active.

## Acceptance authority

First-article signoff requires the full matrix above witnessed and archived
(data files, thermocouple logs, dyno traces). Production release per sampling
plan requires sections A, E, plus C.3 runout check.
