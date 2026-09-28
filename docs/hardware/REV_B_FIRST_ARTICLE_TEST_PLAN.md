# Rev-B first-article verification plan

**Status: procedure only — no physical test results are claimed.** Use this plan only with an identified, fabrication-reviewed board revision and the matching firmware. The ESC can expose destructive fault energy. High-voltage/high-current testing requires a qualified operator, guarded fixtures, remote shutdown, and a written lab risk review.

## 1. Test control and stop rules

Before energizing either board, create a run folder named with date, board revision, serial number, and firmware commit. Record operators, reviewer, instrument IDs and calibration dates, ambient conditions, supply settings, probe setup, raw-data paths, and deviations. A second person must review the setup before any powered motor-cell test.

Do not proceed past a stage unless its prerequisite stage is recorded as PASS. Stop, de-energize, and investigate on unexpected current or temperature, reset, smoke/odor, loss of communications, unsafe gate state, battery/regeneration excursion, or any value outside approved limits. Never bypass a hardware interlock to obtain a passing result.

## 2. Limits to freeze before testing

The design documents specify the broad architecture, but several acceptance limits must be converted into exact numeric values from the approved BOM, datasheets, system requirements, and fabrication stackup before testing. Fill and review this register; a blank limit blocks the corresponding test.

| Parameter | Approved value / source | Reviewer / date |
| --- | --- | --- |
| Board and firmware revision/hash |  |  |
| Allowed input-voltage range and source current limit |  |  |
| Rail DC tolerance and maximum ripple/noise at each IC pin |  |  |
| Allowed inrush and standby current |  |  |
| Gate-driver UVLO/OVLO and allowed VGS/VDS overshoot |  |  |
| PWM frequency, dead-time target and tolerance |  |  |
| Current-sense gain, offset, linearity, and timing-skew limits |  |  |
| Continuous/burst current and duration per cell |  |  |
| Temperature limits and thermal stabilization criterion |  |  |
| Regenerative-energy and battery charge-acceptance limits |  |  |
| CAN FD timing, bus-off recovery, and command-timeout limits |  |  |
| RF conducted power, spurious-emission limits, and link criteria |  |  |
| GNSS desense and coexistence criteria |  |  |
| Environmental/EMI/regulatory limits |  |  |

## 3. Ordered test procedure

### Stage A — unpowered inspection

1. Confirm the PCB, assembly, BOM substitutions, and firmware are the exact revisions listed in the run record.
2. Inspect solder joints, polarity, component markings, mounting, isolation gaps, high-current lugs, shunt Kelvin connections, and contamination under magnification. Photograph both sides and any rework.
3. Check resistance/continuity between battery rails, each motor return, 5 V, 3.3 V, ground domains, and chassis. Compare with a reviewed expected value; do not apply power if a short or unexpected inter-domain bond exists.
4. Check that no motor phase or gate output is inadvertently shorted to a rail, chassis, or another phase. Record the meter and measurement points.

### Stage B — flight-control board, current-limited power

1. Disconnect ESCs, motors, RF power amplifier, and external peripherals.
2. Set the bench source below the approved current limit and ramp input slowly through the approved range. Monitor input current and every generated rail.
3. Check reset, watchdog, clock, USB, sensor identities, GNSS UART, CAN controller/transceiver, and safe GPIO defaults. Confirm gate/arm outputs remain fail-low through reset, watchdog, and firmware restart.
4. Measure rail ripple/noise at receiver IC pins with bandwidth and probe grounding documented. Compare with the approved rail-noise register.
5. Record raw scope captures, telemetry, startup waveform, steady-state current, and anomalies. Do not attach an ESC until this stage passes.

### Stage C — isolated ESC auxiliary rails and supervisor

1. Keep all motor cells disabled and disconnect motors. Apply the approved current-limited supply to the ESC input, beginning at the lowest permitted voltage.
2. Verify both LM5164 auxiliary rails, startup/overshoot, load regulation, ripple, thermal behavior, and that a flight-board-rail short cannot brownout the internal supervisor rail.
3. Verify watchdog reset, communication, local command timeout, and all fail-low arm/isolation states. Confirm no gate-driver enable can assert from power-up, MCU reset, or lost central command.
4. Test one motor cell at a time. Leave the other five cells unpowered or hardware-disabled as required by the approved fixture design.

### Stage D — gate drive and protection with a safe load fixture

1. Use the approved inductive/double-pulse fixture, current-limited source, remote disconnect, differential probes, current probe, and physical guard.
2. At the pre-approved low-energy point, verify PWM polarity, complementary outputs, dead time, VGS amplitude, nFAULT-to-timer-break latency, gate-off behavior, and absence of unintended shoot-through.
3. Increase energy only through pre-reviewed steps. Capture switching-node, VDS, VGS, shunt, and supply waveforms. Compare overshoot, ringing, switching loss, and current-sense disturbance with approved numeric limits.
4. Inject documented driver-fault, overcurrent, command-loss, watchdog, and overtemperature inputs. Verify hardware and firmware safe states and measure shutdown latency. Do not short a live power stage to create faults.

### Stage E — sensing, communications, and interference

1. Calibrate all three phase-current channels and the bus-current channel at zero and approved positive/negative current points. Record gain, offset, linearity, polarity, ADC timing, and channel skew.
2. Exercise CAN FD at planned bus load. Inject missing acknowledgements, bus-off, malformed frames, stale commands, and controller reset; verify recovery and timeout safe-state behavior.
3. With motor switching disabled initially, characterize RF path conducted power and spurs into a rated load. Then test controlled motor operating points while recording CC1121/CC1190 output, receiver desense, GNSS tracking, and supply noise.
4. Perform VNA match, conducted/radiated EMI, and regulatory checks with the production antenna/cables/installation. Use jurisdiction-specific approved limits; a modeled link budget is not a measured range result.

### Stage F — motor/dyno and thermal qualification

1. Attach one approved motor/propulsion load in a guarded dynamometer fixture. Confirm arm, kill, overcurrent, overspeed, and fixture interlocks at low energy before increasing current.
2. Measure DC input power and mechanical output power at agreed speed/torque points. Compute ESC efficiency as mechanical output divided by ESC DC input, stating instrument uncertainty and treatment of auxiliary power consistently.
3. Capture current/voltage, motor speed/torque, switching frequency, and temperatures at documented continuous and burst points. Include production heat spreading, airflow, enclosure, and ambient conditions.
4. Test regeneration only with a source/load that can safely absorb the energy and only within battery charge-acceptance limits. Do not use a non-sinking bench supply as a regenerative battery substitute.
5. Repeat required single-cell tests for all six cells, then run the approved multi-cell test only after each cell independently passes.

### Stage G — aircraft integration gate

1. With propellers removed, verify end-to-end command ownership, failsafe, telemetry, reboot behavior, sensor plausibility checks, and all six ESC safe states.
2. Verify GNSS/radio coexistence and CAN behavior under representative motor switching. Record antenna, cable, installation, and software configuration.
3. Conduct any first powered propulsion test only under an approved fixture and separately signed test authorization. This document is not flight approval.

## 4. Per-test result record

Copy this block for each test; never overwrite raw data from a failed run.

```text
Test ID / stage:
Date, operator, independent reviewer:
Main board revision/hash and serial:
ESC revision/hash and serial/cell:
Firmware commit and configuration:
Procedure revision:
Equipment IDs and calibration dates:
Fixture / wiring / probe setup:
Ambient temperature and airflow:
Approved limits and source:
Input voltage/current limit and load point:
Raw data, scope captures, photos, and scripts:
Measured results, uncertainty, and deviations:
Disposition: NOT RUN / PASS / FAIL / BLOCKED
Failure/anomaly and corrective action:
Retest ID / linked issue:
Operator signoff / reviewer signoff:
```

## 5. Release rule

Update `docs/hardware/VALIDATION_MATRIX.md` with board/firmware revision, test ID, result, and artifact links. Mark EDA/model results separately from bench results. A test is PASS only when raw evidence meets approved numeric limits and an independent reviewer signs the record. Any missing numeric limit, unreviewed waiver, unexplained peak, failed gate, or missing raw data keeps that release gate BLOCKED.
