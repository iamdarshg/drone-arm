# FA4119 Propulsion Decision Record

Status: **selected prototype baseline; not yet flight-released**

Decision date: 2026-08-26

Applies to: four-motor, 12S propulsion prototype

## 1. Decision

Use four Flycci FA4119 outrunners as the motor platform instead of manufacturing
the previously proposed stator, rotor bell, magnet carrier, shaft and bearing
system from first principles.

The purchased motor is the FA4119 KV350:

- 12 slots / 14 poles (7 pole pairs)
- 41 mm stator OD x 19 mm active stack
- approximately 50.5 mm motor OD x 44 mm height
- approximately 280 g including cable
- stock 350 rpm/V winding
- manufacturer recommendation: 10S-12S, 80 A ESC, 13-inch propeller
- manufacturer-published 12S-class point: approximately 3.47 kW, 74 A,
  11,256 rpm and 8.18 kgf using an HQ 13x9 three-blade propeller

The intended experimental variant is a **sensorless rewind to approximately
550 Kv**, operated from 12S with an Orange HD 8045 (8x4.5) CW/CCW propeller.
This combination is a development target, not a validated operating point.
The stock 350 Kv winding remains the lower-risk first-article baseline and
must be dyno-tested before a rewind is authorized for all four units.

No Hall-effect sensors are required. Rotor position is estimated by the ESC.

## 2. Requirements that drove the selection

The final system-level constraints used in this decision are:

| Requirement | Current value |
| --- | --- |
| Motors | 4 |
| Battery | 12S LiPo, 44.4 V nominal, 50.4 V fully charged |
| Desired nominal input per motor | approximately 2.7 kW |
| Desired current point | approximately 60 A at nominal 12S |
| Burst duration under discussion | up to 50 s at approximately 2.7 kW; requires test validation |
| Aircraft mass | approximately 8 kg |
| Desired total static thrust | aspirational 20 kgf; not yet demonstrated |
| Propeller packaging | ultimately moved from 5.5-7 inch concepts to an 8x4.5 candidate |
| Controller | sensorless FOC; current, RPM and temperature limiting required |

At nominal 12S, 60 A corresponds almost exactly to the desired input power:

```text
44.4 V x 60 A = 2.664 kW
```

At maximum charge, the same current is 3.024 kW. The FA4119's published
3.47 kW / 74 A test makes it the only retail candidate considered in this
thread with evidence near the required electrical power. That evidence does
not validate the rewind or the 8-inch propeller.

## 3. Alternatives considered

### 3.1 Fully custom 12-slot / 8-pole motor

The custom route evolved through several geometries, including a 42 mm stator
OD, approximately 20 mm stack, 62 mm motor OD, 45 mm height, 8 mm shaft,
retail block magnets, a formed 0.5 mm mild-steel flux liner, carbon/glass
retention, and machined 6061-T6 end components.

It was rejected as the primary prototype route because it required all of the
following to be solved simultaneously:

- retail electrical-steel or destructive donor-core sourcing
- lamination punching and interlaminar insulation
- a new high-speed rotor and magnet-retention system
- unverified retail magnet grade and temperature capability
- composite process qualification and rotor overspeed testing
- high-speed bearing qualification
- corrected manufacturing CAD, tolerance drawings and CNC quotations
- coupled electromagnetic, thermal, mechanical and harmonic validation

The custom artifacts remain useful research, but they are superseded for the
current prototype. They must not be mixed with the FA4119 BOM.

### 3.2 Three REES52 5010-class donors per motor

This route attempted to combine multiple inexpensive donor stacks to obtain
the desired active length. It required 12 donor motors for four propulsion
units and still retained a custom rotor, magnets, liner, shaft and end cap.
The material and integration cost approached that of a more suitable complete
motor while preserving most of the technical risk. It was rejected.

### 3.3 Axisflying AZ3115/AZ3215 900 Kv

Advantages:

- approximately 900 Kv already close to the earlier 800-840 Kv target
- comparatively inexpensive
- no major rewind needed for that Kv region

Reasons rejected for the final 12S platform:

- nominally a 3S-6S motor
- published power approximately 1.6-1.74 kW, below the 2.7 kW target
- published thrust approximately 4.1 kgf with a much larger 9-10 inch prop
- stock rotor not qualified for 12S no-load speed

### 3.4 Tarot 5008 340 Kv

The Tarot has a large diameter but only an approximately 8 mm active stack and
about a 700 W published power level. Raising it from 340 Kv to the earlier
800 Kv target would require a severe turn-count reduction while retaining a
core designed for low-speed 17-18 inch props. It was rejected.

### 3.5 Flycci FA4119 350 Kv

Selected because its 41x19 mm stator is almost exactly the desired compact
core, the complete motor fits the 62 mm OD / 45 mm height packaging envelope,
and its published 12S test exceeds the desired per-motor power. Buying the
complete motor removes the need to fabricate the rotor, back iron, magnets,
shaft, bearing system and housing for the first prototype.

## 4. Why the topology is now 12N14P

The selected motor is 12-slot / 14-pole. Rewinding changes turns, conductor
area, resistance, inductance, termination and Kv; it does **not** change rotor
pole count. Converting the FA4119 to the previously discussed 8-pole topology
would require a new rotor and would defeat most of the reason for selecting a
complete retail motor.

For this release:

```text
pole pairs = 14 / 2 = 7
electrical RPM = mechanical RPM x 7
electrical frequency (Hz) = mechanical RPM x 7 / 60
```

## 5. Winding decision

### 5.1 Stock first article

Purchase one FA4119 first and record, before destructive unwinding:

1. Stock no-load Kv at a safe low voltage.
2. No-load current versus speed.
3. Line-to-line resistance corrected to 25 C.
4. Line-to-line inductance at 1 kHz and 10 kHz.
5. Star or delta termination.
6. Turns per tooth, winding direction and phase sequence.
7. Strand count and bare/enamelled strand diameter.
8. Removed copper mass and usable slot fill.
9. Back-EMF waveform and phase balance.

The other three motors must not be unwound until the first article produces a
repeatable winding and passes the guarded dyno sequence.

### 5.2 Nominal 550 Kv target

If magnetic flux and termination are unchanged, turns scale approximately
inversely with Kv:

```text
N_550 / N_350 = 350 / 550 = 0.636
```

The 550 Kv winding therefore requires approximately 63.6% of the stock
effective series turns. Integer turns determine the actual Kv, so 550 is a
target, not a value to print on the motor before spin measurement.

If the stock winding is star-connected, a star/delta change alters Kv by
approximately sqrt(3) and must be included before choosing the turn count.
The freed slot area must be used for additional parallel copper so that phase
resistance and current density remain controlled.

Only Class-H or better enamelled winding copper is acceptable. Nomex slot
liners and compatible impregnation varnish remain in the rewind BOM. Building
wire, speaker wire, bare copper strips with many joints, and PVC/nylon-insulated
conductors are not approved winding substitutes.

## 6. 12S speed envelope

The theoretical no-load values are:

| Winding | 44.4 V nominal | 50.4 V maximum | Maximum no-load eRPM |
| --- | ---: | ---: | ---: |
| Stock 350 Kv | 15,540 rpm | 17,640 rpm | 123,480 eRPM |
| Proposed 550 Kv | 24,420 rpm | 27,720 rpm | 194,040 eRPM |

The earlier 40,000 mechanical-rpm discussion corresponds to 280,000 eRPM and
4.67 kHz electrical frequency. It may be retained as a future bare-rotor
research point, but it is not an operating target for an 8-inch propeller.

## 7. Selected propeller and unresolved rating

Candidate: Orange HD 8045, 8x4.5, one CW and one CCW per pair:

<https://robu.in/product/orange-hd-propellers-80458x4-5-carbon-fiber-props-1cw1ccw-1pair-black/>

The retail title calls it carbon fibre, while comparable listings describe an
8045 carbon-nylon propeller. No manufacturer-issued maximum-RPM certificate
for the exact item was found. It must therefore be treated as an **unrated
multirotor propeller**, not as a continuous-laminate carbon racing propeller.

For reference, APC's published category limits are:

```text
multirotor: 105,000 / diameter_in = 13,125 rpm for 8 inches
durable FPV: 150,000 / diameter_in = 18,750 rpm for 8 inches
racing electric: 270,000 / diameter_in = 33,750 rpm for 8 inches
```

These formulas apply to APC's own qualified categories and do not certify the
Orange prop. Until written evidence is obtained, the initial hard ceiling is
13,000 mechanical rpm.

Eight-inch rotational tip speeds are:

| Speed | Tip speed | Approximate Mach at sea level |
| --- | ---: | ---: |
| 13,000 rpm | 138 m/s | 0.40 |
| 17,640 rpm | 188 m/s | 0.55 |
| 18,750 rpm | 199 m/s | 0.58 |
| 27,720 rpm | 295 m/s | 0.86 |
| 40,000 rpm | 426 m/s | 1.24 |

The values exclude axial/pitch velocity. Unrestricted 550 Kv operation on a
fully charged 12S pack is therefore not approved with this propeller.

## 8. Required controller limits

A throttle percentage limit alone is not sufficient because an unloading,
damaged or detached propeller lets the motor accelerate toward no-load speed.
The sensorless FOC controller must use measured/estimated RPM plus current and
temperature feedback.

Initial first-article limits:

| Limit | Initial value |
| --- | ---: |
| Soft RPM rollback | 12,500 mechanical rpm |
| Hard RPM shutdown | 13,500 mechanical rpm |
| Soft battery-current rollback | 55 A |
| Hard battery-current limit | 60-65 A |
| Winding-temperature rollback | 100-110 C |
| Winding-temperature shutdown | 120 C during development |
| Bell/magnet-region warning | 70 C |
| Bell/magnet-region shutdown | 80 C pending magnet/adhesive data |

Only after the exact propeller obtains a suitable rating and guarded dyno
testing passes may a second profile be considered:

| Limit | Provisional upper profile |
| --- | ---: |
| Soft RPM rollback | 17,000 mechanical rpm |
| Hard RPM shutdown | 18,000 mechanical rpm |
| Nominal current limit | 60 A |
| Short overcurrent ceiling | 70-75 A, subject to thermal data |

The ESC must support at least 200,000 eRPM with margin, 12S maximum voltage,
sensorless FOC, RPM telemetry/governing, current limiting and thermal rollback.

## 9. Cost decision for four motors

Confirmed motor price: INR 4,500 each.

| Item | Four-motor estimate |
| --- | ---: |
| Four FA4119 motors | INR 18,000 |
| Rewind copper, Nomex, varnish, thermistors and small consumables | approximately INR 7,000 |
| Motor plus rewind subtotal | **approximately INR 25,000** |
| Previously selected composite order | INR 4,271 |
| Vacuum pump for infusion | INR 6,800 |
| Motor, rewind, composite order and pump | **approximately INR 36,071** |

Two propeller pairs, ESCs, shipping, taxes not included in the confirmed motor
price, and replacement test articles must be added. If the stock 350 Kv
winding is retained, approximately INR 6,900 of rewind materials can initially
be deferred.

The FA4119 path removes these custom-motor purchases from the active BOM:

- separate rotor magnets
- formed mild-steel rotor liner
- custom rotor carbon/glass sleeve
- custom 6061 rotor end cap and stator carrier
- separate 8 mm shaft stock
- separate 608 bearings
- Hall-effect sensors
- donor motors or donor laminations

## 10. Verification and release gates

No claimed thrust, aircraft speed, efficiency or 50-second burst value is a
release fact until measured with the exact winding and propeller.

The minimum sequence is:

1. Characterize one untouched stock motor.
2. Balance the stock bell and each propeller independently.
3. Dyno the stock 350 Kv motor with the selected prop behind containment.
4. Test at 8,000, 10,000, 12,000 and 13,000 rpm while logging voltage,
   current, input power, RPM, thrust, torque, winding temperature, bell
   temperature, ESC temperature and vibration.
5. Inspect the prop hub, blades, shaft, bell, magnets and bearings after every
   step.
6. If the stock configuration cannot meet the measured aircraft requirement,
   rewind only the first article to the calculated 550 Kv winding.
7. Measure actual Kv, resistance, inductance, no-load current and phase balance.
8. Repeat the guarded propeller ladder at the initial limits.
9. Run 2-minute thermal characterization at lower current before attempting
   the requested 50-second 2.7 kW burst.
10. Perform a propeller-free overspeed test in a rated containment fixture;
    do not infer rotor proof from a propeller test.
11. Freeze winding turns/strand count only after two repeat builds correlate.
12. Flight release requires four matched motors and props plus aircraft-level
    loss-of-prop, ESC fault, vibration and EMI testing.

## 11. Superseded documents

The following release-package files describe an older custom motor and remain
only for traceability:

- `MOTOR_SPECIFICATION.md`
- `materials.md`
- `magnet_specification.md`
- `bearing_specification.md`
- `tolerances.md`
- custom STEP, DXF, drawing, winding and optimizer artifacts in this directory
- `FINAL_BOM_4_MOTORS_EXCL_STATOR_CORE.md` and its CSV counterpart

They are not manufacturing instructions for the FA4119 prototype. This
decision record is the current propulsion baseline until measured first-article
data supports a revised release.

## 12. External references

- Flycci FA4119 manufacturer data:
  <https://www.flyccimotor.com/fa4119-brushless-motor-product/>
- Robu FA4119 retail listing:
  <https://robu.in/product/flycci-fpv-brushless-motors-fa4119-350kv/>
- Selected Orange HD 8045 retail listing:
  <https://robu.in/product/orange-hd-propellers-80458x4-5-carbon-fiber-props-1cw1ccw-1pair-black/>
- APC category RPM limits used only as reference:
  <https://www.apcprop.com/technical-information/rpm-limits/>
