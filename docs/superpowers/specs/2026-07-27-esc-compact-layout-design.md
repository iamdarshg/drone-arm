# Compact Six-Motor ESC Layout Design

## Scope

Rebuild the Rev-B six-motor ESC as a manufacturable board no larger than
200 mm by 300 mm while preserving six electrically distinct 60 A motor power
cells. The target outline is 196 mm by 296 mm. This work covers the ESC only;
the control board remains a separate schematic, PCB, BOM, and fabrication
deliverable.

The electrical architecture remains 12S maximum (50.4 V fully charged), one
DRV8353S and one STM32G431 per motor, six IPTC014N10NM5 MOSFETs per motor, and
external individually fused battery feeds. There is no common 360 A PCB power
trunk. The board carries at most 60 A continuous for each motor cell and up to
360 A only as the sum of six independent external branches.

## Mechanical and Thermal Architecture

Use a 2-column by 3-row matrix of identical motor cells inside a nominal
196 mm by 296 mm outline. Each motor-cell allocation is at most 92 mm by
84 mm. A supervisor and auxiliary-power strip occupies the remaining edge
space without crossing any motor-cell power region. Battery and motor
connections face the perimeter so high-current cables do not cross control
or current-sense routing.

All electrical components are on the front side. The back side is flat for a
continuous heatsink interface. Cooling is applied on both faces:

- Electrically insulated front heat spreaders contact the exposed tops of the
  TOLT MOSFETs, following Infineon's TOLT reference-design practice.
- An electrically insulating, thermally conductive interface couples the flat
  PCB back side to a rear heatsink.
- Solder mask is not accepted as the electrical insulation between any live
  copper or MOSFET surface and a common heatsink.
- The front spreader footprint covers only aligned MOSFET rows. Drivers,
  shunts, capacitors, isolation parts, and connectors remain outside its clamp
  and tolerance envelope.
- Final mechanical drawings must specify interface material, dielectric
  rating, thickness, compression, flatness, mounting torque, and creepage from
  conductive clamps.

The IPTC014N10NM5 remains the selected MOSFET. Its TOLT package is intended for
top-side cooling; a rear heatsink alone is not a substitute for the front
thermal path. No gate-drive booster is added by default. The DRV8353S drive
strength and switching resistors are selected by double-pulse testing before
any booster-stage redesign.

## Motor-Cell Placement

Each cell has three parallel, consistently oriented half-bridge columns. The
high-side MOSFET, low-side MOSFET, phase output, and low-side shunt for each
phase form one short vertical power path. The three shunts face their Kelvin
pads toward a centrally placed DRV8353S. Gate resistors sit immediately beside
their associated MOSFET gate/source pads. The STM32G431, ADC filters, and
isolation components occupy a quiet inward-facing control bay adjacent to the
driver rather than behind a switch-node polygon.

Placement must reserve explicit copper-free front-layer corridors for:

- six gate/source return pairs;
- three shunt Kelvin pairs;
- the motor bus-shunt Kelvin pair;
- current-amplifier outputs and ADC filters;
- PWM, SPI, fault, and enable signals between the driver and local MCU.

Power polygons are added only after these routes are complete. A power polygon
must not be used to conceal or force a sensitive signal onto an inner layer.
The six cells use the same relative placement and routing geometry, mirrored
only when the mirror preserves component-side orientation, Kelvin polarity,
gate-loop geometry, and front-spreader access.

## Stackup and Power Distribution

Use four copper layers with 4 oz finished copper on F.Cu and B.Cu and 1.5 oz
finished copper on both inner layers. The stackup, dielectric thicknesses,
finished board thickness, copper tolerances, and resin system must be recorded
in the fabrication notes before release.

Within each motor cell:

- In1.Cu distributes only that cell's fused BATP branch.
- B.Cu distributes only that cell's BATN return and remains free of components.
- In2.Cu carries the three local phase paths to perimeter motor terminals.
- F.Cu contains device landings, short commutation connections, gate routes,
  Kelvin routes, and local decoupling connections.
- Dense via fields join power landings to their assigned planes. Via count,
  drill, annulus, finished-hole copper, and current derating must be verified
  against the selected fabricator's capability.
- Switch-node copper is limited to the area required for current capacity and
  thermal spreading; it must not extend beneath the driver, MCU, isolation
  barrier, current filters, or other quiet analog areas.

The external harness or busbar provides star distribution and one appropriately
rated fuse per motor. The PCB must not connect BATP or BATN nets belonging to
different motor cells. Connector, fuse, cable, busbar, fastener, and terminal
ratings are part of the 60 A continuous-current validation.

## Gate Drive and Switching

Retain the DRV8353S programmable gate drive. Apply these PCB acceptance limits
to every half bridge:

- gate resistor to MOSFET gate/source connection: at most 10 mm;
- driver output to gate resistor: at most 35 mm;
- gate and local source-return routes: F.Cu only, no vias;
- each gate route is paired with its source return and kept outside switch-node
  copper except at the MOSFET termination;
- bootstrap and charge-pump capacitors are placed at their driver pins using
  datasheet-recommended topology;
- local DC-link MLCCs connect directly across each half bridge with minimum
  commutation-loop area.

Initial gate resistors remain configurable footprints. Production values are
chosen from double-pulse measurements of VDS overshoot, ringing, switching
loss, false turn-on margin, and driver temperature at the maximum intended bus
voltage and representative current. A pre-driver or booster is considered only
if the programmable DRV8353S cannot meet those limits without excessive loss.

## Current Measurement

Retain high-speed per-leg and per-motor measurement for every motor:

- Three 0.5 milliohm four-terminal shunts feed the DRV8353S current-sense
  amplifiers for phase-leg current.
- One 0.5 milliohm four-terminal bus shunt feeds an INA296A2 for total motor
  current.
- Each differential sense pair leaves only from the shunt Kelvin pads, is
  routed on F.Cu without vias, and has at most 2 mm pair-length skew.
- Kelvin routes do not share current-carrying copper segments with shunt power
  pads and do not cross switch-node copper.
- The existing 47 ohm and 10 nF output filters remain at the STM32 ADC pins,
  giving an approximate 338 kHz pole unless simulation or measurement justifies
  a documented change.
- All four channels are sampled from timer-triggered ADC sequences synchronized
  to PWM. The three leg channels and motor bus channel must be observable at a
  rate sufficient for cycle-by-cycle protection and control validation.

The audit must verify Kelvin pad identity, polarity, layer, via count, skew,
route length, and separation from power copper for all 24 shunts.

## Isolation, Control, and Protection

Each motor cell retains its local MCU, isolated communications/power boundary,
hardware-gated driver enable, and driver fault connection to a timer break
input. The RP2354B supervisor and auxiliary rails remain outside the six power
cells. Isolation creepage and clearance are enforced through copper and
component keepouts on every layer, including beneath front spreaders and rear
heatsink mounting hardware.

Retain local DC-link decoupling, reverse/overvoltage protections already
validated in the schematic, and the 51 V-class pulse clamp only within its
datasheet pulse-energy limitations. The clamp is not treated as a substitute
for low-inductance battery wiring, local film/ceramic capacitance, or measured
switching overshoot. Any protection component that cannot survive the measured
energy is replaced before production release.

## Schematic Modularity

The ESC root schematic contains only the six motor-cell sheet instances,
supervisor/control sheet, and auxiliary-power sheet. Each motor sheet exposes
explicit battery input, phase outputs, isolated control, fault, and telemetry
ports. Motor sheets remain electrically distinct and use motor-scoped net
names. The control board stays in `hardware/main/rev_b` and is never imported
as an ESC subsheet.

Generated schematic sheets must be tracked intentionally and must not be hidden
by broad ignore rules. The generator and checked-in KiCad sources must produce
the same component populations and connectivity.

## Validation and Release Gates

The ESC is not complete until all of the following pass on the actual release
PCB, not a simplified cell extraction:

1. Board outline is at most 200 mm by 300 mm; target is 196 mm by 296 mm.
2. Every electrical footprint is on F.Cu and the rear component count is zero.
3. Six independent 60 A motor cells and all required component populations are
   present, with no cross-cell BATP/BATN connection.
4. KiCad ERC reports zero errors.
5. KiCad DRC reports zero violations and zero unconnected items after zones are
   filled.
6. Gate-loop and Kelvin-route audits meet every limit in this specification.
7. Power copper, vias, terminals, fuses, and harness paths pass documented
   current-density, voltage-drop, and temperature-rise calculations, followed
   by instrumented load testing.
8. Double-pulse testing validates gate resistance, overshoot, ringing, false
   turn-on margin, and driver temperature.
9. Thermal testing validates the front spreaders and rear heatsink at the
   intended ambient, airflow, bus voltage, PWM frequency, and 60 A motor load.
10. Isolation, creepage, clearance, and dielectric interfaces are documented
    and inspected against the selected fabrication and assembly process.
11. Gerbers, drills, IPC-356/netlist, pick-and-place, BOM, schematics, PCB,
    validation reports, thermal/mechanical notes, and README are regenerated
    into a clean release directory.
12. Costing is recalculated in INR from current supplier pricing, with trivial
    passive substitutions allowed only when footprint, voltage, temperature,
    tolerance, ripple/current, parasitics, lifecycle, and availability remain
    suitable.

Java-based routing tools must be launched with a maximum heap of 500 MB. An
autorouter result is only a candidate; it cannot waive the sensitive-route,
thermal, electrical, or DRC gates.

## Reuse Assessment

The fetched `agent/rev-b-fab-ready` and `agent/rev-b-fab-release` branches may
be mined for schematic modularization and validation utilities, but their PCB
outputs are not accepted. Their recorded control-board routing remains
incomplete, their ESC outline remains 474 mm by 246 mm, and the release branch
records failed ESC layout/DRC gate exit codes. Any reused change must be applied
selectively and revalidated against this specification.
