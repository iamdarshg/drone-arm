# Hardware revision B architecture

Revision B is a new, distinct ESC and flight-control implementation. It does
not overwrite or silently approve the original hardware. The generated
projects are:

- `hardware/esc/rev_b/esc_rev_b.kicad_sch` and
  `hardware/esc/rev_b/esc_rev_b.kicad_pcb`
- `hardware/main/rev_b/main_rev_b.kicad_sch` and
  `hardware/main/rev_b/main_rev_b.kicad_pcb`

## Design basis

| Item | Revision B basis |
| --- | --- |
| Battery | 12S LiPo, 50.4 V maximum charged voltage |
| Motors | Six independent three-phase BLDC/PMSM channels |
| Current | 60 A continuous design current per motor channel |
| Aggregate | 360 A maximum, distributed by six external fused star/busbar taps; never by a shared PCB trunk |
| ESC stack | Four layers: 4 oz F.Cu/B.Cu and 1.5 oz In1.Cu/In2.Cu |
| Initial PWM | 20 kHz, to be finalized by switching-loss, acoustic, and control-loop tests |
| Safe state | Every gate off until both central and local hardware/firmware permissions are valid |

A battery above 12S is a major architecture change. The 100 V MOSFET, 100 V
gate driver, TVS selection, DC-link capacitors, dividers, creepage, and
switching-overshoot margin must all be redesigned for it.

## Why local motor controllers are required

The six battery returns are independent high-current nodes. Millivolt shunt
signals, single-ended PWM, or SPI cannot be carried between those nodes and a
central digital ground without creating measurement error and unintended
return-current paths.

Revision B therefore contains six electrically partitioned motor cells. Each
cell is referenced only to its own `M<n>_BATN` and contains:

- one STM32G431CBT6 motor-control MCU;
- one DRV8353S three-phase gate driver;
- six IPTC014N10NM5 100 V TOLT MOSFETs;
- three 0.5 mOhm four-terminal low-side phase-leg shunts;
- one separate 0.5 mOhm four-terminal high-side DC-input shunt;
- an INA296A2 bidirectional high-side bus-current amplifier;
- local DC-link capacitance, TVS, temperature sensing, and SWD;
- an ISO6731F fail-low command/arm/status interface; and
- an RFM-0505S functional-isolation converter followed by a TPS70933 LDO.

The central RP2354B on the ESC is a supervisor and communications processor.
It does not generate phase PWM and it does not acquire remote analog shunts.
Each motor MCU performs FOC locally and receives torque/thrust commands over
an isolated UART channel.

## High-speed current acquisition

Every motor has four high-speed current observables:

1. phase-leg A current from DRV8353S CSA A;
2. phase-leg B current from DRV8353S CSA B;
3. phase-leg C current from DRV8353S CSA C; and
4. bidirectional DC-input current from INA296A2.

All four outputs terminate at the local STM32G431 ADC pins. The MCU has two
4 MSPS ADCs with timer-triggered injected conversions. ADC2 sequences phase A
and phase B while ADC1 sequences phase C and bus current. Both two-channel
sequences are launched from the same center-aligned PWM event and complete in
about 0.5 us at the maximum ADC clock. The four channels are therefore
sampled once per PWM period with bounded sub-microsecond skew; this is not a
claim that four independent sample/hold circuits capture at exactly the same
instant.

Each output has a 47 ohm/10 nF C0G filter (approximately 339 kHz pole). The
filter is fast enough for cycle-by-cycle control and fault telemetry while
attenuating switching edges. The DRV8353 CSAs and bus amplifier use a 1.65 V
reference:

- shunt: 0.5 mOhm;
- CSA and INA296A2 gain: 20 V/V;
- scale: 10 mV/A;
- 60 A signal: 0.6 V from midscale;
- approximate linear range with 0.25-3.05 V ADC guard limits: +/-140 A; and
- 60 A shunt dissipation: 1.8 W.

The three phase-shunt force paths and their Kelvin sense pads are separate in
the footprint and netlist. Sense traces are routed as quiet pairs and are not
allowed to share load-current copper after the Kelvin pickup.

## PWM and hardware shutdown

The local STM32 uses TIM1 complementary outputs:

| Function | STM32 pin |
| --- | --- |
| AH/BH/CH | PA8 / PA9 / PA10 |
| AL/BL/CL | PB13 / PB14 / PB15 |
| Driver fault break | PA11 / TIM1_BKIN2 |

TIM1 inserts hardware dead time. DRV8353S `nFAULT` drives the timer break
input, so a driver fault stops PWM without waiting for firmware. Driver
`ENABLE` is the output of a hardware AND gate:

`ENABLE = isolated central ARM_SAFE AND local MCU_ARM`

All three signals have fail-low pull-downs. ISO6731F fail-low behavior, the
central TPS3430 watchdog, the local timer break, and the local command timeout
are independent shutdown layers.

An external gate-drive booster is not fitted. DRV8353S already provides
programmable source/sink current, VDS protection, three CSAs, SPI diagnostics,
and matched high/low propagation. Adding six floating booster supplies and
twelve discrete buffers per bridge would increase loop area and delay spread.
The selected MOSFET has 168 nC typical and 211 nC maximum total gate charge at
the datasheet test point. The PCB places every 2.2 ohm series resistor about
4.1 mm from its TOLT gate; the longest driver-to-resistor air-line is about
27.2 mm before detailed routing. A booster is reconsidered only if measured
double-pulse data shows that the selected IDRIVE setting cannot meet the
switching-loss/overshoot target.

## Power and isolation

The RFM-0505S is an unregulated 1 W functional-isolation module. Its output is
not connected directly to 3.3 V logic: a 30 V-input TPS70933 absorbs the
module's light-load tolerance. TPS709 `EN`, which has a lower absolute maximum
than `IN`, is driven by a local 100 kohm/68 kohm divider rather than by the raw
converter output. This is functional isolation for motor-return offset/noise
containment, not a human-safety isolation barrier.

The ESC auxiliary supply uses two independent LM5164 1 A buck rails:

- one 5 V rail for the ESC supervisor and six isolation-module primary sides;
- one fused 5 V rail exported to the flight-control board.

This removes the marginal loading of a single shared 1 A rail and makes a
flight-board short incapable of brownout-resetting the motor cells.

## ESC current-path layout rules

- Six independent M5 battery inputs are mandatory. No 360 A copper trunk is
  present on the ESC PCB.
- The high-current cells are physically partitioned and no `BATN` domains are
  joined.
- Major 60 A paths use copper areas, 4 oz outer copper, inner-layer
  reinforcement where switching-node geometry permits it, and calculated via
  fields.
- The commutation loop is local to each half bridge and its DC-link
  capacitors.
- The F.Cu switch-node copper is split into small MOSFET landing islands.
  In2 carries the phase to the motor lug and leaves a clear F.Cu corridor for
  paired gate/source routing.
- No reference plane is placed directly under a switching node outside its
  local half-bridge region.
- Gate-drive loops are short, do not share load-current copper, and return to
  the corresponding source.
- The bus and phase Kelvin pairs stay inside the quiet analog/control area.
- TOLT tops require an electrically isolated, mechanically controlled heat
  spreader. The headline MOSFET current rating is not a thermal qualification.

Each cell uses an SM8S51A transient clamp: 51 V standoff, 56.7 V minimum
breakdown, and 82.4 V maximum specified clamp. This improves margin to the
100 V MOSFET limit compared with the earlier 54 V part. It is a pulse clamp,
not a dump load; firmware must limit regeneration and the connected battery
must be capable of accepting the returned energy.

The present PCB outline is intentionally large because the inputs are treated
as six continuous 60 A channels rather than a burst-current marketing rating.
Reducing the outline requires a documented duty cycle, lower current, a
different mechanical busbar/heatsink assembly, or separate per-motor modules.

## ESC-to-flight-control link

Both boards use an MCP2518FD SPI CAN-FD controller and a TCAN3413 3.3 V
transceiver because RP2354B has no native CAN controller. The connector
carries fused 5 V, CAN ground, CANH, and CANL. A common-mode choke,
PESD2CANFD24L-T protection, and an optional 120 ohm terminator are local to
each interface.

The flight-control board remains electrically and physically distinct from
the ESC. It sends torque/thrust requests and receives telemetry; it never
sends raw gate patterns.

## Flight-control corrections

- Real USB-C USB 2.0 connector, 5.1 kohm CC pull-downs, and USB ESD protection.
- RP2354B supply pins and internal switcher network follow the RP2350 hardware
  guide; GPIOs are no longer tied to unrelated supply rails.
- Two dissimilar IMUs and the barometer are retained for fault detection.
- GNSS uses a dedicated UART, local 2.8 V/3.3 V quiet LDOs, an always-on
  backup rail, and a U.FL antenna connection.
- The 915 MHz network uses fixed CC1121/CC1190 reference values, U.FL, a
  continuous RF reference plane, local decoupling/matching placement, and a
  via-fenced RF region. Final matching still requires VNA verification on the
  assembled stackup.
- CAN FD replaces high-current or analog signals on the inter-board cable.

## Required physical validation

Passing ERC, netlist checks, and PCB DRC is necessary but cannot prove a
360 A electromechanical assembly safe. Fabrication release still requires:

1. independent symbol/footprint pin audit;
2. impedance and DC-resistance review against the fabricator's actual stack;
3. one-channel current-limited bring-up;
4. double-pulse switching and overshoot measurements;
5. calibrated shunt/ADC gain, offset, and timing tests;
6. thermal soak at 60 A with the production heat spreader and airflow;
7. regenerative braking and bus-disconnect testing;
8. locked-rotor, phase-open, phase-short, command-loss, and watchdog tests;
9. conducted/radiated EMI testing with GNSS and RF active; and
10. VNA tuning plus regional radio regulatory verification.

Firmware release also requires read-back verification of the DRV8353 SPI
configuration: 6x PWM mode, bidirectional CSA reference, 20 V/V CSA gain,
COAST/BRAKE inactive, conservative IDRIVE/tDRIVE/dead-time/VDS thresholds, and
clean fault registers before `MCU_ARM` may assert.
