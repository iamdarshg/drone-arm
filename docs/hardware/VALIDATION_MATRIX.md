# Hardware validation matrix

The original hardware remains unapproved. Revision B is a separate corrected
implementation. `PASS` below means the source/netlist/EDA check passed; it is
not a substitute for the physical qualification listed in
`REV_B_ARCHITECTURE.md`.

## Repository and change-source audit

| Check | Result | Evidence |
| --- | --- | --- |
| Latest main | PASS | Fast-forwarded to `origin/main` commit `3b52fbe75671ab1cc6fcc1d298fd52cf6ec10704` before Rev-B work |
| Open hardware PR fix | NONE | No open PR contains a complete electrical correction |
| Closed PR 6 | REJECTED | Mechanical/placeholder routing does not correct the schematics and includes unsafe scripts |
| Other branch | NOT APPLICABLE | `sdk-recreation` is firmware-only |
| Original dirty files | PRESERVED | Rev-B work is confined to new `hardware/*/rev_b`, `docs`, and `tools/hardware` paths |

## Original ESC findings

| Area | Original finding | Revision-B disposition |
| --- | --- | --- |
| IR2136 supply | Driver VCC was 3.3 V although the part requires a gate-drive supply near 10-20 V | Replaced motor stage with DRV8353S |
| PWM independence | Six bridges shared only two input groups | One local STM32G431 and independent TIM1 PWM group per motor |
| Current trip | INA240 midscale conflicted with IR2136 trip threshold | DRV8353S hardware protection plus local ADC plausibility |
| Current acquisition | Remote ADC architecture could not synchronously serve six FOC loops | Three local CSA channels plus one local INA296 bus channel per motor |
| Power switches | Generic/incomplete production definition; IPTC015N10NM5 is discontinued | Infineon's compatible active/preferred IPTC014N10NM5 replacement with the same TOLT pinout |
| Shunts | Undefined value/power and inappropriate footprints | Four CSS4J-4026 0.5 mOhm Kelvin shunts per motor |
| Aggregate feed | PCB was expected to distribute 360 A | Six fused external star/busbar feeds; no common 360 A PCB trunk |
| Return domains | Remote single-ended signals crossed independent BATN references | Only fail-low isolated digital command/arm/status crosses each cell boundary |
| Gate booster | Proposed as a repair for inadequate drive | Not required with DRV8353S; reconsider only after double-pulse measurement |

## Revision-B ESC schematic

| Check | Result | Evidence |
| --- | --- | --- |
| Modular hierarchy | PASS | Top sheet, auxiliary power, supervisor, and six separate motor-cell sheets |
| ERC | PASS | `hardware/esc/rev_b/reports/esc_rev_b_erc.rpt`: zero errors and zero warnings |
| Design-intent connectivity | PASS | Shared audit: 239 checks, zero failures |
| Motor controllers | PASS | Six STM32G431CBT6 |
| Gate drivers | PASS | Six DRV8353SRTAR |
| Power switches | PASS | 36 IPTC014N10NM5, one high and one low switch per phase |
| Leg-current sensing | PASS | 18 low-side 0.5 mOhm Kelvin shunts, all three DRV CSA outputs routed to local ADCs |
| Per-motor current sensing | PASS | Six separate high-side 0.5 mOhm shunts and six INA296A2 bidirectional amplifiers |
| ADC bandwidth | PASS | Four 47 ohm/10 nF outputs per motor to two 4 MSPS STM32 ADCs |
| Hardware break | PASS | Every DRV `nFAULT` reaches STM32 PA11/TIM1_BKIN2 |
| Hardware arm | PASS | `ENABLE = ARM_ISO AND MCU_ARM`, with fail-low pulls |
| Isolation | PASS | No motor MCU, driver, MOSFET, shunt, or local arm-gate node bonds to DGND |
| Isolated power | PASS | RFM-0505S followed by 30 V-input TPS70933, preventing unregulated 5 V output from overvolting 3.3 V logic |
| Isolated LDO enable | PASS | Raw converter output reaches TPS709 `IN`; a local 100 kohm/68 kohm divider keeps `EN` within rating |
| Transient clamp | PASS WITH PHYSICAL TEST REQUIRED | SM8S51A gives 51 V standoff and 82.4 V specified maximum clamp; it is not a sustained-regeneration absorber |
| CAN FD | PASS | MCP2518FD plus TCAN3413, choke, ESD, and optional terminator |
| Auxiliary power | PASS | Two independent LM5164 1 A rails, one internal and one fused for the flight board |
| RP2354B bypass population | PASS | Nine 3.3 V, three DVDD, and one ADC_AVDD high-frequency bypass positions |

## Original flight-control findings

| Area | Original finding | Revision-B disposition |
| --- | --- | --- |
| MCU pins | GPIOs were tied to unrelated supply rails | Correct RP2354B power mapping and internal switcher network |
| USB | Header substituted for USB-C; CC and ESD absent | USB-C receptacle, 5.1 kohm CC pulls, USB ESD |
| GNSS | Interface pins were crossed into IMU SPI | Dedicated GNSS UART |
| Barometer | Interrupt output was tied to ground | Interrupt routed to an MCU GPIO |
| RF DCPL | CC1121 decoupling node was tied to MOSI | Local decoupling only |
| RF matching | Programmable network did not follow a characterized design | Fixed 915 MHz network and U.FL; VNA tuning remains a physical test |
| CAN | No robust ESC command link | MCP2518FD plus TCAN3413 CAN FD |

## Revision-B flight-control schematic

| Check | Result | Evidence |
| --- | --- | --- |
| Distinct board | PASS | No ESC driver, MOSFET, isolation converter, or current-amplifier population |
| Modular hierarchy | PASS | Power/USB, MCU, sensors, GNSS, RF, and CAN sheets |
| ERC | PASS | `hardware/main/rev_b/reports/main_rev_b_erc.rpt`: zero errors and zero warnings |
| CAN controller/transceiver | PASS | MCP2518FD-H/QBB and TCAN3413DR |
| CAN controller path | PASS | SPI/interrupt to RP2354B, TXCAN/RXCAN to transceiver |
| Sensor redundancy | PASS | ICM-42688-P plus LSM6DSO32 retained; LPS22DF barometer present |
| USB | PASS | USB-C USB2 connector, CC pulls, input protection, and ESD |
| GNSS | PASS | LG77L with dedicated UART, local quiet LDOs, always-on backup rail, protected U.FL path, and 3 mm courtyard |
| RF population | PASS WITH PHYSICAL TEST REQUIRED | Fixed CC1121/CC1190 network; VNA/EMI/regulatory verification remains mandatory |
| RP2354B bypass population | PASS | Nine 3.3 V, three DVDD, and one ADC_AVDD high-frequency bypass positions |

## PCB status

| Check | ESC | Flight control |
| --- | --- | --- |
| Board remains distinct | PASS | PASS |
| Explicit four-layer stack | PASS: 4 oz outer, 1.5 oz inner | PASS: four-layer 1 oz baseline |
| Pre-route shorts | PASS: zero | PASS: zero |
| Pre-route copper-clearance violations | PASS: zero | PASS: zero |
| Pre-route courtyard overlaps | PASS: zero | PASS: zero |
| Pre-route illegal zone intersections | PASS: zero | PASS: zero |
| Sensitive placement | PASS | PASS: CAN, GNSS, sensors, CC1121/CC1190 matching and bypass parts are local rather than parked in generic rows |
| Gate placement | PASS: series resistors are about 4.1 mm from gates; longest driver-to-resistor air-line about 27.2 mm | Not applicable |
| Router memory policy | 500 MB Java heap, Serial GC, one optimizer thread | 500 MB Java heap, Serial GC, one optimizer thread |
| Final routing | IN PROGRESS | IN PROGRESS |
| Final DRC | PENDING | PENDING |
| Manufacturing exports | PENDING | PENDING |

## Required bench qualification

The following cannot be proven by KiCad and therefore remain release gates:

- one-channel current-limited bring-up;
- dead-time and hardware-break measurement;
- double-pulse overshoot/switching-loss test;
- current gain/offset/timing calibration;
- 60 A thermal soak with production heat spreader and airflow;
- regenerative, locked-rotor, phase-open/short, and command-loss tests;
- conducted/radiated EMI;
- CAN fault-injection and bus-off recovery;
- GNSS desense test; and
- 915 MHz VNA match plus regional regulatory testing.
