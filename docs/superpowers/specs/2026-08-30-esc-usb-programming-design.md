# ESC USB-C Programming Architecture

Date: 2026-08-30

## Objective

Add USB programming access to every programmable domain on the Rev-B six-channel ESC while preserving galvanic separation between the controller domain and the six motor-cell domains. Remove the six on-PCB bulk capacitors that are physically external to the PCB from the authoritative schematic and PCB/netlist contract.

## Existing programmable domains

The ESC has seven programmable MCUs:

- Controller domain: `U201`, RP2354B, with native USB on pins 66 `USB_DM` and 67 `USB_DP`.
- Motor cell 1: `U1102`, STM32G431CBT6, native USB FS on PA11/PA12.
- Motor cell 2: `U1202`, STM32G431CBT6, native USB FS on PA11/PA12.
- Motor cell 3: `U1302`, STM32G431CBT6, native USB FS on PA11/PA12.
- Motor cell 4: `U1402`, STM32G431CBT6, native USB FS on PA11/PA12.
- Motor cell 5: `U1502`, STM32G431CBT6, native USB FS on PA11/PA12.
- Motor cell 6: `U1602`, STM32G431CBT6, native USB FS on PA11/PA12.

The six motor cells are separate electrical domains. A programming interface must never bond two cell returns together or bond a cell return permanently to controller ground.

## External bulk capacitors

Remove `C1130`, `C1230`, `C1330`, `C1430`, `C1530`, and `C1630` from the six motor-cell schematics and from generated PCB/netlist expectations. Their BATP/BATN connections remain documented as external wiring requirements rather than PCB components.

The schematic notes for each motor cell shall state that the external bulk capacitor is connected directly across that cell's BATP/BATN power leads with the shortest practical high-current loop. No replacement PCB footprint is generated.

## Controller-domain USB-C port

Add one USB-C receptacle, `J202`, to the controller sheet as a USB 2.0 device-only port.

Electrical implementation:

- Receptacle footprint: `Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal`.
- CC1 and CC2 each receive an independent 5.1 kOhm pull-down to controller ground.
- A6/B6 are joined as `CTRL_USB_DP` at the receptacle.
- A7/B7 are joined as `CTRL_USB_DM` at the receptacle.
- D+ and D- pass through a low-capacitance USB ESD array adjacent to the connector.
- D+ and D- each pass through a 27 Ohm series resistor adjacent to U201.
- `CTRL_USB_DP` terminates at U201 pin 67.
- `CTRL_USB_DM` terminates at U201 pin 66.
- USB VBUS is used only for VBUS presence detection and ESD bias. It is not connected directly to the controller 5 V or 3.3 V power rails.
- The RP2354B remains powered by the ESC controller-domain regulator during programming.
- The connector shield uses the existing controller-domain shield/chassis strategy. If no chassis net exists, the shield connects to controller ground through a 1 MOhm resistor in parallel with a 1 nF, voltage-rated capacitor, plus an optional unpopulated direct-bond jumper.
- Existing `J201 ESC_SWD` remains as the recovery/debug interface.

## Shared motor-cell USB-C service port

Add one USB-C receptacle, `J203`, as a USB 2.0 device service port for the six STM32G431 cells.

The service port connects to one and only one motor cell through `SW201`, a six-position, six-pole, break-before-make service selector. The selector is a panel/harness component represented in the schematic and connected to the PCB through keyed service headers. The PCB does not assume that an uncontrolled generic analog multiplexer can tolerate the six floating cell domains.

The six switched conductors are:

1. USB D+
2. USB D-
3. USB ground/selected cell BATN reference
4. USB VBUS presence
5. BOOT0
6. NRST

Selector requirements:

- Break-before-make operation is mandatory on every pole.
- No position may connect two cell grounds simultaneously.
- The selected USB ground connects only to the selected cell's BATN-referenced logic ground.
- The selector and harness must be rated for the maximum common-mode voltage that can exist between motor-cell domains and must maintain adequate creepage and clearance.
- D+/D- must be routed as a paired, short harness suitable for USB Full Speed. Validation includes enumeration and DFU transfer testing through every selector position.
- VBUS is a detect/bias signal only and must not power a motor cell or backfeed its local regulator.

At `J203`:

- CC1 and CC2 each receive an independent 5.1 kOhm pull-down referenced to the selected USB ground after the selector.
- D+ and D- receive low-capacitance ESD protection adjacent to the receptacle.
- VBUS receives surge/ESD protection and current-limited detection circuitry.
- The connector shield is tied to the service shield node and is not permanently bonded to any motor-cell BATN.

## Per-cell native USB/DFU interface

Each STM32G431 motor sheet receives the same replicated USB interface:

- PA12 is assigned to `M<n>_USB_DP`.
- PA11 is assigned to `M<n>_USB_DM`.
- PA11 and PA12 are reserved exclusively for native USB on every motor cell.
- The STM32G431 integrated USB Full-Speed transceiver termination is used; no external D+/D- series termination resistors are fitted on the motor-cell interfaces.
- A local low-capacitance ESD array protects the cell-side lines at the service-header boundary.
- `M<n>_USB_VBUS_SENSE` is current-limited and clamped to the selected cell logic domain.
- Existing `M<n>_BOOT0` and `M<n>_NRST` connect to the selector while retaining their current local bias networks.
- Existing `J<n>03 LOCAL_SWD` remains unchanged and available for recovery/debugging.
- Native STM32 ROM USB DFU is the programming path. SWD is not removed.

The replicated design preserves the current PWM, break input, gate-driver, ADC, CAN isolation, and power-stage functions. The existing PA11 gate-driver fault assignment moves to PB10, LQFP-48 package pin 22, using its `TIM1_BKIN` alternate function so the hardware PWM shutdown path remains available. The existing PA12 arm-isolation assignment moves to PB11, LQFP-48 package pin 25. PA11 and PA12 are then dedicated to USB DM and DP respectively.

## Symbols, footprints, and generated sources

The authoritative generator remains `tools/hardware/generate_rev_b_schematics.py`. The implementation updates the generator first and then regenerates the checked-in schematics.

Expected schematic changes:

- `hardware/esc/rev_b/controller.kicad_sch`: controller USB-C and shared service USB-C/selector interface.
- `hardware/esc/rev_b/motor_1.kicad_sch` through `motor_6.kicad_sch`: replicated native USB/DFU interfaces and external-capacitor notes.
- `hardware/esc/rev_b/esc_rev_b.kicad_sch`: hierarchical pins/nets required by the selector and service interface.
- `hardware/esc/rev_b/revb.kicad_sym`: only if an existing standard symbol cannot represent the break-before-make selector/harness clearly.
- `hardware/esc/rev_b/esc_rev_b.kicad_pro`: only rule or library-table changes required by the new standard/custom footprints.

All new component references are deterministic in the generator so repeated generation produces stable references and hierarchy.

## PCB reconciliation

After schematic regeneration and netlist export:

- Start from the latest saved 165 x 165 mm placement checkpoint.
- Remove the six obsolete `C<n>130` footprints from PCB parity expectations and from the PCB if present.
- Preserve every existing footprint position, orientation, pad position, and the exact 165 x 165 mm Edge.Cuts outline.
- Add all new USB, ESD, resistor, selector-header, and protection footprints outside Edge.Cuts.
- Do not automatically repack or move the six frozen motor cells.
- Reopen the parity-corrected PCB in KiCad for placement review.
- Generate a Quilter package only after schematic/PCB reference parity passes and the user saves the reconciled board.

## Validation

Schematic validation must include fresh evidence for all hierarchical sheets:

1. Regenerate all Rev-B ESC schematics from the authoritative generator.
2. Re-export the XML netlist.
3. Run KiCad ERC on the top-level ESC schematic and capture the JSON report.
4. Confirm every hierarchical sheet loads without rescue symbols or missing libraries.
5. Confirm U201 USB pins 66/67 are connected to the controller USB-C network.
6. Confirm PA11/PA12 on all six STM32G431 devices are connected to their cell USB networks.
7. Confirm no USB net crosses between motor cells except through the explicit selector representation.
8. Confirm no two BATN/local-ground domains are directly connected in the netlist.
9. Confirm `C1130…C1630` are absent from generated schematics and netlist.
10. Confirm schematic reference uniqueness and complete PCB/netlist parity after new footprints are added outside Edge.Cuts.
11. Run fresh PCB DRC and report exact violations and unconnected counts without claiming fabrication readiness.

Hardware validation required before release:

- USB enumeration on the RP2354B controller port.
- RP2354B firmware load and recovery through USB and SWD.
- STM32 ROM DFU enumeration and firmware transfer in each of the six selector positions.
- Verification that non-selected motor-cell grounds remain isolated during every selector position and transition.
- Break-before-make verification while changing selector position.
- USB D+/D- signal-integrity check at Full Speed through the selector/harness.
- No VBUS backfeed into any unpowered controller or motor-cell regulator.

## Acceptance boundary

Schematic completion requires regenerated files, fresh ERC evidence, explicit domain-isolation checks, and complete reference parity. PCB reconciliation requires all newly introduced footprints present outside Edge.Cuts without moving the existing placement. Neither schematic completion nor a Quilter input package constitutes fabrication readiness; PCB routing, DRC closure, power-loop review, thermal validation, and first-article electrical testing remain separate release gates.
