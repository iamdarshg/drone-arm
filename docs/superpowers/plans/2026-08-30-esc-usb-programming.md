# ESC USB-C Programming Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add one native USB-C port for the RP2354B controller and one break-before-make USB-C service selector for the six isolated STM32G431 motor cells, remove the six off-board bulk capacitors from schematic authority, reconcile the saved 165 mm PCB without disturbing hand placement, and produce fresh electrical-validation evidence and a flat Quilter handoff.

**Architecture:** The controller USB port connects directly to RP2354B USB D+/D-. The service USB port feeds exactly one of six isolated motor-cell USB interfaces through a six-position break-before-make selector; every STM32 keeps SWD. PA11/PA12 become USB DM/DP, while the existing safety and arm signals move only to datasheet-verified spare GPIOs. VBUS is sense-only and must not power or join isolated cell domains.

**Tech Stack:** KiCad 9 schematic/PCB formats and CLI, repository Python schematic generator, `pytest`, XML netlist auditing, `pcbnew`, PowerShell.

**Spec:** `docs/superpowers/specs/2026-08-30-esc-usb-programming-design.md`

## Global Constraints

- Preserve unrelated dirty files and the user's hand placement in `hardware/esc/rev_b/reports/esc_165mm_hybrid_newtop_oldbottom_tight.kicad_pcb`.
- Never connect motor-cell grounds, USB grounds, or VBUS across isolation boundaries.
- Retain all seven SWD interfaces.
- Preserve hardware timer-break behavior for every gate-driver fault input.
- Use STM32G431 native USB without external DM/DP series termination, per the STM32G431 datasheet.
- Keep all newly introduced PCB footprints outside `Edge.Cuts` until the user places/routes them.
- Do not describe the result as fabrication-ready unless ERC, netlist audit, PCB parity, DRC, and unconnected-item gates actually pass.

---

## Task 1: Correct and lock the electrical contract

**Files:**

- Modify: `docs/superpowers/specs/2026-08-30-esc-usb-programming-design.md`
- Create: `tests/test_esc_usb_schematic_contract.py`

- [ ] Replace the per-cell 27 Ω USB-series-resistor requirement with the STM32G431 native-termination requirement.
- [ ] Record the exact verified STM32 package pin used for `DRV_nFAULT`; it must expose a TIM1 break alternate function.
- [ ] Record the exact spare STM32 package pin used for `ARM_ISO`.
- [ ] State that PA11 and PA12 are reserved exclusively for USB DM/DP.
- [ ] Add a failing contract test that reads the generated ESC XML netlist and asserts the six removed capacitor references are absent, USB components exist in the expected counts, PA11/PA12 are USB-only, the migrated safety/arm nets use their approved pins, SWD remains present, and cell domains remain isolated.
- [ ] Run `pytest -q tests/test_esc_usb_schematic_contract.py` and capture the expected failure against the old netlist.
- [ ] Commit only the corrected spec and failing test: `git commit -m "Test ESC USB programming contract"`.

## Task 2: Add deterministic USB and selector primitives to the generator

**Files:**

- Modify: `tools/hardware/generate_rev_b_schematics.py`
- Modify: `hardware/esc/rev_b/revb.kicad_sym` (generated)
- Modify: `hardware/esc/rev_b/sym-lib-table` only if the generator requires it

- [ ] Add generator helpers for a USB-C USB2-device receptacle, 5.1 kΩ CC pull-downs, low-capacitance USB ESD protection, VBUS sense/division, and explicit no-backfeed power flags.
- [ ] Add a deterministic custom symbol for the six-position, six-pole break-before-make service selector or an electrically equivalent connectorized selector representation.
- [ ] Give every new component a stable unique reference; avoid references already used by controller and motor sheets.
- [ ] Add comments and schematic text that identify the selector as break-before-make and prohibit hot-switching during enumeration.
- [ ] Add unit-level assertions for reference uniqueness and selector pole/throw count.

## Task 3: Implement controller-domain USB-C

**Files:**

- Modify: `tools/hardware/generate_rev_b_schematics.py`
- Regenerate: `hardware/esc/rev_b/esc_controller.kicad_sch`
- Regenerate: `hardware/esc/rev_b/esc_rev_b.kicad_sch`

- [ ] Add controller USB-C receptacle `J202`, CC resistors, ESD device, and RP2354B-recommended USB series components.
- [ ] Connect RP2354B USB DM/DP package pins to the controller USB nets.
- [ ] Route VBUS only into protected sensing; prove no connectivity to controller 5 V/3.3 V rails.
- [ ] Keep `J201` SWD connectivity unchanged.
- [ ] Regenerate schematics and run the focused contract test; controller assertions must pass while motor-cell assertions may still fail.

## Task 4: Implement six native motor-cell USB endpoints and remove off-board capacitors

**Files:**

- Modify: `tools/hardware/generate_rev_b_schematics.py`
- Regenerate: `hardware/esc/rev_b/motor1.kicad_sch`
- Regenerate: `hardware/esc/rev_b/motor2.kicad_sch`
- Regenerate: `hardware/esc/rev_b/motor3.kicad_sch`
- Regenerate: `hardware/esc/rev_b/motor4.kicad_sch`
- Regenerate: `hardware/esc/rev_b/motor5.kicad_sch`
- Regenerate: `hardware/esc/rev_b/motor6.kicad_sch`

- [ ] Remove `C1130`, `C1230`, `C1330`, `C1430`, `C1530`, and `C1630` from generation and hierarchy.
- [ ] Assign every STM32 PA11/PA12 pair to cell-local USB DM/DP nets with no external termination resistors.
- [ ] Move every `DRV_nFAULT` to the approved TIM1-break-capable spare pin and every `ARM_ISO` to the approved spare GPIO.
- [ ] Add cell-local low-capacitance ESD protection and cell-local VBUS sensing without powering the cell.
- [ ] Preserve each `Jx103` SWD header and all existing PWM, ADC, CAN-isolation, gate-drive, and power-stage assignments.
- [ ] Regenerate all six sheets and run the contract test until all pin-map, population, and isolation assertions pass.

## Task 5: Wire the shared break-before-make service selector

**Files:**

- Modify: `tools/hardware/generate_rev_b_schematics.py`
- Regenerate: `hardware/esc/rev_b/esc_rev_b.kicad_sch`

- [ ] Add service USB-C receptacle `J203`, CC pull-downs, connector-side ESD, and VBUS sense.
- [ ] Switch DM, DP, ground reference, VBUS sense, and any required shield/drain paths together so exactly one cell is presented at a time.
- [ ] Keep connector shield handling explicit and prevent the selector from creating a shared DC ground between cells.
- [ ] Expose all selector nets through deterministic hierarchy pins.
- [ ] Add netlist assertions proving every selector position reaches one and only one cell and that no two cell grounds share a connected component.

## Task 6: Run complete schematic validation

**Files:**

- Modify: `tools/hardware/validate_rev_b_netlists.py`
- Create: `hardware/esc/rev_b/reports/esc_usb_erc.json`
- Create: `hardware/esc/rev_b/reports/esc_usb_netlist.xml`
- Create: `hardware/esc/rev_b/reports/esc_usb_audit.json`

- [ ] Extend `audit_esc` with removed-capacitor, USB population, exact MCU pin, selector exclusivity, VBUS no-backfeed, SWD retention, and isolation checks.
- [ ] Export the fresh netlist with `kicad-cli sch export netlist --format kicadxml`.
- [ ] Run `pytest -q tests/test_esc_usb_schematic_contract.py tests/test_pin_consistency.py tests/test_motor_constraints.py` and require zero failures.
- [ ] Run `kicad-cli sch erc --format json --severity-all` on the root ESC schematic and report exact errors and warnings.
- [ ] Run `tools/hardware/validate_rev_b_netlists.py` against fresh ESC and Main XML netlists and require all contractual checks to pass.
- [ ] Inspect any remaining ERC item individually; encode only documented intentional exceptions, never blanket exclusions.
- [ ] Commit generator, generated schematics, symbols, validator, tests, and fresh machine-readable reports: `git commit -m "Add ESC USB-C programming interfaces"`.

## Task 7: Reconcile USB footprints with the saved 165 mm hand placement

**Files:**

- Create: `tools/hardware/reconcile_esc_usb_footprints.py`
- Read: `hardware/esc/rev_b/reports/esc_165mm_hybrid_newtop_oldbottom_tight.kicad_pcb`
- Create: `hardware/esc/rev_b/reports/esc_165mm_usb_reconciled.kicad_pcb`
- Create: `hardware/esc/rev_b/reports/esc_165mm_usb_reconciled_drc.json`

- [ ] Write a failing fixture/test that snapshots every existing footprint's position, orientation, side, and pad geometry.
- [ ] Remove only the six now-external capacitor footprints if present.
- [ ] Add every new USB, ESD, selector, and support footprint outside `Edge.Cuts` in labeled groups.
- [ ] Preserve all existing hand-placed footprint transforms and all existing copper exactly.
- [ ] Reload the result through `pcbnew` and prove the saved output is parseable.
- [ ] Run schematic-to-PCB reference parity; require zero missing and zero stale electrical references.
- [ ] Run full KiCad PCB DRC and report exact violations and unconnected items without claiming they are fixed by this schematic change.
- [ ] Commit only the reconciliation tool, reconciled board, tests, and fresh report: `git commit -m "Reconcile ESC USB footprints with hand placement"`.

## Task 8: Build and inspect the flat Quilter handoff

**Files:**

- Modify: `tools/hardware/prepare_quilter_upload.py`
- Create/update: `hardware/esc/rev_b/quilter_upload/`

- [ ] Generalize package validation from the obsolete 150 mm/646-footprint assumptions to the verified 165 mm USB-reconciled board and fresh schematic population.
- [ ] Create a flat folder containing the PCB, root and child schematics, project, custom symbol library, symbol table, README, audit report, and `SHA256SUMS`.
- [ ] Verify there are no nested directories, stale generated files, missing hierarchy sheets, duplicate references, or checksum mismatches.
- [ ] Run final ERC, netlist audit, PCB parity, package audit, and DRC from fresh commands; record exact counts.
- [ ] Open the reconciled PCB in KiCad and the flat package directory in File Explorer for user inspection.
- [ ] Commit the package tooling and manifest only after all package checks pass: `git commit -m "Package ESC USB design for Quilter"`.

## Completion Gate

- [ ] Controller native USB-C present and electrically validated.
- [ ] One service USB-C selects exactly one of six isolated STM32 USB domains at a time.
- [ ] All seven SWD interfaces remain.
- [ ] Six external capacitors are absent from schematic, netlist, and reconciled PCB.
- [ ] Fresh ERC and netlist audit have no unexplained electrical errors.
- [ ] PCB reference parity is exact and the user's existing placement/copper is unchanged except for explicitly removed/added footprints.
- [ ] Final DRC and unconnected counts are reported exactly.
- [ ] Quilter folder is flat, complete, checksummed, and opened for inspection.
