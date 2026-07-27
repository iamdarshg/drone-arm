# Compact Six-Motor ESC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a routed and release-validated six-motor Rev-B ESC measuring no more than 200 mm by 300 mm, with six independent 60 A cells, front-side components, dual-face heatsinking, and high-speed per-leg and per-motor current sensing.

**Architecture:** Rebuild one compact 92 mm by 84 mm motor cell around three aligned half bridges, then place six translated instances in a 2-by-3 matrix inside a 196 mm by 296 mm outline. Route gate and Kelvin nets deterministically before adding local power polygons, replicate only geometry that remains electrically motor-scoped, and use KiCad DRC plus project-specific audits as release gates.

**Tech Stack:** KiCad 9 schematic/PCB formats and CLI, KiCad bundled Python `pcbnew`, project Python generators/audits, FreeRouting with `-Xmx500m`, Markdown fabrication documentation, CSV/XLSX BOM tooling.

## Global Constraints

- Board outline is at most 200 mm by 300 mm; target is 196 mm by 296 mm.
- All electrical footprints are on F.Cu; the B.Cu component count is zero.
- Six battery branches remain electrically distinct and individually fused for 60 A continuous each; no shared 360 A PCB trunk is permitted.
- Stackup is 4 oz F.Cu/B.Cu and 1.5 oz In1.Cu/In2.Cu.
- Gate resistor-to-FET route length is at most 10 mm; driver-to-resistor length is at most 35 mm; all gate routes are F.Cu with no vias.
- Every shunt Kelvin pair is F.Cu with no vias and no more than 2 mm pair skew.
- Retain three phase-leg current channels and one motor-bus current channel per motor.
- Java router heap is capped with `-Xmx500m`.
- Release requires zero ERC errors, zero DRC violations, and zero unconnected items on the filled full board.
- The control board and ESC remain separate projects and release artifacts.

---

### Task 1: Make compact-board requirements executable

**Files:**
- Modify: `tools/hardware/audit_rev_b_pcb.py`
- Create: `tools/hardware/test_rev_b_layout.py`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: a KiCad `pcbnew.BOARD` loaded from a generated ESC PCB.
- Produces: `board_geometry(board) -> dict[str, object]`, `footprint_side_failures(board) -> list[str]`, and audit JSON checks for outline, component side, cell separation, gate routes, and Kelvin routes.

- [ ] **Step 1: Write failing unit tests for board size and footprint side**

```python
def test_compact_esc_outline_and_front_only():
    board = pcbnew.LoadBoard(str(ESC_BOARD))
    geometry = board_geometry(board)
    assert geometry["width_mm"] <= 200.0
    assert geometry["height_mm"] <= 300.0
    assert footprint_side_failures(board) == []
```

- [ ] **Step 2: Run the test and verify the current 474 mm outline fails**

Run: `& 'C:\Program Files\KiCad\9.0\bin\python.exe' tools/hardware/test_rev_b_layout.py`

Expected: FAIL reporting an ESC width near 474 mm.

- [ ] **Step 3: Implement geometry and front-side checks**

Add reusable functions to `audit_rev_b_pcb.py` that read `GetBoardEdgesBoundingBox()`, convert nanometres with `pcbnew.ToMM`, and report any non-mounting-hole footprint whose side is not `F_Cu`.

- [ ] **Step 4: Correct the broad ignored-sheet rule**

Replace `.gitignore` entry `motor*` with narrow generated-output patterns that do not hide `hardware/esc/rev_b/motor_1.kicad_sch` through `motor_6.kicad_sch`.

- [ ] **Step 5: Run the audit test and commit the red test harness**

Run: `& 'C:\Program Files\KiCad\9.0\bin\python.exe' tools/hardware/test_rev_b_layout.py`

Expected: helper tests pass and compact-outline assertion remains an intentional failure until Task 3.

Commit: `git commit -m "test: enforce compact ESC layout constraints"`

### Task 2: Normalize and validate modular ESC schematics

**Files:**
- Modify: `tools/hardware/generate_rev_b_schematics.py`
- Modify: `hardware/esc/rev_b/esc_rev_b.kicad_sch`
- Add: `hardware/esc/rev_b/motor_1.kicad_sch`
- Add: `hardware/esc/rev_b/motor_2.kicad_sch`
- Add: `hardware/esc/rev_b/motor_3.kicad_sch`
- Add: `hardware/esc/rev_b/motor_4.kicad_sch`
- Add: `hardware/esc/rev_b/motor_5.kicad_sch`
- Add: `hardware/esc/rev_b/motor_6.kicad_sch`
- Modify: `tools/hardware/validate_rev_b_netlists.py`

**Interfaces:**
- Consumes: existing validated component/net definitions and the modular-sheet implementation available on `origin/agent/rev-b-fab-ready`.
- Produces: a root ESC sheet containing six motor sheets plus controller and power sheets, and a netlist with the same required populations and motor-scoped nets.

- [ ] **Step 1: Add failing hierarchy assertions**

Extend `validate_rev_b_netlists.py` to require six distinct motor sheet paths, one controller sheet, one auxiliary-power sheet, and absence of control-board source paths.

- [ ] **Step 2: Export the current netlist and verify the hierarchy check fails if sheets are hidden or flattened**

Run: `& 'C:\Program Files\KiCad\9.0\bin\kicad-cli.exe' sch export netlist -o hardware/esc/rev_b/reports/esc_rev_b_netlist.xml hardware/esc/rev_b/esc_rev_b.kicad_sch`

Run: `& 'C:\Program Files\KiCad\9.0\bin\python.exe' tools/hardware/validate_rev_b_netlists.py hardware/esc/rev_b/reports/esc_rev_b_netlist.xml hardware/main/rev_b/reports/main_rev_b_netlist.xml --output hardware/esc/rev_b/reports/rev_b_connectivity_audit.json`

- [ ] **Step 3: Selectively apply only the modular-sheet generator changes from `origin/agent/rev-b-fab-ready`**

Do not copy its PCB or validation reports. Reconcile generated UUIDs and sheet paths with current main and preserve the verified DRV8353, STM32G431, shunt, INA296, isolation, protection, and enable connectivity.

- [ ] **Step 4: Regenerate, export, and validate schematics**

Expected: 36 IPTC014N10NM5, 6 DRV8353S, 6 STM32G431, 24 CSS4J shunts, 6 INA296A2, six distinct cells, and no cross-cell BATP/BATN nets.

- [ ] **Step 5: Run ERC and commit**

Run: `& 'C:\Program Files\KiCad\9.0\bin\kicad-cli.exe' sch erc --exit-code-violations -o hardware/esc/rev_b/reports/esc_rev_b_erc.rpt hardware/esc/rev_b/esc_rev_b.kicad_sch`

Expected: exit 0 with zero ERC errors.

Commit: `git commit -m "refactor: modularize six ESC motor sheets"`

### Task 3: Generate the 196 mm by 296 mm placement

**Files:**
- Modify: `tools/hardware/generate_rev_b_pcbs.py`
- Modify: `tools/hardware/test_rev_b_layout.py`
- Regenerate: `hardware/esc/rev_b/esc_rev_b.kicad_pcb`

**Interfaces:**
- Consumes: `place_esc_motor(footprints, motor, x0, y0)` and validated netlist.
- Produces: `compact_esc_origins() -> dict[int, tuple[float, float]]` and a board with six cells bounded by 92 mm by 84 mm each.

- [ ] **Step 1: Add failing placement assertions**

Assert exact 196 mm by 296 mm outline, six non-overlapping cell rectangles, all footprints within the outline/courtyard margins, rear component count zero, and perimeter placement for power connectors.

- [ ] **Step 2: Run tests and verify failure on the old placement**

Expected: failures for outline width, cell dimensions, and component bounds.

- [ ] **Step 3: Split placement data from placement operations**

Create explicit local-coordinate maps for power train, driver/ADC bay, MCU/isolation bay, and perimeter connectors. Keep each cell's local coordinates inside `[0, 92] x [0, 84]`.

- [ ] **Step 4: Implement the 2-column by 3-row matrix**

Use 4 mm board-edge margins and 4 mm inter-cell gaps. Place the supervisor/auxiliary-power strip in the remaining edge corridor without crossing cell power copper.

- [ ] **Step 5: Regenerate and run placement tests**

Run: `& 'C:\Program Files\KiCad\9.0\bin\python.exe' tools/hardware/generate_rev_b_pcbs.py`

Run: `& 'C:\Program Files\KiCad\9.0\bin\python.exe' tools/hardware/test_rev_b_layout.py`

Expected: all geometry, side, bounds, and separation tests pass.

- [ ] **Step 6: Run pre-route DRC and commit**

Run: `& 'C:\Program Files\KiCad\9.0\bin\kicad-cli.exe' pcb drc --exit-code-violations --all-track-errors -o hardware/esc/rev_b/reports/esc_rev_b_preroute_drc.json --format json hardware/esc/rev_b/esc_rev_b.kicad_pcb`

Review every non-routing violation; courtyard, clearance, board-edge, and malformed-zone violations must be zero.

Commit: `git commit -m "feat: compact ESC into 196 by 296 millimetres"`

### Task 4: Route and verify one sensitive motor cell

**Files:**
- Create: `tools/hardware/route_esc_sensitive.py`
- Modify: `tools/hardware/audit_rev_b_pcb.py`
- Modify: `tools/hardware/test_rev_b_layout.py`
- Modify: `tools/hardware/route_rev_b_sections.py`

**Interfaces:**
- Consumes: a placed, zone-free PCB and motor number.
- Produces: deterministic F.Cu routes for 12 split gate nets, eight Kelvin nets, amplifier/ADC paths, PWM/SPI/fault/enable paths, plus translation metadata for replication.

- [ ] **Step 1: Add failing gate-segment and Kelvin assertions**

Test resistor-to-FET and driver-to-resistor segments separately using pad endpoints rather than total branched-net length. Require F.Cu, zero vias, 10/35 mm limits, all 24 Kelvin pairs present, F.Cu, zero vias, and 2 mm maximum skew.

- [ ] **Step 2: Run the audit on the generated board and verify missing-route failures**

- [ ] **Step 3: Implement deterministic routing primitives**

Implement `add_track(board, net, points_mm, width_mm, layer)` and pad-aware endpoint lookup. Reject a route if it crosses a declared switch-node or isolation keepout.

- [ ] **Step 4: Route Motor 1 in priority order**

Route local source returns, gates, shunt Kelvin pairs, bus-shunt Kelvin pair, CSA/INA outputs to filters, bootstrap/charge-pump paths, PWM, SPI, fault, and enable. Do not add power zones yet.

- [ ] **Step 5: Run sensitive-route audits and DRC**

Expected: all Motor 1 gate and Kelvin checks pass with zero geometric violations in the cell.

- [ ] **Step 6: Commit**

Commit: `git commit -m "feat: route ESC gate and current-sense corridors"`

### Task 5: Replicate sensitive routing and add local power copper

**Files:**
- Modify: `tools/hardware/route_rev_b_sections.py`
- Modify: `tools/hardware/generate_rev_b_pcbs.py`
- Modify: `tools/hardware/audit_rev_b_pcb.py`
- Regenerate: `hardware/esc/rev_b/esc_rev_b.kicad_pcb`

**Interfaces:**
- Consumes: validated Motor 1 geometry and `mapped_net_name(source, motor)`.
- Produces: six identically constrained sensitive-route sets and cell-local F.Cu/In1.Cu/In2.Cu/B.Cu polygons/via fields.

- [ ] **Step 1: Add failing replication and cell-isolation tests**

Require identical route fingerprints after translation, motor-scoped target nets, no foreign motor nets inside a cell, and no shared BATP/BATN polygon between cells.

- [ ] **Step 2: Replicate routes with per-motor net mapping**

Translate Motor 1 tracks to Motors 2–6 while preserving widths, layers, endpoints, and no-via policy. Abort on missing or ambiguous target nets.

- [ ] **Step 3: Add power polygons after sensitive routing**

Use In1.Cu for local BATP, B.Cu for local BATN, In2.Cu for phase paths, and minimal F.Cu device landings. Clip zones around quiet corridors, isolation boundaries, and heatsink fasteners.

- [ ] **Step 4: Add and validate power-via fields**

Keep holes clear of TOLT gates and Kelvin pads. Record via drill, annulus, count, and fabricator assumptions in generated metadata.

- [ ] **Step 5: Fill zones and run full sensitive-route audit**

Expected: all 72 gate nets and all 24 Kelvin pairs pass; six cell-isolation checks pass.

- [ ] **Step 6: Commit**

Commit: `git commit -m "feat: add isolated 60 amp ESC power cells"`

### Task 6: Complete remaining routing under the 500 MB heap cap

**Files:**
- Modify: `tools/hardware/route_rev_b_sections.py`
- Create: `tools/hardware/route_compact_esc.ps1`
- Regenerate: `hardware/esc/rev_b/esc_rev_b.kicad_pcb`
- Create: `hardware/esc/rev_b/reports/esc_rev_b_router_summary.json`

**Interfaces:**
- Consumes: sensitive-routed, power-zone-free or selectively stripped board.
- Produces: completed non-sensitive routing merged without changing locked sensitive tracks.

- [ ] **Step 1: Export a Motor 1 DSN with sensitive routes locked and power planes filtered**

- [ ] **Step 2: Run FreeRouting with the heap cap**

The PowerShell wrapper must invoke Java with `-Xmx500m`, capture the command line, exit code, pass count, unrouted count, and peak reported heap in JSON, and fail if the cap is absent.

- [ ] **Step 3: Import the candidate and reject sensitive-route changes**

Compare sensitive-route fingerprints before and after import. Any moved gate/Kelvin track rejects the candidate.

- [ ] **Step 4: Replicate completed cell routing and route supervisor/power strip**

- [ ] **Step 5: Iterate until the filled full board reports zero unconnected items**

Do not accept an autorouter plateau as completion. Repair residual routes deterministically or adjust placement while preserving prior gates.

- [ ] **Step 6: Commit**

Commit: `git commit -m "feat: complete compact ESC routing"`

### Task 7: Run electrical, thermal, and fabrication validation

**Files:**
- Modify: `docs/hardware/REV_B_ARCHITECTURE.md`
- Modify: `docs/hardware/VALIDATION_MATRIX.md`
- Create: `docs/hardware/ESC_REV_B_THERMAL_MECHANICAL.md`
- Create: `hardware/esc/rev_b/reports/esc_rev_b_layout_audit.json`
- Create: `hardware/esc/rev_b/reports/esc_rev_b_final_drc.json`
- Update: `hardware/esc/rev_b/reports/esc_rev_b_erc.rpt`

**Interfaces:**
- Consumes: filled release PCB and authoritative component/reference-design constraints.
- Produces: reproducible pass/fail reports plus explicit bench-validation items that cannot be proven from CAD.

- [ ] **Step 1: Run netlist connectivity audit and ERC**

Expected: connectivity PASS and zero ERC errors.

- [ ] **Step 2: Run KiCad DRC on the filled board**

Expected: zero violations and zero unconnected items.

- [ ] **Step 3: Run the project-specific layout audit**

Expected: outline, component side, cell separation, gate, Kelvin, RF/isolation, track, and via checks all pass.

- [ ] **Step 4: Document calculable limits and mandatory bench gates**

Record current-density/voltage-drop assumptions, MOSFET loss bounds, shunt dissipation, connector/fuse ratings, TOLT front interface, insulated rear interface, and required double-pulse/thermal tests. Do not label unperformed physical tests as passed.

- [ ] **Step 5: Commit**

Commit: `git commit -m "docs: validate compact ESC fabrication design"`

### Task 8: Recalculate INR cost and build the clean release archive

**Files:**
- Modify: `docs/hardware/REV_B_BOM.csv`
- Create or update: `docs/hardware/costing/REV_B_COSTING_INR.xlsx`
- Create: `tools/hardware/build_rev_b_release.ps1`
- Create: `release/REV_B_RELEASE_MANIFEST.json`
- Create: `release/drone-arm-rev-b.zip`

**Interfaces:**
- Consumes: validated ESC/control sources, netlists, BOM, fabrication outputs, and current supplier prices.
- Produces: itemized ESC/control cost in INR and a deterministic archive containing distinct board folders.

- [ ] **Step 1: Regenerate BOM and identify trivial substitutions**

Substitute only passives with verified equivalent footprint, voltage, temperature, tolerance, power/ripple, parasitics, lifecycle, and availability. Keep safety, gate-drive, sensing, isolation, RF, protection, and power-semiconductor substitutions subject to engineering review.

- [ ] **Step 2: Capture current INR pricing with source, quantity break, date, tax, and shipping assumptions**

- [ ] **Step 3: Generate separate ESC and control-board fabrication folders**

Include schematics, PCB sources, Gerbers, drills, IPC-356/netlist, BOM, pick-and-place, reports, mechanical/thermal notes, and README.

- [ ] **Step 4: Build and verify the ZIP**

Extract to a temporary verification directory, compare every file hash with the manifest, and ensure no router scratch files, caches, locks, backups, or intermediate candidates are included.

- [ ] **Step 5: Run final verification and commit**

Re-run ERC, DRC, layout audits, archive hash verification, and `git diff --check` immediately before the completion claim.

Commit: `git commit -m "release: package validated Rev-B hardware"`

- [ ] **Step 6: Push only after both requested board milestones are genuinely complete**

Push the control-board completion commit first, then the ESC completion/release commits to `main`, matching the requested ordering. Do not push an incomplete board or a report with suppressed failures.
