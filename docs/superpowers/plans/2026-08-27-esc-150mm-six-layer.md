# 150 mm Six-Layer ESC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Regenerate the Rev-B ESC as an exactly 150 mm square, six-layer, deterministic 3-by-2 motor-grid placement candidate.

**Architecture:** Parameterize board creation and stackup generation for six layers, replace the oversized ESC placement and copper model with compact grid-aware placement, and enforce physical invariants with an automated audit. Preserve the existing schematic/netlist and all electrical footprints.

**Tech Stack:** KiCad 9 Python API, `kicad-cli`, Python standard library, pytest-style assertions in a standalone validation script.

**Spec:** `docs/superpowers/specs/2026-08-27-esc-150mm-six-layer-design.md`

## Global Constraints

- ESC outline is exactly 150.00 mm by 150.00 mm.
- ESC copper-layer count is exactly six.
- Preserve every schematic-derived component and pad-to-net assignment.
- Place electrical components on F.Cu and within their assigned grids.
- Do not modify the flight-control PCB or the Rev-B schematics.
- Do not claim routing, current, or thermal qualification from placement success.

---

### Task 1: Add placement acceptance audit

**Files:**
- Create: `tools/hardware/validate_esc_150mm_layout.py`

**Interfaces:**
- Consumes: a generated KiCad PCB and the Rev-B XML netlist.
- Produces: `audit(board_path, netlist_path) -> dict[str, object]` and a nonzero CLI exit on failure.

- [ ] Write assertions for exact outline, six layers, footprint parity, front-only placement, outline containment, and motor-grid membership.
- [ ] Run against the current 474 mm four-layer board and verify failure.
- [ ] Keep the audit independent of the generator implementation.

### Task 2: Parameterize six-layer board and stackup creation

**Files:**
- Modify: `tools/hardware/generate_rev_b_pcbs.py`

**Interfaces:**
- Produces: `create_board(..., copper_layers=4)` and `apply_stackup(..., copper_layers=4)`.

- [ ] Add the copper-layer parameter without changing main-board behavior.
- [ ] Generate a six-layer stack containing In3.Cu and In4.Cu.
- [ ] Verify KiCad reloads the generated layer table as six copper layers.

### Task 3: Generate compact 3-by-2 placement

**Files:**
- Modify: `tools/hardware/generate_rev_b_pcbs.py`
- Generate: `hardware/esc/rev_b/esc_rev_b.kicad_pcb`

**Interfaces:**
- Produces: a deterministic 150 mm placement-only ESC PCB and six grid definitions.

- [ ] Replace the 474 mm outline and legacy placement.
- [ ] Place fixed power-stage components in six 48 mm by 55 mm cells.
- [ ] Pack motor support components inside their owning cells.
- [ ] Pack shared power/controller components in dedicated shared grids.
- [ ] Add only local placeholder power zones that cannot cross cell boundaries.
- [ ] Generate the PCB and fail on placement overflow.

### Task 4: Validate and report

**Files:**
- Generate: `hardware/esc/rev_b/reports/esc_150mm_layout_audit.json`
- Generate: `hardware/esc/rev_b/reports/esc_150mm_preroute_drc.json`

**Interfaces:**
- Produces exact geometry, layer, parity, DRC, and unconnected counts.

- [ ] Run the independent placement audit and require success.
- [ ] Run KiCad DRC with all track errors enabled.
- [ ] Inspect the generated board bounds and component population.
- [ ] Record all remaining routing and thermal work without calling the board fabrication-ready.

