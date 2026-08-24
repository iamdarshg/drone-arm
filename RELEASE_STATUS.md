# PCB Final Release — Status & Remaining Gates

Branch: `codex/pcb-final-release`
Date: 2026-08-24
KiCad: 9.0.7 (CLI at C:\Program Files\KiCad\9.0\bin\kicad-cli.exe)

## What this branch delivers so far

### Completed
1. **Fresh ground truth** for both orderable boards:
   - Main Rev-B: ERC **0 violations**; DRC **5 violations** (silk_over_copper x1, track_dangling x4), **56 unconnected items**
   - ESC Rev-B: ERC **0 violations**; DRC was 233 violations + 499 unconnected
2. **ESC real copper shorts eliminated**:
   - M1_BATN via shorting M1_PHASE_B on In2.Cu (3 vias removed)
   - M1_BATN/5V pad-to-pad short at C1160/C1131 (C1160 relocated)
   - Stray no-net track+via under crystal Y1101 removed
   - Redundant nested M1_BATN zone removed; duplicate-zone priority conflict resolved
   - Generator geometry defects repaired: In1.BATP planes extended (+8/-8 mm) to reach via lattices; F.Cu BATP islands raised 2 mm to swallow top via rows; F.Cu BATN charging-pump rects widened to cover their grids; B.Cu BATN polygon/rect gaps closed with distinct priorities
   - Result: **233 -> ~150 violations**, all remaining are dangling-via warnings + silk issues, zero shorts/clearance/hole errors
3. **Motor contract consumed**: hardware/motor_release/motor_interface.json rev A is a **12S / 50.4 V max** contract (not the stale 6S assumption). Full audit in hardware/esc/rev_b/reports/electrical_audit_vs_contract.json:
   - IPTC014N10NM5 (100 V FETs): PASS
   - DRV8353S gate driver: PASS
   - Shunt/sense chain matches contract ±140 A @ 10 mV/A exactly: PASS
   - Bulk caps 820 uF/100 V: PASS
   - TPS70933 LDO: fed from isolated 5 V rail, not battery: PASS (initial concern resolved)
   - SM8S51A TVS: only 0.6 V standoff headroom vs 50.4 V bus — flagged CHECK, consider SM8S58A on re-spin
4. **Pin consistency**: automated test tests/test_pin_consistency.py verifies firmware GPIO map vs schematic netlist — all 40 RP2354B GPIO nets match, 17/17 repo tests pass.
5. All work committed and pushed to origin.

### Remaining gates (NOT release-ready until these pass)

#### A. ESC Rev-B (hardware/esc/rev_b)
- ~150 via_dangling warnings: BATP/BATN stitching vias whose fills don't yet reach them after geometry repairs. Need either fill re-pour iteration or explicit stub tracks.
- **499 unconnected items**: DGND x76, per-cell BATN x55 each (M2/M3/M4/M6 cells need same controller-strip F.Cu landing zones as M1 got), 3V3 x54, 5V x19, AUX_GND x19, plus small counts.
- Silk edge clearance x4, silk over copper x4.
- Target: 0 DRC violations, 0 unconnected.

#### B. Main Rev-B (hardware/main/rev_b)
- 5 DRC violations: 4 track_dangling + 1 silk_over_copper.
- **56 unconnected items**, mostly CAN_GND/CAN_5V x6 each (J60-J65 headers from commit 04787d7), 1V1/3V3 x4 each, RF nets.
- Target: 0 DRC violations, 0 unconnected.

#### C. Fabrication outputs (not started)
- release/main/ and release/esc/: Gerbers, drill, BOM, CPL, schematic PDF, board drawing, stackup, assembly drawing, ERC/DRC reports, README, SHA256 manifest, STEP for both assemblies.
- CAM review of Gerbers before handoff.

#### D. First-article bring-up plan (not started)
- docs/bring_up_main_rev_b.md and docs/bring_up_esc_rev_b.md per original brief.

## How to continue

1. For each remaining cell (M2-M6), replicate fix_m1_batn_strip.py's approach: add F.Cu BATN landing zones under controller/bootstrap-cap pads, stitched into the B.Cu plane with vias placed clear of the phase-current via lattices.
2. Route or zone-fill the DGND, 3V3, 5V, AUX_GND islands.
3. Re-run tools/hardware/refill_zones.py then kicad-cli pcb drc after every change group.
4. When both boards report 0/0, run fabrication exports (see KiCad CLI: pcb export gerbers, pcb export drill, sch export pdf, pcb export step).
5. Generate SHA256 manifest and write README per release directory.

