# Fab package for board sha256 `b500f5fe…` — NOT FAB-READY (DA-G4 receipt, 2026-11-18)

This package is valid **only** for `hardware/main/rev_b/main_rev_b.kicad_pcb` with sha256
`b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43` (branch
`audit/drone-arm-da-g1`, board last changed in `a6274e5`). If the board hash differs, this
package is stale: regenerate it with the commands in `MANIFEST_SHA256.json` (KiCad 9.0.7
stock `kicad-cli`, no plugins).

**The board is not fab-ready.** DA-G4 asks that the package match the board; it does not say the
board is good. Do not order from this package until the blockers below are closed and the
package is regenerated for the new board hash.

## Contents

| item | value |
|---|---|
| `gerbers/` | 26 Gerber layers (4 copper) + `main_rev_b-job.gbrjob` |
| Excellon drill | 6 files in `gerbers/`: `main_rev_b.drl` (333 hits: 305 through vias + 22 PTH + 6 NPTH pad holes) and 5 blind/buried spans (`front-in1` 14, `front-in2` 3, `in1-in2` 10, `in1-back` 4, `in2-back` 5) |
| `positions.pos` | 161 footprints, mm, both sides (165 on board minus H1–H4, which are excluded from position files) |
| `drc_receipt.json` | `kicad-cli pcb drc --severity-all --schematic-parity` on this board |
| `outline_check.json` | outline / job / drill / position / zone-fill cross-checks (`tools/hardware/da_g4_fab_check.py`) |
| `MANIFEST_SHA256.json` | sha256 of every file above and of the board, plus the commands |

## Consistency checks (all pass)

| check | result |
|---|---|
| Edge.Cuts Gerber extent (centreline) | 140.000 × 100.000 mm = board Edge.Cuts centreline |
| Edge.Cuts Gerber extent incl. 0.15 mm aperture | 140.15 × 100.15 mm = `GetBoardEdgesBoundingBox()` |
| job file `Size` | 140.15 × 100.15 mm = board bbox |
| job `LayerNumber` / `BoardThickness` / `Finish` | 4 / 1.6 mm / ENIG = board (stackup is defined in the board file: 0.2 mm prepreg, 1.04 mm core, εr 4.2) |
| Excellon hits | 369 = 341 vias + 22 PTH + 6 NPTH pad holes on the board |
| position rows | 161 = 165 footprints − 4 excluded (H1–H4) |
| zone fills | 5 zone-layers, stored fill = in-memory refill (37,419.633 mm², 0 changed), so the copper Gerbers carry current fills |

## DRC receipt

**0 violations, 0 unconnected** (`--severity-all`), **191 schematic-parity items**:
161 `footprint_symbol_mismatch`, 20 `net_conflict`, 6 `missing_footprint`, 4 `extra_footprint`.
The 0 unconnected is not evidence of completeness: the six ESC outputs were removed from the
board, so nothing is left to be unconnected (see blocker 1).

## Known blockers (why it must not be ordered)

1. **J60–J65 (ESC_PWM_1..6) are missing from the board and from the BOM.** Parity reports
   `missing_footprint` ×6, and U10 pads 43–48 are `unconnected-(U10-GPIO3x-Pad4x)` on the board
   where the schematic has `/RP2354B MCU/PWM_ESC1..6`. `hardware/rev_b_grouped_bom.csv` has no
   J60–J65. (`tools/hardware/add_main_esc_io_pcb.py` exists but targets a 130 × 74.52 mm outline
   and has not been applied to this 140 × 100 mm board.)
2. **FL50 CANH/CANL swap per schematic parity**: `/CANH` vs schematic `U50_CANL`,
   `U50_CANL` vs schematic `/CANL`, `/CANL` vs schematic `/CANH`. Which side is right depends on
   the ACT45B terminal map (not in the repo); either way board and schematic disagree.
3. **RF reference plane (DA-G2)**: the `RF_50R` traces run over the In1 `/3V3` plane, with 0% of
   their length over In1 `/GND`; `CC_LNA_N` changes to In2 (`/1V1`) while `CC_LNA_P` stays on F.Cu.
   The 50 Ω width is a microstrip number for a ground plane the board does not have there.
4. Not yet confirmed with a board house: the stack-up, the 36 blind/buried vias (5 drill spans,
   which need sequential lamination), and the RF impedance (job file asserts
   `ImpedanceControlled: true`). `RELEASE_STATUS.md` already lists these.

## The old package

`hardware/main/pcb-gerbers-1/` is **left in place and is stale**. It was plotted on 2026-03-31
from project `main` (not `main_rev_b`), and its Edge.Cuts extent is 288.40 × 204.24 mm
(sha256 and extent recorded in `outline_check.json`, `stale_package_edge_cuts`). That matches
neither this board (140 × 100 mm) nor `hardware/main/main.kicad_pcb` (75.55 × 50.55 mm bbox).
Do not use it.
