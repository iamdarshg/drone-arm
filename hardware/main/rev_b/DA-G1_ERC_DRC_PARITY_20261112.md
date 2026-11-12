# DA-G1 — ERC / DRC / schematic parity, Rev B (2026-11-12)

Host: macOS 26.5 arm64, KiCad 9.0.7 (official DMG, isolated config:
`KICAD_CONFIG_HOME` + bundled symbol/footprint tables). Board
`hardware/main/rev_b/main_rev_b.kicad_pcb`, sha256
`b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43`.

```
kicad-cli pcb drc --format json --all-track-errors --schematic-parity hardware/main/rev_b/main_rev_b.kicad_pcb
kicad-cli sch erc --format json hardware/main/rev_b/main_rev_b.kicad_sch
```

## Result

| Check | Count | Gate verdict |
|---|---:|---|
| DRC violations | 0 | pass |
| DRC unconnected items | 0 | pass |
| ERC (root sheet, all sub-sheets) | 1 warning | not 0 — see E-1 |
| Schematic parity | 191 | **FAIL** — includes electrical mismatches P-1 and P-2 |

**DA-G1: FAIL.** DRC is clean, but the board does not match the schematic.
Without the library tables configured, ERC reports 161 `lib_symbol_issues` and
parity reports the same 191; those 161 ERC items are configuration artifacts and
disappear once the bundled tables are loaded. The parity items below do not.

## Parity, classified by identity

### P-1 — FL50 CAN common-mode choke: CANH/CANL likely crossed (electrical, high severity)

Board pad nets for FL50 (`ACT45B-110-2P-TL003`, footprint
`Inductor_SMD:L_CommonModeChoke_Coilank_ACM4532`):

| Pad | Position (mm) | Board net | Schematic net |
|---|---|---|---|
| 1 | 13.175, 18.875 (top-left) | `/CAN interface/U50_CANH` | `U50_CANH` (not flagged) |
| 2 | 13.175, 21.125 (bottom-left) | `/CANH` | `/CAN interface/U50_CANL` |
| 3 | 16.825, 21.125 (bottom-right) | `/CAN interface/U50_CANL` | `/CANL` |
| 4 | 16.825, 18.875 (top-right) | `/CANL` | `/CANH` |

TDK's own SPICE model for this part (`act45b_110_2p_tl003_s.mod`, TDK product
page, "External Node Assignments") places the windings between **n1–n4** and
**n2–n3**. The schematic follows that (U50_CANH→1, CANH→4; U50_CANL→2, CANL→3).
The board instead assumes windings 1–2 and 3–4. If the footprint's pad numbers
are TDK's terminal numbers, the board connects transceiver CANH (pad 1) through
winding 1–4 to bus **CANL** (pad 4), and bus **CANH** (pad 2) through winding
2–3 to transceiver CANL (pad 3), i.e. CAN-H and CAN-L cross inside the choke.

ERC also flags this symbol: `lib_symbol_mismatch` — FL50 `L_Coupled` differs
from the `Device` library copy (E-1), which is consistent with a pin-map edit
on one side.

Open check for the owner: confirm the ACT45B land pattern numbers its terminals
the same way as the Coilank ACM4532 footprint used here (counter-clockwise from
top-left). If it does, the fix is "Update PCB from Schematic" for FL50 and
re-route of the four choke pads.

### P-2 — six ESC PWM outputs exist only in the schematic (electrical)

- `net_conflict` ×6: U10 GPIO34–39 are `/RP2354B MCU/PWM_ESC1..6` in the
  schematic but `unconnected-(U10-GPIO3x-Padxx)` on the board.
- `missing_footprint` ×6: J60–J65 (`ESC_PWM_1..6`) are in the schematic and not
  on the board.

Either the board predates the PWM connectors or they were deliberately dropped
(the drone-mech F-05 note says the ESC moved to CAN-FD, freeing six PWM channels
for servos). Owner decision: place and route J60–J65, or remove them from the
schematic and record why.

### P-3 — cosmetic / no electrical effect

- `footprint_symbol_mismatch` ×161: every one differs only by the missing library
  nickname (board `C_0805_2012Metric`, schematic `Capacitor_SMD:C_0805_2012Metric`).
  Verified programmatically: in all 161 the board name equals the schematic name
  without its `Lib:` prefix.
- `net_conflict` ×11: unconnected-pin net names that differ only by KiCad 9's
  `/`→`{slash}` escaping (U10 GPIO40–47/ADC0–7, U51 CLKO/SOF, ~INT1/GPIO1,
  ~INT0/GPIO0/XSTBY). Both sides are no-connect.
- `extra_footprint` ×4: H1–H4 mounting holes, board-only by design.

## ERC

E-1: 1 warning, `lib_symbol_mismatch` on FL50 (`L_Coupled`). See P-1.

## Next steps (not done here; no board edits in this change)

1. Owner confirms the ACT45B terminal numbering, then updates FL50 from the
   schematic and re-routes it (P-1).
2. Owner decides J60–J65 (P-2).
3. "Update PCB from Schematic" with library IDs to clear P-3, then re-run this
   report; DA-G1 passes at ERC 0 / DRC 0 / parity 0.

No fabrication-readiness or RF-performance claim follows from this report.
