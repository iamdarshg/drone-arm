# DA-G3: differential-pair skew, Rev B (2026-11-18)

Board `hardware/main/rev_b/main_rev_b.kicad_pcb`, sha256 `b500f5fecd22…` (same board as
DA-G1/DA-G4). KiCad 9.0.7 bundled Python. Read-only: nothing on the board was changed.

```
$KICAD_PY tools/hardware/extract_pair_limits.py --drone-arm . --out limits_declared.json
$KICAD_PY tools/hardware/diffpair_skew.py --board hardware/main/rev_b/main_rev_b.kicad_pcb \
    --limits limits_declared.json --out diffpair_skew.json --md diffpair_skew.md
$KICAD_PY tools/hardware/audit_rev_b_pcb.py --kind main --board hardware/main/rev_b/main_rev_b.kicad_pcb --output repo_audit.json
```
Raw outputs: `reports/da_g3_diffpair_skew_20261118/`.

## Correction: the repo already declares skew limits and has a skew checker

An earlier verdict said no diff-pair skew tooling exists in the repo. It does:
`tools/hardware/audit_rev_b_pcb.py` (commit `6777053`) checks four pairs on the main board
(USB connector side 0.5 mm, USB MCU side 0.5 mm, CAN connector 2.0 mm, CAN transceiver 2.0 mm).
Run on this board it **exits 2 with 5 failed checks**: all four skew checks and its
"RF routes on F.Cu without vias" check. Its skew numbers equal the track-only column below
to 0.001 mm. No skew/diff-pair rule exists in a `.kicad_dru` (there is none for this project),
and the netclass `diff_pair_gap`/`diff_pair_width` values (0.25/0.2 mm, the same for every class) are
router defaults, not limits. KiCad's DRC therefore never checks skew on this board.

## Method (new script `tools/hardware/diffpair_skew.py`)

- **Pairs**: found by net name (`_P`/`_N` tokens, trailing `P`/`N`, `DP`/`DM`/`DN`, `+`/`-`,
  `CAN…H`/`CAN…L`), by netclass (any class with `DIFF` in the name: `CAN_DIFF`), by `.kicad_dru`
  `inDiffPair()` rules (none here), and by the repo's declared limits.
- **Length**: sum of track lengths (all branches) plus via barrel length. Barrel = stackup distance
  between the outermost copper layers this net actually uses at that via stack (copper centre to
  copper centre: F→In1 0.235, F→In2 1.31, F→B 1.545 mm). Stacked microvia + buried-via chains are
  one stack. Also the **pad-to-pad** shortest copper path, which excludes stubs.
- **Delay**: each segment uses its own layer. Outer layers: microstrip ε_eff (Hammerstad) on the
  0.2 mm prepreg, εr 4.2 from the board's own stackup → ε_eff 3.04–3.17 (≈5.9 ps/mm). Inner layers
  and via barrels: ε_eff = εr = 4.2 (6.84 ps/mm). This ignores solder mask, the odd/even-mode split and
  dispersion.
- **Coupling**: centreline sampled every 0.05 mm; edge gap to the partner on the same layer.
  Uncoupled length is given at 2 × netclass gap (0.5 mm) and at 1.0 mm.

## Results

| pair | layers P / N | track P / N (mm) | vias P / N | skew track-only (mm) | skew incl. barrels (mm / ps) | pad-to-pad skew (mm / ps) | uncoupled P / N @1.0 mm (mm) | declared limit → verdict |
|---|---|---|---|---:|---:|---|---|---|
| `/USB_DP` / `/USB_DM` (U1→R10/R11) | F.Cu,**In1.Cu** / F.Cu | 56.93 / 32.83 | 2 / 0 | **24.10** | 24.57 / 194.8 | 24.27 / 192.8 | 56.93 / 32.83 (never within 1 mm) | 0.5 mm → **FAIL** |
| `USB_DP_CONN` / `USB_DN_CONN` (J1→U1) | F.Cu,In2.Cu / F.Cu,B.Cu | 15.54 / 28.79 | 2 / 2 | **13.26** | 13.73 / 69.6 | A6/A7 12.42 / 61.9; B6/B7 7.13 / 27.9 | 12.53 / 24.59 | 0.5 mm → **FAIL** |
| `/CANH` / `/CANL` (J2, D50, FL50) | F.Cu / F.Cu | 15.96 / 23.51 | 0 / 0 | **7.55** | 7.55 / 44.3 | FL50→J2 4.11 / 24.1 | 0.00 / 4.66 (min edge gap 0.578 mm) | 2.0 mm → **FAIL** |
| `U50_CANH` / `U50_CANL` (U50, R122, FL50) | F.Cu / F.Cu | 19.05 / 10.16 | 0 / 0 | **8.89** | 8.89 / 52.1 | FL50→U50 7.09 / 41.6 | 12.17 / 7.38 | 2.0 mm → **FAIL** |
| `CC_LNA_P` / `CC_LNA_N` (U40→FL40) | F.Cu / F.Cu,**In2.Cu** | 4.94 / 5.30 | 0 / 2 | 0.37 | **2.99 / 21.7** | 2.53 / 18.6 | 0.00 / 1.84 | none declared |
| `USB_DP_MCU` / `USB_DM_MCU` (U10→R10/R11) | F.Cu,In2.Cu / F.Cu,In2.Cu | 22.93 / 29.15 | 4 / 2 | 6.22 | 6.22 / 30.4 | 6.06 / 29.3 | 21.33 / 27.61 | none declared |

**All four pairs with a declared limit fail it**, on every length definition (track-only, with
barrels, pad-to-pad). The two other pairs have no declared limit; none is invented here.

Notes:
- `CC_LNA_N` is the largest *relative* case: track-only skew is 0.37 mm, but the N leg drops to
  In2 through two vias (2.62 mm of barrel) while P stays on F.Cu. 21.7 ps is about **7° of phase at
  915 MHz** (6° pad-to-pad). Track-only tools miss this, because they ignore vias. This is the
  asymmetry dsh-lead flagged in DA-G2.
- The `/USB_DP`/`/USB_DM` legs never run within 1 mm of each other on the same layer. `/USB_DP`
  takes 51.7 mm on In1, which is the `/3V3` plane layer.
- In timing terms these skews are small against the interface bit times. The RP2354's USB is full-speed,
  12 Mb/s (83 ns bit), and CAN (including CAN FD) runs at a few Mb/s at most. So failing the repo's limits is a
  layout-intent/EMC (mode-conversion) finding, not proof of a functional fault. The limits
  are the repo's own; this report does not judge whether they are the right ones.
- Sum-of-tracks counts stubs (e.g. the D50 TVS branch on CAN), so its numbers can differ from
  pad-to-pad. Both are given.

## Verdict

**DA-G3: RUN → FAIL.** The four repo-declared pair limits are exceeded by 3.8–48×. The repo's own
`audit_rev_b_pcb.py` reproduces the same four skews exactly.
