# DA-G2 — 915 MHz / GNSS RF path geometry audit, Rev B (2026-11-12)

Board sha256 `b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43`, KiCad 9.0.7
(`pcbnew`), sampling every 0.1 mm along each RF segment. **Geometry/analytical audit only:
no EM solve, no S-parameters, no measurement.**

## Stackup (from the board file)
F.Cu 35 µm / prepreg 0.20 mm FR4 εr 4.2 tanδ 0.02 / In1.Cu / core 1.04 mm / In2.Cu / prepreg 0.20 mm / B.Cu.

## Findings

| Net | Width | Layer | In1 under the trace | Nearest F.Cu GND pour edge |
|---|---|---|---|---|
| ANT_915, CC1121_RF, RF_LNA_IN, RF_PA_INOUT, RF_PA_OUT, RF_TR_SW | 0.35 mm | F.Cu | **100% `/3V3` zone, 0% `/GND`** | 0.25 mm |
| GNSS_RF_ANT, GNSS_RF_IN | 0.35 mm | F.Cu | **100% `/3V3` zone** (sampled ANT net) | 0.25 mm |

1. **Reference plane is 3V3, not GND (design review item).** The RF return current flows on the
   In1 3V3 pour and reaches ground only through decoupling capacitors. Usual RF practice is a
   continuous GND reference directly beneath the line, with stitching vias at transitions.
   Either move the GND zone under the RF corridor on In1 or document why 3V3 is acceptable
   (bypass placement and expected frequency of plane resonances).
2. **The lines are grounded coplanar, not microstrip.** An F.Cu GND pour runs 0.25 mm from both
   edges, which changes the impedance estimate.
3. **Impedance estimate (analytical, not an EM solve):**
   - microstrip (Hammerstad–Jensen, validated on textbook cases): 51.3 Ω; 50 Ω at 0.366 mm;
   - grounded coplanar (conformal mapping, Wadell/Simons): 52.9 Ω at 0.25 mm gap. This
     approximation is loose at wide gaps (at 10 mm it gives 61.7 Ω instead of approaching the
     microstrip value), so quote only **≈51–53 Ω**.
   Both assume the In1 plane is an RF ground; with a 3V3 reference that holds only where it is
   well bypassed. A fabricator stackup solve and a field solver are needed for sign-off (DA-G5).

## Not covered
Antenna feed geometry vs the antenna datasheet, via transitions, keep-outs, and the J40 SMA
launch. No S-parameter or matching claim.

Evidence: `return_path.json` (per-net sample coverage and gaps) and analytical calculator outputs
are kept in the orchestrator evidence folder; scripts `microstrip_hj.py`, `cpwg.py`.
