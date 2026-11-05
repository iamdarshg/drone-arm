# Exploratory 915 MHz board-region comparison

These are openEMS runs on a clipped, Gerber-derived region of the board pinned
in the [broadband failure report](../rf_board_broadband_rejected_20261105/README.md).
Both used a 0.6–1.3 GHz excitation, nominal FR4 stackup, 50-ohm port
terminations and the
In1 power plane as the reference conductor. The coarser run stopped at
-60.34 dB energy and the refined run at -60.35 dB. The refined run used
2,197,008 cells and 225,810 timesteps. The substrate's constant conductivity
was fitted to loss tangent 0.02 at 915 MHz; see the loss manifests.

The original CSVs used software-derived complex port references; their
voltage-wave sums are not valid 50-ohm passive power checks. Reprocessing the
original port voltages and currents with explicit 50-ohm wave references gives
S11 -21.61 dB and -18.90 dB, and S21 -0.216 dB and -0.148 dB, for the coarse
and refined meshes respectively at 915 MHz. Their wave-power sums are 0.958
and 0.979 under a single-incident-port assumption. No mesh-convergence
criterion was specified. The sensitivity and model assumptions prevent
treating these values as a verified 915 MHz match. The installed wrapper sets
the boundary on a different solver instance from the one used to simulate;
the actual solver therefore used openEMS's default PEC boundary, creating an
artificial conducting enclosure. The reported energy decay does not correct
this boundary error.

A corrected PML-boundary run on the coarse ROI has been queued for after the
broadband PML trial. `prepare_pml.py`, `pml_config.json`, and `check_pml.py`
pin its preparation and the 50-ohm both-port diagnostic; its outcome and
energy termination remain unknown. It will still lack the antenna, matching
components and active transmitter.

The simulation omits the matching parts, powered RF devices, real antenna,
enclosure, complete return path and blind microvias; the assumed power-plane
reference has not been validated. It cannot establish transmitter harmonics,
emissions, or fabrication readiness. The raw CSVs and exact refined geometry
are preserved; `python3 verify.py` independently recomputes the original
software-normalized points. The explicit 50-ohm recalculation and its source
are recorded separately. Both checks ran in WSL against solver outputs. The
raw port probes are in `rf_port_probes.tar.gz` in the adjacent broadband
failure report; extract them under `/work` to rerun `repost_50ohm.py`.
The coarse geometry and clipped fabrication inputs are stored with the
broadband failure report; geometry hashes are in the loss manifests.
