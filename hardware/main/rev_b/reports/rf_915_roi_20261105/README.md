# Exploratory 915 MHz board-region comparison

These are openEMS runs on a clipped, Gerber-derived region of the board pinned
in the [broadband failure report](../rf_board_broadband_rejected_20261105/README.md).
Both used a 0.6–1.3 GHz excitation, nominal FR4 stackup, 50-ohm ports and the
In1 power plane as the reference conductor. The coarser run stopped at
-60.34 dB energy and the refined run at -60.35 dB. The refined run used
2,197,008 cells and 225,810 timesteps. The substrate's constant conductivity
was fitted to loss tangent 0.02 at 915 MHz; see the loss manifests.

At 915 MHz, S11 was -9.38 dB on the coarse mesh and -8.71 dB on the refined
mesh; S21 was -2.95 dB and -2.48 dB respectively. Reflected plus transmitted
power was 0.623 and 0.700. These checks are limited to this frequency, and no
mesh-convergence criterion was specified. The changes on refinement and the
model assumptions prevent treating these figures as a verified 915 MHz match.

The simulation omits the matching parts, powered RF devices, real antenna,
enclosure, complete return path and blind microvias; the assumed power-plane
reference has not been validated. It cannot establish transmitter harmonics,
emissions, or fabrication readiness. The raw CSVs and exact refined geometry
are preserved; `python3 verify.py` independently recomputes the reported
915 MHz points. The check was run in WSL against the original solver outputs.
The coarse geometry and clipped fabrication inputs are stored with the
broadband failure report; geometry hashes are in the loss manifests.
