# Rejected broadband board-region run

This attempted 0.1–10 GHz openEMS run is **invalid** and is preserved as a
diagnostic, not RF evidence. It used the pinned Rev-B board identified in
`manifest.json`, a clipped Gerber-derived 20 mm by 14 mm region, manually
placed 50-ohm ports, nominal stackup and a conductivity approximation fitted
at 915 MHz. It omits components, RF-chip nonlinearity, antenna, enclosure,
the full return path and blind microvias. The power plane on In1 was used as
the reference conductor; that assumption is not validated.

The solver stopped at its 48,733-timestep limit before reaching its -60 dB
energy criterion. The original report incorrectly treated the software's
complex, frequency-dependent port references as real 50-ohm power-wave
references: the 89.66 voltage-wave sum in `sanity.json` is not a valid passive
power ratio. Reprocessing the saved voltages and currents with explicit real
50-ohm wave references gives a maximum sum of 3.82 and 146 of 1,001 points
above 1.05 (`postprocess_50ohm_summary.json`). This fails the expected
single-incident-port power check; port excitation and reference-plane errors
also need investigation. The unachieved energy criterion independently
prevents accepting the run.

A further calculation accounts for incident waves at both ports. Its
outgoing-to-total-incident ratio exceeds 1.05 at 84 frequencies, with a
maximum of 3.52. The extended 300,000-timestep run also failed to reach
-60 dB, and the corresponding both-incident calculation exceeds 1.05 at
88 frequencies. This calculation assumes correctly oriented, collocated port
voltage and current signals; it needs a matched-through calibration before
being treated as a physical power measurement. These checks flag inconsistent
port/model results; they do not identify the underlying cause. See
`both_incidents.json`. Extracting the
archived probes into a fresh workspace and rerunning `both_incidents.py` in
WSL reproduced those numbers. The first attempted PML sensitivity run was
stopped because its override only applied during geometry generation, which
that run skipped. `run_pml_v2.py` instead explicitly applies PML to the
solver immediately before running; that separate trial is in progress and
its result must be checked independently. Inspection of the installed wrapper
shows it sets MUR only on the solver used for geometry creation; `-s` creates
a new solver without setting its boundaries. The installed openEMS API
documents PEC as the default. Thus the original runs used the default PEC
boundary and modeled an artificial conducting enclosure. This is a plausible
source of ringing, not proof that it explains all port inconsistencies.
`boundary_audit.py` checks this against the installed code and API docs;
`boundary_audit.json` records the hashes of both inspected wrapper files.
Separate native-to-raster comparison suggests the original port coordinates
may also be shifted by roughly 0.05–0.075 mm; the corrected-boundary retry
retains those original port positions and does not resolve that concern.

A live snapshot of the corrected PML run, truncated to common probe time
3.134 ns, still has 107 frequencies above the 1.05 both-port diagnostic
threshold. The snapshot was captured before solver termination and must not
be used as the final result. `aligned_snapshot.tar.gz` preserves the signals
and provisional results (SHA-256
`a7dbe7d677b3ae38c12019322aec3ce8939dae925d87024a0faacd5889724dc8`).
Values from this run must not be used for matching, resonance, emissions or
fabrication decisions. The raw `Sx0.csv` is retained so the failure can be
inspected with `check.py`, which reproduces the original voltage-wave sum but
does not perform a valid 50-ohm passivity test. Exit code 0 from the simulation command
did not establish a valid solution.

A Hann-windowed FFT of the late-time port-voltage probes has its largest bin
near 7.35 GHz, with about 0.41 GHz bin spacing (`tail_spectrum.json`). This is
a diagnostic of residual model energy, not proof of a physical resonance or
the cause of the invalid solution. The three source probe files are included.

`geometry.xml` is the exact solver geometry (SHA-256
`a792ba9517538bb836f182f5774f529709db784e3c83a5eb30a71f48b5056126`),
`simulation.json` is the solver configuration, and `manifest.json` describes
the source and ports. `fab.tar.gz` contains the actual clipped input files
(SHA-256 `2f44bc98f9e30a3ef43c4b1889c2c3216110069488821a11979d863669be6a22`).
To reproduce the solver inputs, extract that archive in a fresh working
directory, copy `simulation.json` there, place `geometry.xml` in
`ems/geometry/geometry.xml`, and run `gerber2ems -s` with the openEMS image
identified in the adjacent broadband feed report. `prepare.py` records how
the inputs were copied from the earlier workspace ROI. `rf_port_probes.tar.gz`
contains the raw probes from both broadband and both 915 MHz runs; extract it
under `/work` to rerun the explicit 50-ohm postprocessing scripts. Its SHA-256
is `2bb94e206e4c8fc772ad77bf86d6415085368a332502a027db576ef756c5169c`.
