# Broadband ideal-feed EM screen

On 2026-11-05 openEMS v0.0.36-142-g32c5c6b ran `model.py` in an isolated WSL
container (`local-gerber2ems:latest`, image SHA-256
`bb92165073b517d125a2872947fb3ec451fc7bbfcde02e1a66e5e30f678952de`).
It exited 0 after 10,860 timesteps, reaching -56.51 dB field-energy decay.
The model checks 992 frequencies from 0.1 to 10 GHz, including 915 MHz; the
largest reflected-plus-transmitted power ratio is 0.99424. `sparameters.csv`
and `summary.json` preserve the calculated points. Nearest frequency samples
for odd half-grid harmonics are 5 MHz away, as the summary explicitly shows.

The input geometry is only the straight 11.365 mm by 0.35 mm antenna-feed
segment of PCB SHA-256
`b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43`.
The modeled feed assumes an uninterrupted PEC ground plane and nominal FR4;
dielectric conductivity is constant and fitted at 915 MHz. This is **not** the
actual reference-plane geometry. It excludes the matching network, active
transmitter, antenna, enclosure, other conductors and realistic material
dispersion. Broadband mesh convergence has not been demonstrated. The
modeled feed's 915 MHz S11 of -38.73 dB is not the actual antenna or PCB match.

This linear model does not create transmitter harmonics, measure radiated
emissions, establish a tuned 915 MHz system or establish fabrication readiness.
See the adjacent [matching screen](../rf_tuning_screen_20261105/README.md) for
the distinction between the requested inspection range and operation at
915 MHz. A nonlinear device model with appropriate source/load conditions or
future calibrated measurements are required to establish harmonic emissions.
No physical testing is authorized by this report.

To reproduce, mount a writable directory at `/work`, copy `model.py` there
as `rf_microstrip_broadband_0p1_10.py`, and run it with the named image's
`/home/docker/opt/openEMS/venv/bin/python3`. `summarize.py` reads the generated
CSV from `/work/rf_microstrip_broadband_0p1_10_run`. Use a fresh output
directory so outputs from earlier runs cannot be mistaken for a new result.
