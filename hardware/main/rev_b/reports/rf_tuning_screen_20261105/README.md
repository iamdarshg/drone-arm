# Exploratory 915 MHz matching screen

This is an ideal lumped-circuit calculation, not an EM simulation or a measured
match. It is pinned to `main_rev_b.kicad_pcb` at commit `a6274e5`, SHA-256
`b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43`.
The calculation was run on 2026-11-05 inside a container in WSL using Python;
`sweep.csv` and `summary.json` are the outputs. It checks conservation of
power for its lossless network at every point.

The model uses ideal 50-ohm ports at MATCH_A and ANT_915 and sweeps a
**hypothetical** adjustable C96 from 0.10 to 5.00 pF. At 915 MHz, the best
point in this restricted sweep is 4.24 pF, with S11 -4.66 dB. This result
does not predict the real board match: the model omits the CC1190 source
impedance, the receive branch, antenna, PCB fields, components' parasitics,
power handling and tuning bias. It does not demonstrate that 4.24 pF is an
appropriate hardware value. C96 is currently an unpopulated fixed-component
footprint, not a variable capacitor. C94 is also an unpopulated matching
position. Neither automatic nor user-adjustable tuning is implemented.

The CC1190 specifies 850–950 MHz operation; the CC1121 specifies selected
bands only. A matching component cannot turn this RF chain into a continuously
operating 0.1–10 GHz radio. See the [CC1190 datasheet](https://www.ti.com/lit/ds/symlink/cc1190.pdf)
and [CC1121 datasheet](https://www.ti.com/lit/ds/symlink/cc1121.pdf). A
wideband requirement requires a separate architecture and specified frequency,
power, antenna, tuning-control and performance requirements.

To reproduce in an isolated Linux environment, place the exact PCB at
`/work/main_rev_b_source.kicad_pcb` and this script at
`/work/rf_tuning_ideal.py`, ensure `/work/rf_tuning_ideal` does not exist,
and run `python3 /work/rf_tuning_ideal.py`. The script checks the board hash
before creating its output directory. Actual matching requires the intended
antenna and installed assembly, appropriate component models and a defined
verification method; this calculation does not approve fabrication or testing.
