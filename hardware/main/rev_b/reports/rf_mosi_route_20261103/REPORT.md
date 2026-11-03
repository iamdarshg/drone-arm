# RF MOSI route closeout

The previous main board (commit 84dfda3, board SHA-256 7923ad03efefebfba5251be189821cd3ed78e588c92cb7c62774082a207ed41d) had one open net, /RF_MOSI, and zero DRC violations. The final candidate routes /RF_MOSI and makes the local /3V3 escape; the candidate passes KiCad 9.0.2 DRC with zero unconnected items and zero violations. All 657 footprint pad assignments, positions, and sizes match the route input and candidate.

Compared with previous main, copper was also adjusted on /3V3 (+2/-5 segments), /GNSS_RX (+17/-4), /RF_MISO (+44/-19), and /RF_SCLK (+3/-5) to accommodate the local fanout. The candidate makes no footprint or pad changes. source.drc.json records the pinned route input's one open and two dangling-track warnings, and candidate.drc.json records the clean final DRC.

The deterministic replay input source.kicad_pcb is pinned at SHA-256 e73000b8edbb97769f9436f48b41fd33304c518db2bf9bf047b1120717e1664f. route_mosi.py adds the lower /3V3 bridge and removes three obsolete dangling spur segments. verify.py replays this exact source-to-candidate step, compares native copper geometry and all footprint pads, and reruns DRC with the pinned project rules. The source is an intermediate board snapshot carrying the local RF/GNSS preparation routes; it is not the previous main board. The prior main board snapshot and its accepted GPIO2 evidence remain available in repository history and rf_gpio2_route_20261103.

This is an electrical-connectivity milestone, not fabrication approval. Confirm the 4-layer stack-up, microvia construction/fill/cap, via-in-pad process, annular geometry, RF impedance, and board-house tolerances before fabrication. No physical testing or manufacturing-readiness claim is made.
