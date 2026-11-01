# Issue 21 main-board independent DRC snapshot

This snapshot independently re-runs KiCad CLI DRC on the unchanged Rev-B main board at `drone-arm` source commit `f653d8cb0cb33571db6507e5afae5fb1bb38515b`.

- Board: `hardware/main/rev_b/main_rev_b.kicad_pcb`
- Board SHA-256: `095aa3fd44285b885c2a4ca68c05633b169dd053fbc8407d2160e8585e73336b`
- Tool: KiCad CLI 9.0.2
- Command: `kicad-cli pcb drc --format json --output main-board-drc.json hardware/main/rev_b/main_rev_b.kicad_pcb`
- DRC JSON SHA-256: `d184402e53501b8059a31bdc11c5199d3fca738b167900d7bf057a22f3071de0`
- Result: 7 rule violations (4 hole-clearance, 2 silk-edge-clearance, 1 silk-overlap) and 42 unconnected items.

This DRC invocation does not refill zones or prove the change-specific paired-refill comparisons in the route reports. It independently verifies the current committed board snapshot only. The board remains incomplete and is not fabrication-ready.
