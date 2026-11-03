# RF GPIO2 routing progress

This change closes `/RF_GPIO2` on the Rev-B board. The source at commit `a007a13d1b3828fd9fcb128ade1af097c901b3d0` had two unconnected items (`/RF_GPIO2` and `/RF_MOSI`) and zero DRC violations. The routed candidate keeps all 657 footprint pad assignments and locations unchanged, has zero DRC violations, and leaves only `/RF_MOSI` open.

The replay source is `prep.kicad_pcb` (SHA-256 `19f3405fee98c1da4a156c6db314f16d2fac410a064ccc927a9e7ce96b43890d`). The deterministic route script `route_gpio2.py` is pinned to that board and adds `/RF_GPIO2`; a semantic copper comparison confirms that its replay matches the committed board. The complete candidate SHA-256 is `7923ad03efefebfba5251be189821cd3ed78e588c92cb7c62774082a207ed41d`.

The preparation board also carries earlier local copper changes on `/HGM`, `/RF_GPIO0`, `/RF_MISO`, and `/SENS_SCLK`. Comparing the original main board to this candidate reports 144 added and 34 removed copper items across those nets and `/RF_GPIO2`; no footprint or pad-map changes occurred.

The `/RF_GPIO2` path uses a 0.30/0.10 mm F–In2 microvia at U10.22, a 0.30/0.10 mm In2–B microvia, a 0.60/0.30 mm through-via, and a 0.30/0.10 mm F–In1 microvia at U40.4. The two microvias at component pads require board-house confirmation for via-in-pad processing, fill/cap, annular geometry, stack-up, and RF performance. This is a routing-progress candidate; it is not fabrication-ready.

Run the independent checks from the repository root with KiCad 9.0.2 and its Python bindings installed:

```sh
/usr/bin/python3 hardware/main/rev_b/reports/rf_gpio2_route_20261103/verify.py
```

The verifier checks source, prep, script, and candidate hashes; replays the route; compares exact copper geometry and pad maps; and reruns native DRC on both baseline and candidate. The pinned reports are `baseline.drc.json`, `candidate.drc.json`, and `verification.json`.
