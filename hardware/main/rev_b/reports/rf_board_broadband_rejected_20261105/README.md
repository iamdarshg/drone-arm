# Rejected broadband board-region run

This attempted 0.1–10 GHz openEMS run is **invalid** and is preserved as a
diagnostic, not RF evidence. It used the pinned Rev-B board identified in
`manifest.json`, a clipped Gerber-derived 20 mm by 14 mm region, manually
placed 50-ohm ports, nominal stackup and a conductivity approximation fitted
at 915 MHz. It omits components, RF-chip nonlinearity, antenna, enclosure,
the full return path and blind microvias. The power plane on In1 was used as
the reference conductor; that assumption is not validated.

The solver stopped at its 48,733-timestep limit before reaching its -60 dB
energy criterion. The 1,001-point result also fails the passive energy check:
the largest reflected-plus-transmitted power ratio is 89.66 near 9.04 GHz.
Values from this run must not be used for matching, resonance, emissions or
fabrication decisions. The raw `Sx0.csv` is retained so the failure can be
independently checked with `check.py`. Exit code 0 from the simulation command
did not establish a valid solution.

`geometry.xml` is the exact solver geometry (SHA-256
`a792ba9517538bb836f182f5774f529709db784e3c83a5eb30a71f48b5056126`),
`simulation.json` is the solver configuration, and `manifest.json` describes
the source and ports. `prepare.py` records how the inputs were copied from
the earlier workspace ROI; that source ROI and the full fab export remain in
`E:\1vm\TMP`, so the committed inputs alone do not yet reproduce its creation.
A longer run was started separately with a 300,000-timestep ceiling; its
outcome must be evaluated independently.
