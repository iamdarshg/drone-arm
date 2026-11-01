# Rejected Rev-B ground and BARO route probes

Each baseline and candidate was explicitly zone-refilled with KiCad pcbnew 9.0.2 `ZONE_FILLER.Fill(board.Zones())`, then checked with KiCad CLI 9.0.2 `pcb drc --severity-all --format json` using a same-stem copy of the matching project.

## U10.62 `/GND` via-in-pad

Source commit `a1f214d26921eba633dce8dbe3c8767fe787a2e0`; source board SHA-256 `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee`; matching project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. A 0.25/0.10 mm through-via was placed in U10.62 at `(62.4,45.05)`. Baseline DRC: 7 violations / 44 unconnected, JSON SHA-256 `ecc0ac1f43d490b8a4e8ed7856db55c0623888193b982d0b66d0679f32f769cc`. Candidate board SHA-256 `87d47fadb1449c2e6a001ffbaf2c537565fd65cd85c6bb2889a3cfb964c962ba`; DRC: 10 violations / 43 unconnected, JSON SHA-256 `2f2dd76abd5d446bc9b9edaaf64db7d3043817411d76fec267570a0ccbca2c2c`. Rejected: the via reduces one open but violates the project's 0.6 mm minimum via diameter, 0.2 mm minimum drill, and 0.1 mm annular width.

## `/BARO_SDA` In2.Cu span

Source commit `7dd81fc8d597883acab9c923ceff024bd5c49ac3`; source board SHA-256 `2bcb4d6ba0293d280306393d070ccc16a0b6755d0d34b77cf89aefac7d4e638a`; matching project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`. The disposable route used 0.6/0.3 mm through-vias at `(52.8,49)` and `(71,84.6)`, with 0.20 mm F.Cu escapes from U10.8 and the existing `/BARO_SDA` stub and a straight In2.Cu span between vias. Baseline DRC: 7 violations / 43 unconnected, JSON SHA-256 `f32af28139cff03c94614a5145c14049c19f31376dcf1f9c4b7e5e463b01dd20`. Candidate board SHA-256 `60ff3b3e53195a99a6099463dc54cd8860e703193e3fdad0064ab2a9263da6ef`; DRC: 20 violations / 43 unconnected, JSON SHA-256 `fadae90bb887cb0d014d5660a978fd582ce25803ae17be8e6c048aa45d6e2982`. Rejected: it does not reduce the unrouted count and adds 13 DRC signatures. Neither candidate was promoted.
