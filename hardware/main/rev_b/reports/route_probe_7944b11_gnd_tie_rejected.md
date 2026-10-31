# Rejected `/GND` plane tie via at the board corner

This test added one 0.6/0.3 mm F.Cu/B.Cu through-via at `(10,10)` where the filled front and back GND zones overlap. It was checked against source `7944b112e05897d26d3dd6de60d9bf7e099a7816` with KiCad 9.0.2; both boards were explicitly zone-filled with pcbnew `ZONE_FILLER.Fill(board.Zones())` and run through matching-project CLI DRC `--severity-all`.

- Source board SHA-256: `79ad895d0c525f15e949773f79808c77164fd09495c8d576f6b80390bee4baee`
- Matching project SHA-256: `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`
- Baseline JSON SHA-256: `65b0b6071d3bff1fccca44570532111911b62028dd468888476fea18e20a2f56`; 7 violations, 44 unconnected items.
- Candidate board SHA-256: `95ad766d8a6833d75920aa76f5c4780502ef3546b8f366b14ddcc0bc3fe7b757`
- Candidate JSON SHA-256: `e6bde8611b34393b6f5413cea8d1ca083c04168aed2c40d88a3befb563d80f46`; 7 violations, 44 unconnected items; all seven signatures unchanged.

Rejected: the via did not join the reported F.Cu/B.Cu GND-zone pair in native DRC connectivity, and the unconnected count did not improve. No board change was promoted.
