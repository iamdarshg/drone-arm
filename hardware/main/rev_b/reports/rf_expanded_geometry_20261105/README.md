# Expanded RF region: native via import experiment

The original clipped model imported zero vias. Its mixed KiCad drill export
used absolute coordinates with negative Y, while the installed gerber2ems
drill parser accepts only positive coordinates. This experiment copies the
exported board copper into a 40 by 33 mm region, then builds a separate
positive-coordinate simulation drill file from native KiCad through-vias on
the pinned board (`b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43`).
It does not modify the PCB or fabrication files.

`native_via_manifest.json` records 36 selected through-vias, including 28
ground and one 3V3 via, their original and transformed positions, drills and
nets. The generated geometry contains 36 via and 36 via-filling primitives,
versus zero before conversion; both geometry audits are preserved. The
transform uses the port-coordinate convention from the earlier model and is
not independently verified. A coarse copper-pixel check finds copper at each
candidate location on both outer layers, but also finds copper after shifts of
0.15 mm, so it **does not establish accurate alignment**.

`evidence.tar.gz` contains the generated geometry, simulation inputs and
audits (SHA-256
`75cfc3dde1e6a91f68c9caf3ec29e0e02a248aae6b17d218220899fdf3077b5b`).
The scripts document the preparation and native selection. The full source
Gerbers are in the neighboring broadband failure report's `fab.tar.gz`.

No EM solve was performed on this geometry. Component through-holes,
blind/buried vias, components, antenna, full return path and enclosure remain
outside the model; via plating, actual layer connections and mesh resolution
need validation. The geometry has not yet had the dielectric-loss patch
applied. This is not a tuned-radio, harmonic-emissions or fabrication result.
