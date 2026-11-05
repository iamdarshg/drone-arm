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

A second check compared 20,000 fixed-seed KiCad ground-zone samples with the
rasterized F.Cu image. The best match among 0.025 mm increments was shifted
0.075 mm in X and 0.05 mm in Y relative to the first transform; other-net
copper and pixelization limit this comparison. Inspecting the rasterizer showed
that it crops to the edge center. Version 2 uses the edge-centered transform
`x_local = x_board - 100`, `y_local = 78 - y_board` and shifts its port
positions accordingly. It retains all 36 through-vias; precise port and via
alignment still needs verification. See `zone_alignment.json`.

The native-board port centerline check sampled 11 points along each proposed
1 mm port and found all 22 on the target ANT_915 F.Cu track. This establishes
only centerline inclusion in source CAD; it does not establish correct
reference-terminal connection, clearance, raster alignment or calibrated
port behavior. See `port_native_alignment.json`.

`v2_inputs.tar.gz` preserves the version-2 geometry and the loss-adjusted
copy prepared for a queued 915 MHz PML-boundary trial (SHA-256
`d2855fce1a3a6be01ebf4700ea4b11321093b6d2cf2d007031e499c2763b8a11`).
That trial reached the energy criterion after 103,599 timesteps, but openEMS
reported 21 unused via-metal primitives. It also stopped before the reported
108,890-timestep excitation length, and its 300,000-step cap was below three
times that length. Its numerical 50-ohm port check did
not flag an energy excess, but the missing via metal invalidates the intended
return-path geometry. The run is rejected as an RF result. `finished_915.tar.gz`
preserves the raw probes and calculations (SHA-256
`72e883c2ddc4ac581104dad99c2ea034fb9c6e34884380e98740fb97ac197857`).

To investigate the unused barrels, `solid_via_experiment.py` retains the
36 via outer boundaries but replaces their hollow interiors with solid PEC.
That is an explicitly approximate electromagnetic sensitivity model, not the
physical plated-hole geometry. A 200-timestep setup trial reported no unused
via primitives, compared with 21 in the hollow-barrel run. Its postprocessing
exited with an index error because the intentionally short run did not produce
enough probe samples; it provides no RF result. Both transcripts are saved.
The first full solid-via run was stopped early after discovering the
premature-stop issue. A replacement 400,000-step PML run is in progress with
automatic energy stopping disabled; a small separate openEMS probe confirmed
that setting executes its full timestep count. This is not a relaxed
acceptance criterion: `check_fixed_duration.py` separately requires the
completed duration to exceed three reported excitation lengths, the final
logged energy decay to reach at least 60 dB, no unused primitive warnings,
and a passing both-port diagnostic. Mesh and physical-model checks remain open.

`evidence.tar.gz` contains the generated geometry, simulation inputs and
audits (SHA-256
`75cfc3dde1e6a91f68c9caf3ec29e0e02a248aae6b17d218220899fdf3077b5b`).
The scripts document the preparation and native selection. The full source
Gerbers are in the neighboring broadband failure report's `fab.tar.gz`.

No accepted EM solve has been obtained on this geometry. Component through-holes,
blind/buried vias, components, antenna, full return path and enclosure remain
outside the model; via plating, actual layer connections and mesh resolution
need validation. The geometry has not yet had the dielectric-loss patch
applied. This is not a tuned-radio, harmonic-emissions or fabrication result.
