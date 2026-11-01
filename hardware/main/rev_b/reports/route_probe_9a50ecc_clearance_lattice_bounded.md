# Bounded GND clearance-lattice scan outcome

Source commit: `9a50ecce4a4634d0231f7660737de8a5c14b3617`; source PCB SHA-256 `095aa3fd44285b885c2a4ca68c05633b169dd053fbc8407d2160e8585e73336b`; matching project SHA-256 `f5bfbb6150d2799b8cc0ddb899c0e18c86ef452cbf68e05c60f883a969068a9e`.

Two inline Python searches sampled F.Cu GND fill outlines at 0.1 mm and 0.2 mm to find through-via sites with courtyard, pad, and zone-edge clearance. They had no progress files and their output pipes were no longer retrievable. I stopped both at the 30-minute wall limit (PIDs 14871 and 14937; elapsed 30:38 and 30:12). No new completed coordinates were recovered from those active runs.

The earlier completed 0.2 mm exploration identified `(107.0,72.4)` in F.Cu GND island 0 as the best open-area point (over 1 mm clearance to nearby pads/courtyards and zone edge). This point was already checked on the exact same PCB hash with explicit paired `pcbnew.ZONE_FILLER.Fill(board.Zones())` and matching-project KiCad 9.0.2 DRC. The full paired output and candidate hash are pinned in [`route_probe_3f2de7e_gnd_c110_rejected.md`](route_probe_3f2de7e_gnd_c110_rejected.md): source and candidate both reported 7 violations / 42 unconnected, with unchanged signatures and unchanged GND plane airwire. The via was redundant because an existing GND through-via is 1.6 mm away, so no candidate was promoted.

The scan outcome is bounded: no additional coordinates from the terminated processes are used as evidence, and no route or via is promoted from geometry alone. The board remains unchanged.
