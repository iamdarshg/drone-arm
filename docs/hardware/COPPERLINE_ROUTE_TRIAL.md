# Copperline routing trial — Rev-B main board

**Disposition: rejected; do not use this output for fabrication.** This note records a headless routing experiment against the Rev-B main-board PCB. It is not a release report and does not certify or replace the KiCad board.

## Reproduction context

| Field | Value |
| --- | --- |
| Repository | `iamdarshg/drone-arm` |
| Input file | `hardware/main/rev_b/main_rev_b_final.kicad_pcb` |
| Input revision | `744994a` (`main` at trial time) |
| Router | Copperline `0.1.0`, commit `e2a0beb` |
| Seed / workers | `42` / `4` |
| Search time limit | `120 s` |
| Output | Temporary file only; not copied over the source PCB |

The Copperline binary was built from its checked-in C++ sources using GCC 13 and C++20 because CMake and KiCad CLI were unavailable in the execution environment. The source build completed. Copperline's KiCad importer emitted warnings for non-Manhattan copper and its documented thru-hole pad fanout approximation.

## Results

| Check | Untouched input through Copperline verifier | Routed candidate through Copperline verifier |
| --- | ---: | ---: |
| Unconnected terminals | 388 | 241 |
| Clearance/rule violations | 745 | 838 |
| Copperline result | `ok: false` | `status: VIOLATION`, `verifier_ok: false` |

The route search connected additional terminals, but the exported candidate added 93 verifier violations and still had 241 unconnected terminals. These counts are Copperline's model of the file, not a KiCad DRC result; they also show that Copperline's current direct-KiCad round trip cannot be accepted for this board. The untouched source PCB remains unchanged.

## Next safe attempt

1. Use the exact KiCad 9.0.7 CLI to export the board to a routing interchange format with its net classes, stackup, planes, and constraints preserved.
2. Add a version-controlled Copperline sidecar for actual net currents, voltages, widths, clearances, and via rules. Do not use its fallback `0.5 A` board default for the motor power, battery, or shunt paths.
3. Route a copy/checkpoint, export the session, and import it back into the same KiCad project. Keep all existing high-current, RF, Kelvin-sense, return-path, creepage, and impedance constraints intact.
4. Run fresh KiCad ERC/DRC on the re-imported candidate and compare against the unmodified board. Reject any new violation, unconnected net, changed schematic contract, or unexpected plane/stackup change.
5. Only promote a verified checkpoint after engineering review. Continue to route and verify the ESC separately; this trial says nothing about ESC routing.

## What this trial does not establish

- It is not an accepted route and does not close the Rev-B fabrication issue.
- It is not a KiCad DRC run; exact KiCad 9.0.7 was not installed here.
- It does not qualify ampacity, impedance, RF behavior, thermal performance, or manufacturability.
- It does not establish the current state of the ESC board.
