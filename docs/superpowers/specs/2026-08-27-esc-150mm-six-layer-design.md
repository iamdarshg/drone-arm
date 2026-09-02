# 150 mm Six-Layer ESC Physical Design

## Scope

Replace only `hardware/esc/rev_b/esc_rev_b.kicad_pcb` with a deterministic placement candidate generated from the existing validated Rev-B ESC netlist. Preserve the schematic, all electrical footprints, reference designators, pad-to-net assignments, and the separate flight-control PCB.

## Mechanical architecture

The PCB outline is a closed 150.00 mm by 150.00 mm square. Six motor cells occupy a 3-by-2 matrix. Each cell owns a 48 mm by 55 mm placement grid; motor cells 1-3 occupy the upper row and 4-6 the lower row. Shared auxiliary power and supervisor components occupy dedicated horizontal grids outside the motor grids. Mounting holes remain inside a 4 mm edge inset and must not overlap electrical courtyards.

Every electrical footprint must be fully inside the outline. High-current and gate-loop parts remain on F.Cu; low-power local control and support parts may be placed on B.Cu within the same owning motor grid. Grid membership is derived from the component sheet and the existing motor-numbered reference blocks. The generator must fail instead of silently placing a footprint outside its assigned grid.

## Six-layer stack

The copper layers are F.Cu, In1.Cu, In2.Cu, In3.Cu, In4.Cu, and B.Cu in a nominal 1.6 mm stack. F.Cu hosts all components, local switching copper, gate loops, and Kelvin pickups. In1.Cu is the quiet/local return reference. In2.Cu carries reinforced BAT+ distribution. In3.Cu carries reinforced BAT- distribution. In4.Cu is reserved for low-current control and telemetry routing. B.Cu provides local thermal spreading and reinforced power return. Final dielectric and copper thicknesses require fabricator field solving and thermal review.

Switch-node copper stays local to each motor cell. No global phase plane is permitted. BAT+ and BAT- reinforcement may be shared only through intentional star/bus regions and must retain isolation from quiet references.

## Placement strategy

Each motor grid places its three half bridges, gate driver, gate resistors, current shunts, and local DC-link components on F.Cu. Motor terminals, isolated control, local MCU, current-sense support, and remaining passives use a deterministic B.Cu grid where necessary. Remaining components are packed deterministically using courtyard-aware rectangle packing. Support parts cannot cross a cell boundary.

The top shared grid contains battery input/protection and auxiliary supplies. The center/shared corridor contains supervisor, ADC, CAN-FD, and isolated-control components. Connectors that require physical access are placed on board edges.

## Generated-board gates

The generation stage is successful only when all of these are true:

- outline bounding box is exactly 150.00 mm by 150.00 mm;
- copper-layer count is exactly six;
- all schematic-derived electrical footprints are present exactly once;
- every footprint is fully inside the board and assigned to an intentional side;
- each motor-numbered footprint anchor is inside its assigned grid; courtyard boundary bleed is reported separately and must be resolved before fabrication routing;
- no courtyard overlap exists between different motor grids;
- KiCad accepts the PCB and reports no malformed outline;
- a placement audit records grid bounds, populations, and failures.

Routing, zero-open DRC, continuous 60 A operation, aggregate 360 A operation, busbar design, heatsinking, double-pulse behavior, and thermal qualification are downstream gates. Passing placement does not establish those ratings.
