# Work in progress — 2026-11-04

All work in this directory is diagnostic; none of the results establishes PCB fabrication readiness or aircraft flight readiness. Containers used are individually named codex_* and no other container was stopped. No WSL distribution was modified.

## Inputs
- Main Rev-B PCB exact Git blob: main_rev_b_source.kicad_pcb, SHA-256 b500f5fecd2299cdca0af12a05424e820ed845e1bb5ab35568e35cc38e826d43.
- Phase-15 full assembly STEP: phase15_airframe_full_assembly.step, SHA-256 9b83f3bfa5ea261c2055152c9fdd2147b9e16819634715c3c4ae23b45deae772.
- Solver images include local-gerber2ems (sha256 bb92165073b517d125a2872947fb3ec451fc7bbfcde02e1a66e5e30f678952de) and opencfd/openfoam-default:2406.

## RF
- rf_ground_audit.json samples 11 points of the straight /915 MHz RF/ANT_915 segment and records /3V3 on In1 beneath each, and /GND on B.Cu; point containment is not proof of RF return-current quality. No vias were found inside the specified rectangular ROI.
- Two native openEMS idealized straight-line runs reached their energy stop criterion. They assume an uninterrupted PEC reference plane, omit the antenna, component models, actual return geometry and copper loss. Their S11 values differ substantially on refinement; they are not mesh-converged and are not RF sign-off.
- Gerber-derived rf_gerber_roi_v2 and rf_gerber_roi_v3_refined used a 3V3 reference plane and lossless dielectric; both fail strict passive power balance at 915 MHz (see rf_gerber_mesh_comparison.json). The net-selected rf_gerber_roi_v4_x2 has a patched constant-conductivity approximation for the specified FR4 loss tangent at 915 MHz, and its simulation is in progress. Port definitions, ROI truncation, components, antenna and coupling remain unresolved.

## Mechanical and CFD
- Gmsh imported 516 solids from the pinned full assembly. Default OCC bounds were inflated by B-spline control geometry; triangulated bounds span approximately 520 x 148 x 705.5 mm including the aft electronics. This is an import check, not a CFD mesh.
- OpenFOAM 2406 checkMesh on the tracked synthetic outboard wing found 73 small-determinant cells. An HXT remesh found 302 such cells and 7 low-weight faces; relocation found 263 and 3 respectively. Each fails the strict gate; no CFD performance result has been accepted. Logs are in cfd_wing_t30_diagnostic, cfd_hxt_check and cfd_hxt_relocate_check.
- CalculiX 2.21 completed ten modes and two static diagnostics on the repository's synthetic wing sector, fixed at the root, with homogeneous E=10 GPa, nu=0.3 and density=150 kg/m3. These are assumed parameters, not measured laminate properties or an assembly model. See modal_wing_diagnostic, static_wing_unitload, and static_wing_900ias_1q_screen.
- The 900 IAS 1q sensitivity applies uniform upward suction on the selected synthetic upper wing faces (not a CFD-derived load); root reactions balance the load on unconstrained nodes in equilibrium.json. There is no failure criterion, laminate definition, full-aircraft coupling or structural safety conclusion.
