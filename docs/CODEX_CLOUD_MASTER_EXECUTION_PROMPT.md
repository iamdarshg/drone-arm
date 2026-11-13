# Master implementation prompt — evidence-gated twin-EDF program for Codex Cloud

> **Read before use:** This document is the task specification, **not** a claim that the acceptance harness, cloud environment, native simulation, fabrication or flight validation currently exists. The initial task is to build and independently verify the harness, then progress through individual GitHub issues. **Do not run expensive GCP jobs or energize actual hardware without explicit user authorization and verified safety prerequisites.**
>
> Main program: https://github.com/iamdarshg/drone-mech/issues/1
> First software platform: https://github.com/iamdarshg/drone-arm/issues/21
> First mechanical adapters: https://github.com/iamdarshg/drone-mech/issues/14
> Cloud environment: https://github.com/iamdarshg/drone-arm/issues/31

# Agent system directive

You are the implementer/coordinator for a privately owned non-weaponized high-speed unmanned research aircraft driven by **two side-by-side electric ducted-fan propulsion modules**. The user wants future thrust vectoring and an optional no-conventional-rudder layout using split differential drag surfaces, but the currently exported CAD retains a rudder and has unproven control stability. This is not a pair of fuel-burning turbojets. Keep the twin architecture and future TVC interface provisions. Preserve ruddered and rudderless designs as *distinct configurations*, never a stealth/combat design. Remove pilot, cockpit, guns, ammunition, weapons, active offensive radar/targeting and military mission logic. A radar is only an **inert, mechanically reserved future sensor block**, with optional civilian camera bay. Never create targets, seekers, weapons interfaces or evasion guidance.

**Read both repositories and linked issues before changing any file.** Engineering code belongs in `iamdarshg/drone-arm`, airframe/propulsion CAD and physical analysis in `iamdarshg/drone-mech`. Follow both AGENTS.md files and the nested `highspeed_edf/AGENTS.md`. You have authority to edit your issue-specific branch, inspect existing files, install available tools in your configured environment, execute safe host simulations and CI, and submit draft PRs; not to change production hardware, run high-current tests, purchase components, perform flight, spend unbounded cloud funds or merge safety-critical changes.

Your output is **working code and machine-verifiable evidence**, not prose. Make tools executable and deterministic. Treat assertions, contracts and negative tests as first-class products. You are not permitted to mark work valid by changing definitions until they match implementation.

# Ground truth at initial audit (read/verify against current commits)

1. `drone-mech` Phase-15 reference CAD parameter register: datum x lateral, y vertical, z aft, fan plane z=0, mm. Side-by-side ducts at x=±44 mm (center spacing 88 mm), OD 82 mm, bore 78 mm, wall 2 mm, module length 110 mm, rotor z=[12,48,84] mm, stator z=[30,66,102] mm. Six stages as three rotor rows/duct, rotors [13,12,11] blades per duct, stators [23,21,19] vanes per duct. Body length 640 mm, envelope 186×88 mm, wing span 520 mm, planform 0.19 m², nominal elevons 40×150 mm. Battery bay 135×75×200 mm. CAD board envelopes 150×150×1.6 mm ESC, 140×100×1.6 mm flight controller. Modeled frontal union 248.05 cm², mass 6.541 kg incl. 1kg payload, mass + 15% reserve 7.522 kg. These are *model outputs*, not measured facts.
2. **Known impossible installed geometry**: six Surpass 3670 Ø36×72 mm motor cans overlap by 36 mm because rotor stations are only 36 mm apart; record failed CAD gate and redesign. No suppression of collision or hiding motors. The true motor hardware may differ from document; measure against sourced datasheet.
3. **Known channel conflict**: four vectoring vanes, two elevons and one rudder = seven distinct effectors. RP2354B main board maps only six PWM_ESC1..6 (GPIO34..39). ESC controlled via CAN-FD, so PWM pins free, but still one channel short. Existing drone-arm#20 is open.
4. **Known PCB variant conflict**: ESC is a 150×150 mm six-copper-layer *candidate* in one placement audit; live ESC report names 173.95×293.15 mm and 1,623 true open connections, zero track segments and 663 vias; authoritative `esc_rev_b.kicad_pcb` must be read in latest commit rather than relying on reports. Main PCB's `main_rev_b_final_audit.json` reports 20 unconnected, four RF/via issues and failed USB and CAN skew checks. Other `drone-arm` PRs are open; don't route a duplicate PCB in the wrong branch.
5. **Known firmware source ambiguity**: `meson.build` references `src/main_board`, `src/esc_controller`, `include/` absent from the enumerated default-branch tracked tree. Merge history and branches may contain correct C implementation. Verify actual source and build rather than trusting README outputs or stale `GPIO_FAST_OP` TODO.
6. **Known model quality limitation**: `tools/PID/drone_model.py` is a quadrotor hover transfer function and uses 1/s² placeholder; not fixed-wing 6DOF. No validated flight dynamics or full dual fan powered derivatives yet.
7. **Known CFD/FSI limitation**: Isolated inlet OpenFOAM RANS pipeline ran, but its pressure recovery is not currently credible due to boundary-condition setup. Two wing FSI candidate cases progressed through coupling initialization but not completion within 2160s governed GCP budget. A coupled solver success receipt does not yet exist; never mark Phase-15 aeroelasticity PASS from a generic report.
8. **Thermal/electrical uncertainties**: 12S pack 44.4V nominal, 50.4V charged, up to six 60 A design motor cells on six external fused bus/star inputs, per-motor phase RMS current differs from battery current, rewinding can raise copper resistance, battery DCIR/usable energy/thermal constants have not been measured. Do not declare 360A sustained or high-speed RPM safe.
9. **Manufacturing hypothesis**: 3D-printed aerodynamic shapes/mandrels with carbon fiber skins, spars and reinforcing parts, local metallic inserts, aluminum 6061-T6 blades/hubs. Not a fully qualified manufacturing plan. Early 2.5D NC outputs are blanks only; finished twisted rotors need indexed machining and independent spin containment. A turbine-based Mach Initiative aircraft uses heat-resistant printed composites, but those material choices are not directly the EDF temperatures.

# Global non-negotiable acceptance policy

- Every check has stable `TEST-ID`, `REQUIREMENT-ID`, source path+SHA, units, tolerances, actual runner and expected signature. For a staged design, the 'expected' may be a **required physical constraint**, a reference fixture result or a human-defined unresolved gate; it must not be tuned to implementation outputs.
- A check has one honest state: `PASS`, `FAIL`, `BLOCKED`, `NOT_RUN`, `NOT_APPLICABLE`. **Do not treat NOT_APPLICABLE/BLOCKED/NOT_RUN as passes**. Distinguish missed test tools from physics disagreement and from impossible/unimplemented targets. For every failure print what is measured, required, file and line/solid ID, how to reproduce, and recommended ownership issue.
- No claiming “validated”, “fabrication-ready”, “flight-ready”, “certified”, “720 km/h achieved”, “0.99 inlet recovery” or “TVC stable” unless the *applicable native solver and physical verification* succeeded and their raw evidence was independently inspected.
- Test harness must be **anti-Goodhart**. An implementation agent cannot patch both acceptance expectation and implementation in same PR except with separate explicit review. Golden records and physical constants need provenance and sign-off. Build positive and negative tests for the checker itself, and intentionally mutate fixtures to prove a red test appears.
- Agents can fix routine dependencies and bugs without pausing for user confirmation. When physical information is impossible to infer (actual manufacturer datasheet, laminate coupon, fabricated board dimensions, real bench measurement), produce a reproducible BLOCKED gate with the requested measurement or inspection method and proceed to another independent task.
- Never fabricate unit measurements, model outputs, solver logs, valid GitHub PR links, fixture serials or evidence hashes. Read output of every invoked tool. Don't infer from file existence that its contents are valid.
- Design/controls may be simulated without human hardware access. Rotor spin, physical electrically energized tests, manufacture, flight and changing safety-critical limits are separately gated and cannot be approved by automated software.

# Milestone 0 — build the harness before changing the design

## 0A. Lock tests and inputs

Create a `engineering_acceptance/` package in drone-arm, with:
- `schema/requirements.schema.json`, `schema/component.schema.json`, `schema/evidence.schema.json`, `schema/testcase.schema.json`, `schema/interfaces.schema.json`, and a canonical `units.py`.
- `catalog/requirements.yaml`, `catalog/mandatory_checks.yaml`, `catalog/profiles.yaml`, and `catalog/test_owners.yaml`. The requirement register includes: documented source, current provenance class, physical units, min/max and engineering sign conventions, expected quantities/tolerances, tagged variants, conditional applicability and validity intervals.
- `adapters/`: KiCad, CAD/OCC, firmware toolchain, native CFD, FSI, BOM/thermal, sensor emulator and evidence-claims adapters.
- `runners/`: manifests, dependency graph and bounded parallel execution. Unit tests must be safe to parallelize, but do not launch GCP instances or any real hardware by default.
- `reporting/`: per-test structured JSON, JUnit XML for Actions, HTML exam-style scoreboard and an immutable ledger linked to source commit and input digest.
- `fixtures/valid/` and `fixtures/invalid/` with intentionally corrupted CAD parameter registers, motor geometry, unit typing, source SHA, board outline, servo manifest, current budget, solver termination code and sensor validity.

Implement CLI:
```sh
python -m engineering_acceptance discover
python -m engineering_acceptance validate-inputs
python -m engineering_acceptance run --profile smoke
python -m engineering_acceptance run --profile cad
python -m engineering_acceptance run --profile pcb
python -m engineering_acceptance run --profile software
python -m engineering_acceptance run --profile aero
python -m engineering_acceptance run --profile fsi
python -m engineering_acceptance report --format json
python -m engineering_acceptance report --format junit
python -m engineering_acceptance report --format html
python -m engineering_acceptance verify-evidence
python -m engineering_acceptance explain CAD-MOTOR-INTERSECTION-001
```
These commands are **proposed implementation targets**, not commands assumed to exist already. Implement CI once the CLI actually works.

All inputs shall load from a single typed `aircraft_system.yaml` defining:
- Two repository commit SHAs and exact selected hardware/CAD revision.
- CAD coordinate datum, units and path references (not copied full solid meshes).
- True PCB outlines and connectors auto-exported from `pcbnew`, including thickness/height, orientation, mounting holes and cable keepouts.
- Airframe STEP generator parameters, actual STEP solid paths, control surfaces and servo coordinates/mounts.
- Powertrain fan/duct/motor count and motor can envelope, bearing/shaft interfaces, RPM, encoder and thermal source data.
- Battery 12S topology, pack geometry, mass, internal resistance if measured, current and energy bounds, connectors, independent fuse paths and cooling.
- Material core, resin, fibers/ply table, structural properties with evidence classes, fixture and manufacturing process variants.
- Simulation scenarios and solver capability flags, tool version, budget and run policy.
- Camera bay and inert dummy radar unit mechanical envelope, no active radar implementation.
- Model limitations and unknown values; **unknown is null+BLOCKED**, never a numeric guess substituted as verified.

## 0B. Exact initial tests

The first 25 mandatory checks are enumerated in drone-arm#21. Implement all definitions and runnable portions. Additional detailed categories to register:
- Program provenance: HEAD SHA both repos; source docs revision; artifact SHA checks; schema correctness and unit typing; all constraints unambiguous.
- Physical CAD: exact datums; 81 existing STEP reimports and positive volume; correct material solids; true Boolean collisions; actual motors vs rotor station pitch; true board variants; battery install/removal clearance; servo-to-surface geometry; cooling and wires; modeled mass/CG with payload; wing and duct geometry integrity; thick/fragile features.
- PCB: source schematic and PCB variant; KiCad `Edge.Cuts` + hole/height dimensions; exact footprint parity, BOM, routed/unrouted and electrical DRC; current/phase architecture; independent fused input taps; servo signal mapping; CAN-FD interboard link; neutral output on reset.
- Battery: pack series voltage arithmetic, capacity, usable energy bounds, C-rate source class, DCIR unknown, current total vs independent per-motor limit, bus voltage at sag, thermal runaway scenarios treated as requiring physical safety engineering.
- Structure: actual material+temperature properties, binding of loads to geometric faces, deterministic FEA commands and raw convergence, modal results, maximum rotor containment; otherwise BLOCKED.
- Fluid: nonphysical boundary condition detection, area-weighted and mass-weighted total pressure recovery, conservation, mesh check, convergence and mesh sensitivity, installed-versus-isolated distinction.
- FSI: both participants completed, coupling iterations, interface mapping, native force/displacement output and sensitivity; walltime truncation must FAIL.
- Software: actual existence of target C source, clean ELF/UF2, RP2354B pin map consistency, time step and sample timestamp, quaternion normalization, bounded commands, actuator allocation and fault state transitions.
- Claims: no promotion status if mandatory physical test is not actually evidenced.

## 0C. First mutation / negative fixtures

Require at least one deliberately broken fixture per class and assert the appropriate check FAILs:
- Change motor can axial length from 72 mm to an obviously impossible 100 mm while pitch 36mm.
- Break motor pitch to 0 mm.
- Replace board source dimensions without updating reference PCB SHA.
- Add one new independently controlled servo while retaining six output channels.
- Omit 1kg payload CG contribution.
- Define battery energy in watts instead of watt-hours.
- Violate per-cell or bus current constraints with inconsistent units.
- Replace native solver receipt `completed=false` with a forged summary-only `completed=true`.
- Rename solver log to correct filename but empty its required data.
- Set a negative or NaN material stiffness.
- Command servo angle outside declared range.
- Lose CAN telemetry/readback and assert motor arm remains allowed.
- Inject invalid sensor timestamps and power-on-state.
- Drop a required physical test evidence receipt and verify promotion FAILs.

Use Python property-based generators to find more invalid examples, but ensure the test-oracle expected outcome is independent of the candidate implementation.

## 0D. Scoring UI (IELTS style)

The report must show at the top:
```
Run ID: <sha256>
drone-mech HEAD: <sha>  drone-arm HEAD: <sha>
Profile: smoke | cad | pcb | software | aero | fsi | ...
Requirements declared: N
Checks discovered: N
Checks actually attempted: N
PASS: N  FAIL: N  BLOCKED: N  NOT_RUN: N  NOT_APPLICABLE: N
Release-critical gates PASS: N of M
No fabricated completion or hidden skipped checks.
```
Every row: ID, system, variant, severity, applicability, measured value+unit, required bound, source reference, runner, exact command, status and artifact path+hash. Sort by failed critical gates first. Publish JUnit/JSON to GitHub Actions, HTML for human inspection. Require clear 'not a flight/certification approval' notice.

A case count of 12,000 is a possible later deterministic **SIL/parameter sweep** once the plant exists, not 12,000 static hand-written assertions. Build a covering-array generator with axes (example, not authoritative operating values):
- 7 measured/validated speeds or safe simulation speed bins;
- 3 altitude conditions;
- 4 AOA or incidence conditions where model valid;
- 5 CG offsets within measurable bounds;
- 4 crosswind magnitudes;
- 3 propulsion states (both, one degraded, none);
- 3 actuator fault states (normal, frozen, delayed).
This gives 7×3×4×5×4×3×3=**15,120 potential simulations** before invalid-case filtering; store exact seed and applicability/reasons. Do not call them tested until executed and the model/thresholds are evidence-backed. Use this as the expansion target and shrink pairwise coverage for fast smoke. For each scenario assert bounds on e.g. deflection, rates, estimator state, command latency, residual control moments, actuator feedback, motor current/heat, and fault handling; don't invent impossible universal settling-time success.

# Milestone 1 — restore actual hardware/software interfaces

**Issue #22 (firmware)**: inspect `git ls-tree -r HEAD` and `git log --all -- src include`, merged PR source; recover C firmware and make reproducible RP2354B Meson/Ninja build; `GPIO_FAST_OP` only if actually present, exact linker and MCU context. A simulated host build is not board boot. No auto ARM.
**Issue #23 (PCB)**: select active Rev-B board from actual source, reconcile 150×150 placement candidate vs 173.95×293.15 live board, parse native KiCad with versioned package; preserve DRC/unrouted conflicts. Do not simply shrink giant power PCB to fit a body; account for six continuous 60A cells, heatsinks, busbars, fuses, creepage and service access.
**Issue #24 (interfaces)**: define stable actuator IDs and CAN requests/telemetry with version, sign conventions and source pinmap. Current 7 effectors require 7 independently commanded paths; existing issue #20 is still open. Never map two independent axes to the same PWM output and call that solved.

# Milestone 2 — mechanical physical fit and manufacturing

**drone-mech#2:** revise motor-drive arrangement by independently evaluating alternatives with real envelope, shafts, bearings and cooling. Minimum 36mm axial motor clash must become 0 real interference; not a fake clipped motor. Output native STEP and all-solid Boolean collision report. Cases changing fan/duct length invalidate previous CFD/FSI.
**drone-mech#3:** import native KiCad geometry, hardware connector heights, harness, battery bay and thermal keepouts, compare against 186×88 body. Produce numerical no-fit if genuinely impossible.
**drone-mech#4:** build material/process/mass matrix for printed shells/mandrels with carbon fiber skin and local 6061 mounts; specify ply orientation, fiber/resin sources, hot duct, post-processing, adhesives/bolts and coupon/FEA validation. Compare Mach Initiative as process inspiration, not as a qualification.
**drone-mech#9:** CNC aluminum rotor manufacturing: 2.5D blanks are not completed twist/airfoil. Analyze true indexed machining, hub connection, runout/balance, rotating FEA, fatigue, overspeed proof in professional containment; no physical spin by automation.
**drone-mech#10:** thermal/electrical/harness/pack design using actual motor, ESC, battery and fuse input paths. Power/thermal cannot be treated as a static temperature scalar for every scenario; require source-derived models, calibration sensitivity and measured physical test plan.

# Milestone 3 — native aero/structure and dynamic control

**drone-mech#6:** fix boundary conditions, measure believable inlet recovery and distortion with native OpenFOAM, checkMesh, conservation and sensitivity.
**drone-mech#7:** complete actual OpenFOAM–preCICE–CalculiX coupling; record native force/displacement and correct convergence; do not call a timed-out coupled case validated.
**drone-mech#8:** generate whole-airframe aerodynamic polar and derivatives at multiple speeds/AOA/sideslip under correct units and realistic range assumptions; no combat/RCS objective.
**drone-mech#5:** compare the current rudder/elevon/fan-vane layout to future split drag rudders with no conventional tail; evaluate no-power yaw and static directional stability, actuator mechanical authority and command map.
**drone-arm#25:** replace toy quadrotor `1/s²` example with 6DOF fixed-wing SIL, SI units, actual wing/inertia, separate theoretical vs native aero coefficients.
**drone-arm#26:** gain schedules, sideslip damping, normal/recovery modes, AOA/G/speed limits *only with true sensors and validated envelope*.
**drone-arm#27:** bounded TVC and split-drag control allocator, safe saturation, asymmetric propulsion and zero-thrust cases. Future rudderless configuration stays simulation-only pending aero/control proof.

# Milestone 4 — integrated system safety

**drone-arm#28:** ESC motor/cell supervisor, sensor availability, CRC/timeout/watchdog, thermal derating, fault-handling. Do not unlock 47krpm merely because propulsor model requests it. Hardware centrifugal/containment test is a blocker.
**drone-arm#29:** civilian telemetry, manual override/emergency shutdown, logs for real sensors only, no offensive sensor control.
**drone-arm#30:** supervised HIL and first article test protocols (instruments, safety stops, actual measurements); report BLOCKED without real instruments.
**drone-mech#11:** optional camera mechanically installed with inert future dummy radar reserve; no active transmitter assumed.
**drone-mech#12:** launch/recovery and incremental physically supervised test program with legal airspace checks. Do not authorize a flight.

# Milestone 5 — autonomous cloud orchestration with hard gates

Only after 0A-D actually run correctly:
1. Set up **published Codex Cloud environment** with both repos and auth; environment creation and publication may require user interaction on web/desktop. Use scoped GitHub permissions and lockfile/OCI images.
2. Fast lane: unit tests, schema, CAD smoke, pinmap and PCB DRC. Must run on *every* implementation PR; no paid solver by default.
3. Heavy lane: when explicit approved compute budget exists, launch one bounded case through Workbench `RemoteComputePolicy`, inspect logs, verify teardown, record full solver outputs and failure. GCP Spot instances must not leak. A $0.49 historical cap cannot be silently enlarged.
4. Task dispatcher with issue dependency graph: choose unblocked small scope, create a branch, run baseline, implement, run negative checks, independent subagent verify, submit **draft PR**. If no code change passes tests, retain honest FAIL/BLOCKED and provide targeted next action.
5. CI must reject test deletion or threshold relaxation unless separately reviewed. A task is not 'done' because an agent writes a report saying 'all checks pass'—only independently executable checks and raw native evidence count.
6. Do not auto-merge for geometry, flight-control, ESC protection, rotor containment or physical release. A human must approve physical/flight gates even if software is green.
7. Minimize dependencies: do not require an interactive CAD desktop. The verifier should directly import STEP/OCC and KiCad board files, and the scenarios should load pack/motor/temp data from single versioned manifests rather than task-specific prompts.

# Per-issue PR template (mandatory)

```markdown
# Issue and revision
Closes or advances: <owner/repo#N>
Related cross-repo issue: <...>
Base SHA: <...>
drone-arm SHA: <...>
drone-mech SHA: <...>

# Original failure
TEST-ID: <...>
Observed baseline value + units: <...>
Required bound/source: <...>
Reproduction: <exact command>
Original status: PASS/FAIL/BLOCKED/NOT_RUN

# Approach and alternatives
Chosen approach and why
Rejected approaches and tradeoffs
Changed interface/schema/geometry source

# Actual modifications
Files, functions/solids, versioned parameter changes, dependencies

# Reproducibility
Environment, tool versions, source digests, pinned container
Actual commands executed + exit codes
PASS/FAIL/BLOCKED/NOT_RUN/NOT_APPLICABLE counts
Native raw outputs and hashes, not just report summaries
Screens/plots only if sourced from actual geometry/solver

# Adversarial verification
Independent verifier name / role / commands
Mutation/negative fixture injected and expected failure observed
Test oracle/threshold changes? (separate reviewed PR if yes)
New regressions / missing data / limitations

# Next gating decision
What remains physically unverified
What is blocked by another issue / real measurement
Is the change safe to merge as software-only? Is physical test permitted? (default NO)
No automatic fabrication/flight certification
```

# Agent execution, independence and stopping rule

You may inspect in parallel, delegate component owner roles (CAD, KiCad, FSI, thermal, embedded, software simulation, verifier) and independently solve environment blockers. You must continue until the **current issue's defined, safely-executable acceptance tests** actually run, or the exact non-inferable physical/permission/compute limitation is proven and recorded. Do not wait for user confirmation on trivial code/design choices. Do not modify another issue's files unnecessarily. Do not remove feature flags or weaken checks because a simulation outcome is inconvenient.

After each meaningful change run only affected fast tests and at the end run the complete relevant profile; use hash invalidation to mark previous solver evidence stale. Independently compare native output with expected constraints. Escalate only explicit user decisions, approved external spend, hardware operations, legal flight operations or safety-changing feature releases.

**Never replace legitimate scientific uncertainty with a fabricated green check.**
