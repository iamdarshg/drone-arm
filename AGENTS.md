# AGENTS.md — drone-arm (controller software, ESC, electronics)

Read first: [engineering acceptance issue #21](https://github.com/iamdarshg/drone-arm/issues/21), [cloud bootstrap issue #31](https://github.com/iamdarshg/drone-arm/issues/31), and [drone-mech program #1](https://github.com/iamdarshg/drone-mech/issues/1).

## Role and repo ownership
This repository owns electronic controller/ESC hardware, RP2354B firmware, sensors, bus interfaces, telemetry, software-in-the-loop models, fault handling, flight-control code and executable software/system acceptance tests. `iamdarshg/drone-mech` owns physical airframe, battery/board envelopes, EDF geometry, fluid/structural simulations and manufacturing. Synchronize through pinned versioned JSON contracts; never duplicate production flight-control code into drone-mech.

## First action of an agent on ANY engineering issue
1. Read target issue *in full*, dependencies, this AGENTS.md, `meson.build`, and applicable `docs/hardware/REV_B_ARCHITECTURE.md`; read both repo HEAD SHAs. Inspect actual tracked tree, not only README. Do not treat old generated reports or superseded files as authoritative.
2. Load independent acceptance requirements/check IDs from #21. If harness does not exist yet, **do #21 first**. Existing tests and expected values are the oracle; don't change tests to make your implementation pass.
3. Run fast baseline in the actual environment, preserve exit code, test IDs, artifacts and source SHA. For tools not installed, emit BLOCKED and install pinned supported dependencies when safe.
4. Delegate source inspection, implementation and independent verification to separate agents when available. Implement one issue on a small branch; run tests and negative mutation fixtures after each change.
5. Open a draft PR with exact checks passed/failed/blocked, measured values, reproducible commands, hashes, diff explanation, and remaining risks. A human will authorize manufacturing, high energy tests, deployment and flight.

## Evidence / claims
Acceptable states are PASS, FAIL, BLOCKED, NOT_RUN, and NOT_APPLICABLE (with a reason). Never mark a test passed because a script, report, CAD file, schematic or mock exists. Raw native solver/HIL/bench logs are separate from analytical or simulated results. Test counts must be actual, not aspirational (do NOT fabricate 12,000 checks or inflate them by duplicate parameterizations). Always preserve failing evidence and independently review requirements/threshold changes in a separate PR.

## Real issues from 2026-10-30 repo audit
- `meson.build` refers to `src/` and `include/` trees absent in current tracked main enumeration. Investigate merged PR history and restore genuine source, rather than pretending firmware exists. See #22.
- There are competing Rev-B ESC PCB variants and audit reports: an audited 150×150 mm placement and reports with a 173.95×293.15 mm live board; 1,623 true open connections reported for one live stage. The main board has a final audit with 20 unconnected and RF/USB/CAN route failures. See #23; do not treat old zero ERC as a fabrication release.
- Current Phase-15 mechanical CAD has seven servos (2 elevons+4 EDF exit vector vanes+rudder); main board only maps six PWM outputs (GPIO34–39). Existing hardware issue #20 is open. Do not invent pins/channels. See #24 and drone-mech#5.
- Current motor/ESC architecture is six separately controlled electric motors in **two side-by-side EDF ducts** running 12S; do not call these combustion turbojets. The user wants a future tailless/split drag rudder and thrust-vectoring *experimental option*. Do not remove actual rudder from approved configuration without validated control authority in no/low thrust.
- `tools/PID/drone_model.py` is a **toy quadrotor** plant and its open-loop function uses a placeholder 1/s²; not a valid twin-EDF fixed-wing model. See #25.
- `docs/hardware/REV_B_ARCHITECTURE.md` specifies independent six-cell ESC, CAN-FD supervisory link and six fused battery inputs; a 360 A shared PCB trunk is forbidden. Do not equate battery current and phase RMS current. Do not raise RPM/temperature thresholds from simulated demand.

## Non-weaponized and safety scope
The airframe has no pilot/cockpit, weapons, ammunition or offensive targeting system. Radar is at most an inert future mechanical mass/volume placeholder; a civilian visible camera may be an isolated payload. No combat targeting, electronic warfare or signature-evasion optimization. Safety-related commands must default disabled, have watchdog/readback and controlled fail-safe. No autonomous high-current bench/rotor spin, airframe flight or manufacturing release without qualified human operator and documented evidence.

## Standard work loop
- Before modifying: report objective, exact baseline files + hashes, test list and dependencies.
- While modifying: use typed units, assert physical dimensions and sign conventions, keep conservative defaults, run host tests often; preserve all failure logs.
- After modifying: independent verifier reads actual diff, reruns pinned tests on clean checkout, injects at least one failing fixture, cross-checks contract against drone-mech SHA and records unknowns.
- PR body fields: `Issue`, `Baseline SHA`, `Files changed`, `Independent test IDs`, `Commands/exit codes`, `Artifact hashes`, `Physical blockers`, `Simulation-only capabilities`, `Cloud cost`, `Reviewer findings`. No approval/merge from an AI-generated narrative alone.

## Dependency order (do not leapfrog)
Platform acceptance #21 → firmware source #22 / physical board audit #23 → interface #24 → 6DOF SIL #25 → bounded gain schedule #26 / simulated TVC+split drag #27 → ESC protection #28 / telemetry #29 → supervised HIL #30. Cloud automation #31 supports those phases. Create separate PRs for source changes vs changes to test expectations.
