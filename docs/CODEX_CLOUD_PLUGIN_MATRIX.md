# Codex Cloud infrastructure: capabilities, plugins, tools and setup

This is a **proposed setup**, not a claim that the Codex Cloud environment, paid credentials, CI or external services are already configured. Work item: https://github.com/iamdarshg/drone-arm/issues/31 ; first executable tests: https://github.com/iamdarshg/drone-arm/issues/21 ; mechanical adapters: https://github.com/iamdarshg/drone-mech/issues/14 .

## Platform distinctions
The GitHub connector grants repo read/write in ChatGPT, but does **not** imply that a separate Codex Cloud environment is published with both repositories, or that KiCad, OCC, OpenFOAM and GCP are installed. A ChatGPT plugin/MCP is not a native CAD solver. A Codex skill is procedure + scripts; it cannot supply physical measurements. Prefer deterministic CLIs over GUI/computer-use.

## Required capabilities for a reusable Codex environment

| Role | Actual package/plugin | Need | Cloud setup & verifiable probe |
|---|---|---|---|
| Repository operations | Connected GitHub plugin; `git`, `gh` CLI or GitHub MCP | Required | Grant scoped access to `iamdarshg/drone-arm` and private `iamdarshg/drone-mech`; run `git ls-remote` on both, `gh repo view`; never print tokens |
| Agent policy | Root `AGENTS.md` + repo-local `.agents/skills` + issue acceptance instructions | Required | Codex must read and report issue IDs; no tool installation needed |
| Cloud environment | Codex Cloud published environment (via Codex web/desktop) | Required for managed cloud tasks | Set up GitHub repos/dependencies/network controls and verify clean checkout; mobile tasks reuse published environment |
| CI orchestration | GitHub Actions with pinned actions + `gh` | Required | Run acceptance smoke on PR and upload JUnit/JSON; branch protection/required checks may need manual settings |
| Workflow dispatch | GitHub Actions issue/PR workflow + Codex CLI/SDK if available | Optional later | Only start after #21, return **draft PR**, never auto-merge safety-critical work; use scoped bot permissions |
| Python test engine | Python 3, `pytest`, `hypothesis`, `pydantic`, `jsonschema`, `pint` and `numpy` | Required | Pin lockfile/uv; `pytest -q`, schema and intentional-invalid fixture tests |
| Scenarios/coverage | `hypothesis`, `pytest-xdist`, `allpairspy` or custom covering arrays | Recommended | Generate unique seeded cases and explain coverage/denominators; do not fabricate 12k pass scores |
| Embedded build | `arm-none-eabi-gcc`, Meson, Ninja, binutils, target headers, UF2 tools | Required for drone-arm | `meson setup`, `ninja -C build`, `nm/readelf/objdump` with artifacts; missing src means FAIL/BLOCKED |
| KiCad PCB | KiCad 9 CLI + `pcbnew` compatible Python + `kicad-cli` | Required for PCB adapter | Parse `Edge.Cuts`, footprint models, holes, DRC and connectivity on actual latest variant; `kicad-cli version` |
| CAD solids | CadQuery + OpenCascade (OCP); FreeCAD headless if source requires it | Required for native CAD import/generator | STEP/BREP round-trip and Boolean intersection checks; no GUI; pin shapes/version; sample check run |
| Meshing | Gmsh / meshio | Recommended; required for CFD | Mesh quality and negative geometry tests; `gmsh --version` |
| Aerodynamics | OpenFOAM v2606 or precisely supported alternative | Heavy opt-in | `foamVersion`, `checkMesh`, solver residuals; do not infer fluid result from a generated case |
| FSI | preCICE, OpenFOAM adapter, CalculiX + adapter, matching versions | Heavy opt-in | preCICE participants must finish real coupled windows, not just exchange toy fields |
| Structures/thermal | CalculiX / Code_Aster (choose one validated path) and parameterized property tables | Recommended | Native solve, loads/materials/BC, eigenmodes and heat; measured thermal properties still a physical gate |
| Aero model | Aero Multiphysics Workbench repo/use existing versioned SDK | Existing external dependency | Pin commit, evaluate-only screening labeled analytical; do not substitute for CFD |
| Cloud compute | Google Cloud SDK + GCP IAM (prefer workload identity) + Workbench RemoteComputePolicy | Heavy opt-in only | Dry-run max spend; no remotely booted VM by default; check deletion; store receipts/cost |
| Artifact store | GitHub Actions artifacts, GCS with restricted service account / immutable hash | Recommended | Keep multi-GB STEP/FSI off PR diffs; SHA256 and retention; no credentials in artifacts |
| Quality/static security | ruff, mypy/pyright, clang-format, clang-tidy, cppcheck, Semgrep as appropriate | Recommended | Pin tool versions, fail on relevant C/Python regressions, do not use as physical design approval |
| Visualization | matplotlib, VTK / PyVista headless | Recommended | Render true STEP/geometries and solver fields; no render-based assertion of manufacturability |
| OpenAI docs | Public OpenAI Developers docs MCP `https://developers.openai.com/mcp` | Optional | Read-only, for up-to-date cloud/SDK docs; not needed to perform CAD |
| Computer use | Desktop/browser automation | Avoid unless unavoidable | KiCad GUI is not a reliable CI dependency; prefer APIs; manual GUI only if truly no programmatic equivalent |

## Cloud setup sequence
1. Publish a reusable Codex Cloud environment with both repos and verified permissions. Published environments are set up on desktop or web, then reusable on mobile.
2. Create a pinned, **small** base image for Python/meson/KiCad/OCC and separate heavyweight remote solver image to avoid rebuilding OpenFOAM/preCICE/CalculiX every task. Do not install everything from scratch during each agent run.
3. Build `engineering_acceptance` (#21) + `highspeed_edf.verification` (#14), confirm red baseline and mutation detection. This is the first implementation PR—not a flight design edit.
4. Add a shell/bootstrap readiness test that prints versions, SHA, disk free, memory, solver capability flags and no-cost result counts. Use machine-readable JSON output.
5. Configure required CI checks and protected test-oracle expectations. Do not allow implementation PRs to edit matching golden requirements without a separate reviewed PR.
6. Add scoped issue-to-draft-PR automation only after human-approved repo credentials/runner. Each issue should be one branch, dependency aware. Stop at draft PR when physical validation is incomplete; record whether tests are FAIL/BLOCKED and why.
7. For GCP native solvers, use explicit approved budget parameters, project/region allowlist, task-specific credentials, automatic cleanup, native logs and digest provenance; never run on every PR.
8. Keep independent reviewer separate from implementer. Reviewer must run tests rather than trust tool-report status.

## Grounded setup docs
- Codex Cloud environments: https://help.openai.com/en/articles/20001545-using-codex-cloud
- Codex repo skills + GitHub Actions: https://developers.openai.com/blog/skills-agents-sdk
- Plugin concept: https://developers.openai.com/plugins/concepts/plugins
- Packaging custom plugins: https://developers.openai.com/plugins/build/plugins
- OpenAI docs MCP: https://developers.openai.com/resources/docs-mcp
- Codex CI auto-fix reference: https://developers.openai.com/cookbook/examples/codex/autofix-github-actions

## Permissions/approval
Do not request blanket administrator/secret/cloud permissions for 'unattended' operation. Use least-privilege GitHub repo auth and narrowly allowed domains, with separate artifact-upload and paid-compute roles. A Cloud agent can run software/CAD/CFD, prepare proposed changes and draft PRs; it cannot physically validate rotor overspeed, hot battery tests, launch, or flight, and must never report those as passed without real data.
