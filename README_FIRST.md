# Engineering acceptance runner

This repository has a local, evidence-gated acceptance runner for drone-arm#21. It reports measured checks as `PASS`, `FAIL`, `BLOCKED`, `NOT_RUN`, or `NOT_APPLICABLE`. A blocked physical or native-solver requirement is never treated as success. Reports do not approve flight, fabrication, or certification.

## Run the local smoke profile

From the repository root, bootstrap the pinned local tools and run smoke:

```sh
./tools/prepare_acceptance.sh
.venv-acceptance/bin/python -m engineering_acceptance validate-inputs
.venv-acceptance/bin/python -m engineering_acceptance run --profile smoke
.venv-acceptance/bin/python -m engineering_acceptance report --json engineering_acceptance/reports/latest.json --junit engineering_acceptance/reports/latest.xml --html engineering_acceptance/reports/latest.html
```

The main typed system input is [aircraft_system.yaml](engineering_acceptance/aircraft_system.yaml). Catalog definitions live in `engineering_acceptance/catalog/`; the catalog is an immutable source of requirements and profiles. Keep unknown physical values as `null` with a `BLOCKED` status and a reason. Record any evidence-backed override with its source and revision.

For a sourced configuration variant, pass one file explicitly: `.venv-acceptance/bin/python -m engineering_acceptance validate-inputs --config /path/to/aircraft_system.yaml` and `.venv-acceptance/bin/python -m engineering_acceptance run --profile cad --config /path/to/aircraft_system.yaml`. Every run records the exact config path and SHA-256.

## Profiles and evidence

Supported profiles are `smoke`, `cad`, `pcb`, `software`, `thermal`, `power`, `aero`, `fsi`, and `all`. The default cost policy has a zero dollar remote-compute budget and disables remote compute. Smoke does not contact cloud services or hardware. Native CAD, PCB, solver, and physical checks report `BLOCKED` when their adapter or raw evidence is unavailable.

Every run appends JSONL records to `engineering_acceptance/reports/evidence.jsonl`. Each record includes the current drone-arm and drone-mech revisions, catalog digest, command, runner, status, and hashes of input artifacts when available. Verify the ledger with:

```sh
.venv-acceptance/bin/python -m engineering_acceptance verify-evidence
```

Other commands:

```sh
.venv-acceptance/bin/python -m engineering_acceptance discover
.venv-acceptance/bin/python -m engineering_acceptance explain BASE-008
.venv-acceptance/bin/python -m engineering_acceptance diff baseline.json current.json
```

For a future SIL matrix, provide an explicit YAML file whose values each include `value`, `unit`, and `source`, then run `generate-matrix --input matrix.yaml --output matrix.json --seed 17`. The pairwise planner only creates deterministic, uniquely identified cases. Every case stays `NOT_RUN` until a validated model executes it; it never supplies operating values or claims coverage of a flight envelope.

An actual `FAIL` exits nonzero. `BLOCKED` remains visible as unresolved evidence and does not produce a pass claim. `NOT_RUN` is reserved for checks not attempted; it is excluded from the attempted count.

## Setup and boundaries

`./tools/prepare_acceptance.sh` installs the version-pinned tools listed in `engineering_acceptance/requirements-cloud.txt` for the configured environment. Do not start heavy remote profiles without a separately authorized, budgeted receipt policy. Do not energize hardware or start physical tests through this runner.
