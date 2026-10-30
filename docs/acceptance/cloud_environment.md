# Prepared cloud environment

Place source-pinned checkouts side by side as `drone-arm/` and `drone-mech/`. Read both repositories’ root and nested agent policies. The root draft instructions were read at arm PR32 commit `8c83c0f216963db8462f6241baa3b1038e1c635e` and mech PR15 commit `91946404f33daa1a77947b22b1b850f5aa765cac`.

From drone-arm:

```sh
./tools/prepare_acceptance.sh
.venv-acceptance/bin/python -m engineering_acceptance validate-inputs
.venv-acceptance/bin/python -m pytest -q tests/acceptance
.venv-acceptance/bin/python -m engineering_acceptance run --profile all
.venv-acceptance/bin/python -m engineering_acceptance report --json engineering_acceptance/reports/latest.json --junit engineering_acceptance/reports/latest.xml --html engineering_acceptance/reports/latest.html
.venv-acceptance/bin/python -m engineering_acceptance verify-evidence
```

`run` is expected to return nonzero for the known engineering failures. Generate reports even after that failure. Generated reports and the local virtual environment are ignored by git. Native raw logs, input hashes, implementation revisions, and immutable manifests remain in the run artifacts; the reports index these files.

The setup command installs the fully version-pinned Python dependency set in `requirements-cloud.lock` and probes native KiCad. This prepared Cloud environment has KiCad/pcbnew 9.0.2 (system Python `/usr/bin/python3`), CadQuery 2.5.2 and cadquery-ocp 7.7.2 in its acceptance virtual environment. Preparation succeeded here with real version probes. Missing system KiCad must be installed by the environment image owner; it cannot be supplied by a ChatGPT plugin. The CI image installs KiCad `9.0.2+dfsg-1` from a dated Debian snapshot; Python image and Actions are pinned by digest/commit. The hosted workflow itself has not yet been confirmed executed by this document.

The CI matrix covers smoke, pcb, software, cad and evidence and uploads reports even when gates fail. Its companion private checkout requires repository secret `DRONE_MECH_READ_TOKEN`, scoped read-only to drone-mech. The current GitHub integration returned HTTP 403 for listing repository secret names, so presence of this secret is unverified. No secret value should be pasted into chat or committed. Branch protection and review enforcement require repository settings; this implementation does not claim to configure them. The catalog diff guard rejects changes from the separate oracle baseline `3e10e84b321f6f8da6f72a1d177f3ddfd1a56886`.

No cloud VMs, paid solvers, physical devices, hardware energization, fabrication or flight are invoked by these commands. Historical solver receipts are inspected locally and are not evidence of a new solve. Physical evidence remains separate from digital framework tests.

## Observed hosted CI result

[Run 41008984994](https://github.com/iamdarshg/drone-arm/actions/runs/41008984994), source `271a5fc`, completed with a bootstrap failure. All five snapshot-native-tool installation steps succeeded; companion checkout failed with `Input required and not supplied: token`. No engineering checks were executed in that hosted run. The missing `DRONE_MECH_READ_TOKEN` must be configured in repository Actions secrets with read-only Contents access to drone-mech; this integration cannot administer secrets (HTTP 403). The source-pinned Cloud verification remains separate and passed its framework/mutation checks.

The workflow now checks credential presence before expensive preparation and always emits JSON/JUnit/HTML infrastructure reports when preparation fails. These reports label the environment BLOCKED and explicitly state zero engineering checks attempted. The fallback was executed locally and its JSON/XML parsed successfully; it is not a substitute for the actual engineering scorecard.
