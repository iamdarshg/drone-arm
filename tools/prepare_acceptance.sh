#!/usr/bin/env bash
# No paid compute and no physical devices are invoked by this setup.
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m venv .venv-acceptance
.venv-acceptance/bin/python -m pip install -r engineering_acceptance/requirements-cloud.lock
.venv-acceptance/bin/python - <<'PROBE'
import importlib.metadata as m, json, shutil, subprocess
print(json.dumps({p:m.version(p) for p in ['cadquery','cadquery-ocp','PyYAML','jsonschema','pytest']},sort_keys=True))
for cmd in [['kicad-cli','version'], ['/usr/bin/python3','-c','import pcbnew; print(pcbnew.Version())']]:
    r=subprocess.run(cmd,check=False,text=True,capture_output=True)
    print(json.dumps({'command':cmd,'exit_code':r.returncode,'stdout':r.stdout,'stderr':r.stderr}))
    if r.returncode: raise SystemExit('BLOCKED: install KiCad 9.0.2 CLI and system pcbnew before native PCB checks')
PROBE
