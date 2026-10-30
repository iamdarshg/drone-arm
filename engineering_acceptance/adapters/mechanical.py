"""Adapters that import pinned native drone-mech STEP and drone-arm PCB data."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import uuid
from pathlib import Path

from ..core import ROOT

MECH_ROOT = ROOT.parent / "drone-mech"
MECH_ADAPTER = MECH_ROOT / "highspeed_edf/verification/native_adapters.py"
CAD_GEOMETRY = MECH_ROOT / "highspeed_edf/deliverables/phase15_geometry"
STEP_DIR = CAD_GEOMETRY / "step"
ARM_ROOT = ROOT
PCB_FILES = {
    "esc": ARM_ROOT / "hardware/esc/rev_b/esc_rev_b.kicad_pcb",
    "main": ARM_ROOT / "hardware/main/rev_b/main_rev_b.kicad_pcb",
}
NATIVE_REPORTS = ROOT / "engineering_acceptance/reports/native"


def _sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1024 * 1024), b""):
            h.update(b)
    return h.hexdigest()


def _native():
    spec = importlib.util.spec_from_file_location("mech_native_adapters", MECH_ADAPTER)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load mechanical adapter {MECH_ADAPTER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _base(check, command, tool_versions=None):
    return {"status": "BLOCKED", "measured": None, "errors": [], "artifact_paths": [],
            "artifact_sha256": None, "command": command, "tool_versions": tool_versions or {}}


def _configured_path(config: dict | None, key: str, default: Path) -> Path:
    if not config:
        return default
    artifact = config.get("source_artifacts", {}).get(key)
    if artifact is None and key in ("esc_board", "main_board"):
        candidate = config.get("boards", {}).get("selected_esc_source" if key == "esc_board" else "selected_main_source")
        if candidate:
            artifact = {"repo": "drone-arm", "path": candidate}
    if not artifact:
        return default
    if isinstance(artifact, str):
        artifact = {"repo": "drone-arm", "path": artifact}
    base = MECH_ROOT if artifact.get("repo") == "drone-mech" else ARM_ROOT
    return (base / artifact["path"]).resolve()


def run(check: dict, profile: str, config: dict | None = None, run_id: str | None = None) -> dict:
    """Dispatch the mechanical check requested by runner.py.

    CADQUERY_PYTHON can select the isolated, pinned CadQuery environment.
    Results are raw native measurements; known source conflicts remain FAIL.
    """
    ident = check.get("id", "")
    name = check.get("runner", "")
    if isinstance(name, dict):
        name = name.get("name", name.get("capability", ""))
    result = _base(check, [sys.executable, "engineering_acceptance.adapters.mechanical.run", ident])
    run_id = run_id or uuid.uuid4().hex
    safe_run_id = "".join(c for c in run_id if c.isalnum() or c in "-_" ) or uuid.uuid4().hex
    selected_pcb = {"esc": _configured_path(config, "esc_board", PCB_FILES["esc"]),
                    "main": _configured_path(config, "main_board", PCB_FILES["main"])}
    if name == "kicad_drc" or ident == "BASE-018":
        cli = os.environ.get("KICAD_CLI", "/usr/bin/kicad-cli")
        if not Path(cli).is_file():
            result["errors"] = [f"BLOCKED: native kicad-cli executable not found: {cli}"]
            return result
        run_reports = NATIVE_REPORTS / safe_run_id
        run_reports.mkdir(parents=True, exist_ok=True)
        measurements = {}
        raw_paths = []
        commands = []
        try:
            version = subprocess.run([cli, "version"], text=True, capture_output=True, check=True).stdout.strip()
            for label, board in selected_pcb.items():
                if not board.is_file():
                    result["errors"].append(f"BLOCKED: native {label} PCB is absent: {board}")
                    continue
                raw = run_reports / f"{label}-native-drc.json"
                command = [cli, "pcb", "drc", "--severity-all", "--exit-code-violations", "--format", "json", "--output", str(raw), str(board)]
                commands.append(command)
                proc = subprocess.run(command, text=True, capture_output=True)
                if proc.returncode not in (0, 5) or not raw.is_file():
                    result["errors"].append(f"{label} DRC tool failure exit {proc.returncode}: {proc.stderr.strip()}")
                    continue
                data = json.loads(raw.read_text())
                data["board_sha256"] = _sha(board)
                data["command"] = command
                raw.write_text(json.dumps(data, indent=2) + "\n")
                raw_paths.append(str(raw))
                measurements[label] = {"board_sha256": data["board_sha256"],
                                       "kicad_version": data.get("kicad_version"),
                                       "unconnected_items": len(data.get("unconnected_items", [])),
                                       "violations": len(data.get("violations", [])),
                                       "exit_code": proc.returncode,
                                       "source": str(board)}
            total = sum(x["unconnected_items"] + x["violations"] for x in measurements.values())
            status = "FAIL" if result["errors"] or total else "PASS"
            result.update(status=status, measured={"boards": measurements, "total_drc_and_open_net_findings": total},
                          command=commands, artifact_paths=raw_paths,
                          artifact_sha256={str(p): _sha(Path(p)) for p in raw_paths},
                          tool_versions={"kicad_cli": version})
            if total:
                result["errors"].append(f"native KiCad DRC found {total} DRC/open-net findings")
            return result
        except (OSError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
            result["status"] = "BLOCKED" if isinstance(exc, OSError) else "FAIL"
            result["errors"].append(f"{type(exc).__name__}: {exc}")
            return result
    if name in ("kicad_edge_cuts",) or ident in ("BASE-005", "BASE-006"):
        board = selected_pcb["esc" if ident == "BASE-005" else "main"]
        if not board.is_file():
            result["errors"] = [f"BLOCKED: selected native PCB is absent: {board}"]
            return result
        try:
            adapter = _native()
            measured = adapter.import_kicad_board(board)
            # At these pinned sources, only geometry declared directly in the PCB is authoritative.
            expected_sha = check.get("expected_sha256")
            errors = []
            sha_result = adapter.validate_pcb_sha(measured["sha256"], expected_sha) if expected_sha else None
            if sha_result and sha_result["status"] == "FAIL":
                errors.extend(sha_result["errors"])
            # Compare authoritative Edge.Cuts against the dimensions the CAD generator
            # currently embeds. This catches the known ESC placeholder mismatch.
            placeholder = (150.0, 150.0) if ident == "BASE-005" else (140.0, 100.0)
            comparison = adapter.board_placeholder_comparison(measured, placeholder)
            measured["cad_placeholder_comparison"] = comparison
            if comparison["status"] == "FAIL":
                errors.append("native PCB Edge.Cuts dimensions differ from the CAD board placeholder")
            result.update(status="FAIL" if errors else "PASS", measured=measured, errors=errors)
            result["command"] = measured.get("command")
            result["artifact_paths"] = [str(board)]
            result["artifact_sha256"] = measured["sha256"]
            result["tool_versions"] = {"kicad_pcbnew": measured["version"], "python": "/usr/bin/python3"}
            return result
        except Exception as exc:
            result["status"] = "BLOCKED" if isinstance(exc, (ImportError, FileNotFoundError)) else "FAIL"
            result["errors"] = [f"{type(exc).__name__}: {exc}"]
            return result
    if name in ("cad_occ", "cad_interference", "battery_bay_fit", "assembly_access", "cad_mass_cg") or ident in ("BASE-004", "BASE-008", "BASE-009", "BASE-010", "BASE-011", "BASE-012"):
        generator = _configured_path(config, "geometry_generator", MECH_ROOT / "highspeed_edf/phase15/optimization/phase15_generate_geometry.py")
        parameter_register = _configured_path(config, "parameter_register", CAD_GEOMETRY / "reports/phase15_parameter_register.json")
        configured_step = (config or {}).get("cad", {}).get("step_root")
        if isinstance(configured_step, dict):
            configured_step = configured_step.get("value")
        geometry_dir = (MECH_ROOT / configured_step).resolve() if configured_step else CAD_GEOMETRY
        if geometry_dir.name == "step":
            geometry_dir = geometry_dir.parent
        step_dir = geometry_dir / "step"
        masscsv = _configured_path(config, "mass_properties", geometry_dir / "reports/mass_cg.csv")
        if not MECH_ADAPTER.is_file() or not geometry_dir.is_dir():
            result["errors"] = [f"BLOCKED: native CAD source missing: {geometry_dir}"]
            return result
        executable = os.environ.get("CADQUERY_PYTHON", sys.executable)
        raw_path = MECH_ROOT / "highspeed_edf/verification/evidence/runs" / safe_run_id
        raw_path.mkdir(parents=True, exist_ok=True)
        if ident == "BASE-008":
            raw_path = raw_path / "native_motor_interference.json"
            code = ("from pathlib import Path; import json; from highspeed_edf.verification.native_adapters "
                    "import import_step,motor_bbox_collision; p=Path(" + repr(str(step_dir)) + "); "
                    "m=[import_step(x) for x in sorted(p.glob('motor_*_3670_*_envelope.step'))]; "
                    "r=motor_bbox_collision(m); Path(" + repr(str(raw_path)) + ").write_text(json.dumps(r,indent=2)+'\\n'); "
                    "print(json.dumps({k:r[k] for k in ('status','pairs_checked','collisions','clearance_failures')}))")
            command = [executable, "-c", code]
        elif ident == "BASE-004":
            raw_path = raw_path / "native_step_import.json"
            datum_path = parameter_register
            code = ("from pathlib import Path; import json; from highspeed_edf.verification.native_adapters "
                    "import import_all_step,validate_cad_datum; r=import_all_step(Path(" + repr(str(geometry_dir)) + ")); "
                    "r['datum']=validate_cad_datum(Path(" + repr(str(datum_path)) + ")); "
                    "r['status']='FAIL' if r['status']=='FAIL' or r['datum']['status']=='FAIL' else 'PASS'; r['errors'] += r['datum']['errors']; "
                    "Path(" + repr(str(raw_path)) + ").write_text(json.dumps(r,indent=2)+'\\n'); "
                    "print(json.dumps({k:r[k] for k in ('status','file_count','imported_count','valid_count','datum','errors','tool_versions','python_executable')}))")
            command = [executable, "-c", code]
        elif ident == "BASE-009":
            raw_path = raw_path / "native_duct_body_interference.json"
            code = ("from pathlib import Path; import json; from highspeed_edf.verification.native_adapters "
                    "import boolean_interference; p=Path(" + repr(str(step_dir)) + "); "
                    "r=[boolean_interference(p/'fuselage_battery_bay_shell.step',p/f'duct_{d}_82OD_78ID_L110.step') for d in ('A','B')]; "
                    "o={'status':'FAIL' if any(x['status']=='FAIL' for x in r) else 'PASS','ducts':r}; "
                    "Path(" + repr(str(raw_path)) + ").write_text(json.dumps(o,indent=2)+'\\n'); print(json.dumps(o))")
            command = [executable, "-c", code]
        elif ident == "BASE-010":
            raw_path = raw_path / "native_battery_bay_fit.json"
            gen = generator
            code = ("from pathlib import Path; import json; from highspeed_edf.verification.native_adapters "
                    "import battery_bay_fit; r=battery_bay_fit(Path(" + repr(str(geometry_dir)) + "),Path(" + repr(str(gen)) + ")); "
                    "Path(" + repr(str(raw_path)) + ").write_text(json.dumps(r,indent=2)+'\\n'); print(json.dumps(r))")
            command = [executable, "-c", code]
        elif ident == "BASE-011":
            raw_path = raw_path / "native_connector_access.json"
            code = ("from pathlib import Path; import json; from highspeed_edf.verification.native_adapters "
                    "import connector_access_screen; r=connector_access_screen(Path(" + repr(str(geometry_dir)) + ")); "
                    "Path(" + repr(str(raw_path)) + ").write_text(json.dumps(r,indent=2)+'\\n'); print(json.dumps(r))")
            command = [executable, "-c", code]
        elif ident == "BASE-012":
            raw_path = raw_path / "native_mass_cg.json"
            gen = generator
            code = ("from pathlib import Path; import json; from highspeed_edf.verification.native_adapters "
                    "import recalculate_mass_cg; r=recalculate_mass_cg(Path(" + repr(str(geometry_dir)) + "),Path(" + repr(str(masscsv)) + "),Path(" + repr(str(gen)) + ")); "
                    "Path(" + repr(str(raw_path)) + ").write_text(json.dumps(r,indent=2)+'\\n'); "
                    "print(json.dumps({k:r[k] for k in ('status','total_mass_kg','cg_mm','component_count','components_using_mass_override','errors')}))")
            command = [executable, "-c", code]
        else:
            code = ("from pathlib import Path; import json; from highspeed_edf.verification.native_adapters "
                    "import import_step; p=Path(" + repr(str(step_dir)) + "); paths=[p/'phase15_airframe_full_assembly.step',"
                    "p/'duct_A_82OD_78ID_L110.step',p/'duct_B_82OD_78ID_L110.step']; "
                    "print(json.dumps([import_step(x) for x in paths]))")
            command = [executable, "-c", code]
        env = dict(os.environ)
        env["PYTHONPATH"] = str(MECH_ROOT) + os.pathsep + env.get("PYTHONPATH", "")
        # Execute in chosen pinned CadQuery interpreter. A missing binary/package is BLOCKED.
        probe = subprocess.run([executable, "-c", "import cadquery, importlib.metadata, json; print(json.dumps({'cadquery': importlib.metadata.version('cadquery'), 'cadquery-ocp': importlib.metadata.version('cadquery-ocp')}))"],
                               text=True, capture_output=True, env=env)
        result["command"] = command
        if probe.returncode:
            result["status"] = "BLOCKED"
            result["errors"] = [f"native CadQuery unavailable (exit {probe.returncode}): {probe.stderr.strip()}"]
            return result
        result["tool_versions"] = json.loads(probe.stdout)
        proc = subprocess.run(command, text=True, capture_output=True, env=env)
        if proc.returncode:
            result["status"] = "FAIL"
            result["errors"] = [f"STEP adapter command exited {proc.returncode}: {proc.stderr.strip()}"]
            return result
        evidence = json.loads(proc.stdout)
        if raw_path.is_file():
            raw_evidence = json.loads(raw_path.read_text())
            raw_evidence["command"] = command
            raw_evidence["runner_exit_code"] = proc.returncode
            raw_evidence.setdefault("tool_versions", result["tool_versions"])
            raw_path.write_text(json.dumps(raw_evidence, indent=2) + "\n")
        if ident in ("BASE-008", "BASE-009", "BASE-010", "BASE-011", "BASE-012"):
            result["measured"] = evidence
            result["artifact_paths"] = [str(raw_path)] if raw_path.is_file() else [str(STEP_DIR)]
            result["artifact_sha256"] = _sha(raw_path) if raw_path.is_file() else hashlib.sha256(json.dumps(evidence, sort_keys=True).encode()).hexdigest()
            result["status"] = evidence.get("status", "BLOCKED")
            result["errors"] = evidence.get("errors", [])
            if ident == "BASE-008" and evidence.get("collisions"):
                result["errors"].append(f"native motor Boolean found {len(evidence['collisions'])} intersecting pairs")
            return result
        result["measured"] = evidence
        result["artifact_paths"] = [str(raw_path)]
        result["artifact_sha256"] = _sha(raw_path) if raw_path.is_file() else None
        result["status"] = evidence["status"]
        result["errors"] = evidence["errors"]
        return result
    return result
