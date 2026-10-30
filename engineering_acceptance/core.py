"""Deterministic catalog, validation, execution, and append-only evidence ledger."""
from __future__ import annotations

import hashlib, json, math, os, subprocess, time, importlib.metadata, platform, shlex, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from . import __version__
from .units import UNITS

ROOT = Path(__file__).resolve().parents[1]
EA = ROOT / "engineering_acceptance"
CATALOG = EA / "catalog"
LEDGER = EA / "reports" / "evidence.jsonl"
CONFIG_PATH = EA / "aircraft_system.yaml"
STATES = {"PASS", "FAIL", "BLOCKED", "NOT_RUN", "NOT_APPLICABLE"}
# Immutable issue-21 oracle digest set. Update only with the independently reviewed catalog revision.
LOCKED_CATALOG_SHA256 = {
    "mandatory_checks.yaml": "9baf04ed130248d68c4c3c5c88f537ce0db8cc69c1051581aff02abec66cfc23",
    "profiles.yaml": "b337e0705a6218bb523fe0824ddc27459e71dc6582254e1d69602c329bac97ba",
    "requirements.yaml": "bb930791d68a79e3a4bcbfb4803760147a41884a0edb25923db4bcdab6c5cd15",
    "test_owners.yaml": "54d2d6d534c2dcbe8dcfd1cad975f617678b93993a73f9c6a387a0f069e36e1b",
}


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def load_yaml(path: Path) -> Any:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def finite_tree(value: Any, where: str = "root") -> list[str]:
    errors = []
    if isinstance(value, float) and not math.isfinite(value):
        errors.append(f"{where}: NaN and infinity are forbidden")
    elif isinstance(value, dict):
        for k, v in value.items(): errors += finite_tree(v, f"{where}.{k}")
    elif isinstance(value, list):
        for i, v in enumerate(value): errors += finite_tree(v, f"{where}[{i}]")
    return errors


def verify_source_artifacts(config: dict) -> dict:
    """Compare declared engineering inputs byte-for-byte with pinned git trees."""
    artifacts=config.get("source_artifacts",{})
    sources={"drone-arm":(ROOT,config.get("source_of_truth",{}).get("drone_arm",{}).get("revision")),
             "drone-mech":(ROOT.parent/"drone-mech",config.get("source_of_truth",{}).get("drone_mech",{}).get("revision"))}
    errors=[]; measured=[]; aggregate=hashlib.sha256()
    if not isinstance(artifacts,dict) or not artifacts:
        return {"status":"FAIL","aggregate_sha256":None,"artifacts":[],"errors":["source_artifacts must declare pinned engineering input files"]}
    for key,entry in sorted(artifacts.items()):
        if not isinstance(entry,dict): errors.append(f"source_artifacts.{key}: expected a source record"); continue
        repo_name,revision,rel=entry.get("repo"),entry.get("revision"),entry.get("path")
        if repo_name not in sources or not isinstance(rel,str) or not rel: errors.append(f"source_artifacts.{key}: invalid repo or path"); continue
        repo,pinned=sources[repo_name]
        if revision != pinned: errors.append(f"source_artifacts.{key}: revision does not equal immutable repo baseline"); continue
        try: raw=subprocess.check_output(["git","-C",str(repo),"ls-tree","-rz","-r",revision,"--",rel])
        except (OSError,subprocess.CalledProcessError) as exc: errors.append(f"source_artifacts.{key}: cannot read pinned git tree: {exc}");continue
        expected={}
        for item in raw.split(b"\0"):
            if not item: continue
            meta,path=item.split(b"\t",1);fields=meta.decode().split()
            if len(fields)==3 and fields[1]=="blob": expected[path.decode()]=fields[2]
        if not expected: errors.append(f"source_artifacts.{key}: pinned path has no files: {rel}");continue
        target=repo/rel
        actual={str(target.relative_to(repo)):target} if target.is_file() else ({str(p.relative_to(repo)):p for p in target.rglob("*") if p.is_file()} if target.is_dir() else {})
        if set(actual)!=set(expected): errors.append(f"source_artifacts.{key}: checked-out files differ from pinned tree (missing={sorted(set(expected)-set(actual))}, extra={sorted(set(actual)-set(expected))})")
        for relpath,path in sorted(actual.items()):
            try: data=path.read_bytes()
            except OSError as exc: errors.append(f"source_artifacts.{key}: cannot read {relpath}: {exc}");continue
            blob=hashlib.sha1(b"blob "+str(len(data)).encode()+b"\0"+data).hexdigest();digest=hashlib.sha256(data).hexdigest()
            matches=expected.get(relpath)==blob
            if not matches: errors.append(f"source_artifacts.{key}: dirty/altered source {relpath}; expected Git blob {expected.get(relpath)}, observed {blob}")
            measured.append({"key":key,"repo":repo_name,"revision":revision,"path":relpath,"sha256":digest,"git_blob_sha1":blob,"matches_pinned":matches})
            aggregate.update(key.encode()+b"\0"+relpath.encode()+b"\0"+digest.encode()+b"\0")
    digest=aggregate.hexdigest() if measured else None
    declared=config.get("cad",{}).get("source_sha256",{})
    if isinstance(declared,dict) and declared.get("value") is not None and declared.get("value") != digest:
        errors.append(f"cad.source_sha256.value does not match computed source aggregate {digest}")
    return {"status":"FAIL" if errors else "PASS","aggregate_sha256":digest,"artifacts":measured,"errors":errors}


def run_input_drift(start: dict, end: dict) -> list[str]:
    """Return integrity errors if repository heads or pinned input digests changed mid-run."""
    errors=[]
    for repo in ("drone_arm","drone_mech"):
        if start.get("heads",{}).get(repo) != end.get("heads",{}).get(repo):
            errors.append(f"{repo} HEAD changed during run: {start.get('heads',{}).get(repo)} -> {end.get('heads',{}).get(repo)}")
    for key in ("config_sha256","test_definition_sha256","source_artifacts_sha256"):
        if start.get(key) != end.get(key): errors.append(f"{key} changed during run")
    return errors


def catalog_files() -> list[Path]:
    return sorted(CATALOG.glob("*.yaml")) if CATALOG.exists() else []


def discover() -> dict:
    checks_path = CATALOG / "mandatory_checks.yaml"
    checks = load_yaml(checks_path) if checks_path.exists() else []
    if isinstance(checks, dict): checks = checks.get("checks", [])
    return {"requirements": load_yaml(CATALOG / "requirements.yaml") if (CATALOG / "requirements.yaml").exists() else [],
            "checks": checks, "profiles": load_yaml(CATALOG / "profiles.yaml") if (CATALOG / "profiles.yaml").exists() else {},
            "path": str(CATALOG)}


def validate_inputs(config_path: Path | None = None) -> list[str]:
    errors: list[str] = []
    config_path = config_path or CONFIG_PATH
    if not config_path.exists(): return [f"missing config: {config_path}"]
    try: config = load_yaml(config_path)
    except (OSError, yaml.YAMLError) as e: return [f"config parse error: {e}"]
    if not isinstance(config, dict): return ["config must be a mapping"]
    errors += finite_tree(config)
    try:
        from .contracts import validate as validate_contract
        validate_contract(config)
    except (ImportError, ValueError) as e:
        errors.extend(getattr(e, "errors", (str(e),)))
    source_integrity=verify_source_artifacts(config)
    errors.extend(source_integrity["errors"])
    for name in ("drone_arm", "drone_mech"):
        repo = ROOT if name == "drone_arm" else ROOT.parent / "drone-mech"
        actual = git_head(repo)
        declared = config.get("source_of_truth", {}).get(name, {}).get("revision")
        if not isinstance(declared, str) or len(declared) != 40 or any(ch not in "0123456789abcdef" for ch in declared):
            errors.append(f"source_of_truth.{name}.revision must be a full lowercase git SHA")
        if not actual: errors.append(f"source repository {name} has no readable git HEAD")
    for rel in ("requirements.yaml", "mandatory_checks.yaml", "profiles.yaml", "test_owners.yaml"):
        path = CATALOG / rel
        if not path.exists():
            errors.append(f"catalog file missing: {path.relative_to(ROOT)}")
        elif sha256(path) != LOCKED_CATALOG_SHA256.get(rel):
            errors.append(f"catalog file digest does not match the independently reviewed oracle lock: {path.relative_to(ROOT)}")
    try: data = discover()
    except (OSError, yaml.YAMLError) as e: return [f"catalog parse error: {e}"]
    requirements = data["requirements"]
    if isinstance(requirements, dict): requirements = requirements.get("requirements", [])
    checks = data["checks"]
    seen: set[str] = set()
    for i, check in enumerate(checks):
        where = f"check[{i}]"
        if not isinstance(check, dict): errors.append(f"{where} must be an object"); continue
        for field in ("id", "source", "severity", "input_paths", "runner", "expected", "tolerance", "unit", "applicability", "blocking", "evidence_required", "owner_repo"):
            if field not in check: errors.append(f"{where} missing required field {field}")
        ident = check.get("id")
        if not isinstance(ident, str) or not ident: errors.append(f"{where}.id must be a nonempty string")
        elif ident in seen: errors.append(f"duplicate check id {ident}")
        else: seen.add(ident)
        unit = check.get("unit")
        if unit is not None and unit not in UNITS: errors.append(f"{where} has unsupported unit {unit!r}")
        if check.get("requirement_id") and check["requirement_id"] not in {r.get("id") for r in requirements if isinstance(r, dict)}:
            errors.append(f"{where} references unknown requirement {check['requirement_id']}")
        errors += finite_tree(check, where)
    try:
        # JSON Schema validation is an additional check when the optional validator is installed.
        import jsonschema
        schema = json.loads((EA / "schema" / "requirement.schema.json").read_text())
        checker = jsonschema.FormatChecker()
        for i, check in enumerate(checks):
            try: jsonschema.validate(check, schema, format_checker=checker)
            except jsonschema.ValidationError as e: errors.append(f"check[{i}] schema: {e.message}")
        reqs = requirements if isinstance(requirements, list) else []
        for i, requirement in enumerate(reqs):
            try: jsonschema.validate(requirement, schema, format_checker=checker)
            except jsonschema.ValidationError as e: errors.append(f"requirement[{i}] schema: {e.message}")
        try:
            aircraft_schema=json.loads((EA/"schema"/"aircraft_system.schema.json").read_text())
            jsonschema.validate(config, aircraft_schema, format_checker=checker)
        except jsonschema.ValidationError as e: errors.append(f"aircraft_system schema: {e.message}")
    except ImportError:
        pass
    return errors


def git_head(repo: Path) -> str | None:
    try: return subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError): return None


def record_result(row: dict) -> None:
    LEDGER.parent.mkdir(parents=True, exist_ok=True)
    previous = None
    if LEDGER.exists():
        for line in LEDGER.read_text(encoding="utf-8").splitlines():
            if line.strip():
                try: previous = json.loads(line).get("record_sha256")
                except json.JSONDecodeError: previous = "INVALID"
    row["previous_record_sha256"] = previous
    digest_input = {k: v for k, v in row.items() if k != "record_sha256"}
    row["record_sha256"] = hashlib.sha256(json.dumps(digest_input, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    from .contracts import validate_result
    validate_result(row)
    with LEDGER.open("a", encoding="utf-8") as f:
        f.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")


def run(profile: str, config_path: Path | None = None) -> dict:
    try: data = discover()
    except (OSError, yaml.YAMLError) as e: raise ValueError(f"acceptance catalog cannot be parsed: {e}") from e
    checks = data["checks"]
    if not checks: raise ValueError("no acceptance check catalog found")
    profiles = data["profiles"]
    if isinstance(profiles, dict) and "profiles" in profiles: profiles = profiles["profiles"]
    selected = profiles.get(profile) if isinstance(profiles, dict) else None
    if isinstance(selected, dict): selected = selected.get("checks", [])
    if not selected: raise ValueError(f"profile {profile!r} has no mapped checks")
    by_id = {c["id"]: c for c in checks}
    config_path = config_path or CONFIG_PATH
    try: config = load_yaml(config_path)
    except (OSError, yaml.YAMLError): config = {}
    source_heads={"drone_arm":git_head(ROOT),"drone_mech":git_head(ROOT.parent/"drone-mech")}
    start_source=verify_source_artifacts(config)
    start_lock={"heads":source_heads,"config_sha256":sha256(config_path) if config_path.is_file() else None,
                "test_definition_sha256":sha256(CATALOG/"mandatory_checks.yaml") if (CATALOG/"mandatory_checks.yaml").is_file() else None,
                "source_artifacts_sha256":start_source.get("aggregate_sha256")}
    validation = validate_inputs(config_path)
    run_id = hashlib.sha256(f"{time.time_ns()}:{source_heads['drone_arm']}:{profile}".encode()).hexdigest()
    results = []
    for ident in selected:
        c = by_id.get(ident)
        if c is None:
            row = {"id": ident, "status": "FAIL", "errors": ["profile references undeclared check"]}
        elif validation:
            row = {"id": ident, "requirement_id": c.get("requirement_id"), "status": "FAIL", "measured": None,
                   "unit": c.get("unit"), "required": c.get("expected"), "source": c.get("source"), "runner": c.get("runner"),
                   "command": "python -m engineering_acceptance validate-inputs", "errors": ["invalid catalog/input", *validation]}
        else:
            from .runners.local import execute
            row = {"id": ident, "requirement_id": c.get("requirement_id"), "unit": c.get("unit"), "required": c.get("expected"), "source": c.get("source"), "runner": c.get("runner")}
            row.update(execute(c, profile=profile, config=config, run_id=run_id,
                               source_heads=source_heads, config_path=config_path))
        if c:
            source_repo = c.get("source", {}).get("repo") if isinstance(c.get("source"), dict) else c.get("owner_repo")
            row["source_revision"] = source_heads["drone_arm" if source_repo == "drone-arm" else "drone_mech"]
            artifacts = []
            for rel in c.get("input_paths", []):
                for base in (ROOT, ROOT.parent / "drone-mech"):
                    candidate = base / rel
                    if candidate.is_file(): artifacts.append({"path": str(candidate.relative_to(ROOT.parent)), "sha256": sha256(candidate)})
                    elif candidate.is_dir():
                        repo = ROOT if base == ROOT else ROOT.parent / "drone-mech"
                        artifacts.append({"path": f"git:{repo.name}", "sha256": source_heads["drone_arm" if repo == ROOT else "drone_mech"], "kind": "git_commit"})
            for path in row.get("artifact_paths", []):
                candidate = Path(path)
                if not candidate.is_absolute(): candidate = (ROOT / candidate).resolve()
                if candidate.is_file(): artifacts.append({"path": str(candidate.relative_to(ROOT.parent)), "sha256": sha256(candidate)})
            row["artifact_paths"] = [a["path"] for a in artifacts]
            row["artifact_sha256"] = hashlib.sha256("".join(a["sha256"] for a in artifacts).encode()).hexdigest() if artifacts else None
            row["artifacts"] = artifacts
            row["evidence_required"] = bool(c.get("evidence_required"))
        tool_versions = {p: importlib.metadata.version(p) for p in ("PyYAML", "jsonschema")}
        tool_versions.update(row.get("tool_versions", {}) if isinstance(row.get("tool_versions"), dict) else {})
        row.update({"run_id": run_id, "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                    "drone_arm_sha": source_heads["drone_arm"], "drone_mech_sha": source_heads["drone_mech"],
                    "catalog_sha256": start_lock["test_definition_sha256"],
                    "test_definition_sha256": start_lock["test_definition_sha256"],
                    "config_path":str(config_path.resolve()),"config_sha256":start_lock["config_sha256"],
                    "config_present":config_path.is_file(),
                    "tool_version": __version__, "python_version": platform.python_version(),
                    "tool_versions": tool_versions,
                    "profile": profile})
        results.append(row)
    end_source=verify_source_artifacts(config)
    end_heads={"drone_arm":git_head(ROOT),"drone_mech":git_head(ROOT.parent/"drone-mech")}
    end_lock={"heads":end_heads,"config_sha256":sha256(config_path) if config_path.is_file() else None,
              "test_definition_sha256":sha256(CATALOG/"mandatory_checks.yaml") if (CATALOG/"mandatory_checks.yaml").is_file() else None,
              "source_artifacts_sha256":end_source.get("aggregate_sha256")}
    drift_errors=run_input_drift(start_lock,end_lock)+end_source["errors"]
    if drift_errors:
        for row in results:
            row["status"]="FAIL"
            row.setdefault("errors",[]).append("run invalidated by input/source drift: "+"; ".join(drift_errors))
            row["run_integrity"]={"ok":False,"errors":drift_errors,"heads_at_start":source_heads,"heads_at_end":end_heads}
    attempted = sum(r["status"] not in ("NOT_RUN", "NOT_APPLICABLE") for r in results); counts = {s: sum(r["status"] == s for r in results) for s in STATES}
    reqs=data["requirements"].get("requirements",[]) if isinstance(data["requirements"],dict) else data["requirements"]
    critical=[c for c in checks if c.get("id") in set(selected) and c.get("blocking") and c.get("severity") in ("P0","critical")]
    critical_pass=sum(any(row.get("id")==c["id"] and row.get("status")=="PASS" for row in results) for c in critical)
    timestamp=datetime.now(timezone.utc).isoformat()
    for row in results:
        check=by_id.get(row["id"],{})
        invocation=shlex.join([sys.executable,"-m","engineering_acceptance","run","--profile",profile,"--config",str(config_path.resolve())])
        row.update({"system":"twin_electric_ducted_fan_aircraft","variant":check.get("applicability"),"severity":check.get("severity"),"applicability":check.get("applicability"),"blocking":check.get("blocking"),
                    "repository_heads_at_end":end_heads,"run_integrity_ok":not drift_errors,
                    "exact_command":invocation,"check_exit_code":0 if row["status"]=="PASS" else 1 if row["status"]=="FAIL" else None})
        if row.get("command") is None: row["command"]=invocation
        result_path=EA/"reports"/"runs"/run_id/"results"/(row["id"]+".json")
        result_path.parent.mkdir(parents=True,exist_ok=True)
        result_payload=json.dumps({k:row.get(k) for k in ("id","status","measured","errors","runner","command","exact_command","source","source_revision")},sort_keys=True,indent=2,allow_nan=False)+"\n"
        with result_path.open("x",encoding="utf-8") as f:f.write(result_payload)
        result_artifact={"path":str(result_path.relative_to(ROOT.parent)),"sha256":sha256(result_path)}
        row.setdefault("artifacts",[]).append(result_artifact)
        row["artifact_paths"]=[a["path"] for a in row["artifacts"]]
        row["artifact_sha256"]=hashlib.sha256("".join(a["sha256"] for a in row["artifacts"]).encode()).hexdigest()
        if isinstance(row.get("command"), (list, tuple)): row["command"]=shlex.join([str(x) for x in row["command"]])
    planned_manifest=EA/"reports"/"runs"/f"{run_id}.json"
    for row in results:
        row["run_manifest_path"]=str(planned_manifest.relative_to(ROOT));row["run_manifest_sha256"]="0"*64
        try:
            from .contracts import validate_result
            validate_result(row)
        except (ImportError, ValueError) as exc:
            row["status"]="FAIL";row["check_exit_code"]=1
            row.setdefault("errors",[]).append("runner result rejected by evidence contract: "+str(exc))
    counts = {s: sum(r["status"] == s for r in results) for s in STATES}
    attempted = sum(r["status"] not in ("NOT_RUN", "NOT_APPLICABLE") for r in results)
    critical_pass=sum(any(row.get("id")==c["id"] and row.get("status")=="PASS" for row in results) for c in critical)
    manifest={"run_id":run_id,"timestamp_utc":timestamp,"profile":profile,"drone_arm_sha":source_heads["drone_arm"],"drone_mech_sha":source_heads["drone_mech"],
              "repository_heads_at_end":end_heads,"integrity_ok":not drift_errors,
              "test_definition_sha256":start_lock["test_definition_sha256"],"config_path":str(config_path.resolve()),"config_sha256":start_lock["config_sha256"],"config_present":config_path.is_file(),"tool_version":__version__,"python_version":platform.python_version(),
              "tool_versions":tool_versions,"results":[{"id":r["id"],"status":r["status"],"artifact_sha256":r.get("artifact_sha256"),"check_exit_code":r.get("check_exit_code"),"evidence_required":r.get("evidence_required")} for r in results]}
    manifest_path=planned_manifest;manifest_path.parent.mkdir(parents=True,exist_ok=True)
    try:
        from .contracts import validate_manifest
        validate_manifest(manifest)
    except (ImportError, ValueError) as exc:
        raise ValueError(f"run manifest rejected by evidence contract: {exc}") from exc
    payload=json.dumps(manifest,sort_keys=True,indent=2,allow_nan=False)+"\n"
    with manifest_path.open("x",encoding="utf-8") as f:f.write(payload)
    manifest_sha=hashlib.sha256(payload.encode()).hexdigest()
    for row in results: row["run_manifest_path"]=str(manifest_path.relative_to(ROOT));row["run_manifest_sha256"]=manifest_sha
    for row in results: record_result(row)
    return {"run_id": run_id, "profile": profile, "timestamp_utc":timestamp,"drone_arm_sha": source_heads["drone_arm"], "drone_mech_sha": source_heads["drone_mech"],
            "requirements_declared": len(reqs), "checks_discovered": len(checks), "checks_attempted": attempted,
            "counts": counts,"release_critical_pass":critical_pass,"release_critical_total":len(critical),"integrity_ok":not drift_errors,"repository_heads_at_end":end_heads,"integrity_errors":drift_errors,
            "manifest_path":str(manifest_path.relative_to(ROOT)),"manifest_sha256":manifest_sha,
            "results": results, "notice": "Acceptance report is not flight or certification approval."}


def latest_run() -> list[dict]:
    if not LEDGER.exists(): return []
    rows = [json.loads(line) for line in LEDGER.read_text().splitlines() if line.strip()]
    if not rows: return []
    latest = rows[-1]["run_id"]
    return [r for r in rows if r.get("run_id") == latest]


def verify_ledger() -> list[str]:
    errors = []
    if not LEDGER.exists(): return ["evidence ledger does not exist"]
    previous = None
    for n, line in enumerate(LEDGER.read_text().splitlines(), 1):
        try:
            row = json.loads(line, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
            digest_payload = {k: v for k, v in row.items() if k != "record_sha256"}
            expected_record_sha = hashlib.sha256(json.dumps(digest_payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
            if row.get("previous_record_sha256") != previous: errors.append(f"line {n}: ledger hash chain broken")
            if row.get("record_sha256") != expected_record_sha: errors.append(f"line {n}: record digest mismatch")
            previous = row.get("record_sha256")
            if row.get("status") not in STATES: errors.append(f"line {n}: invalid status")
            if not row.get("run_id") or not row.get("drone_arm_sha"): errors.append(f"line {n}: missing run/source SHA")
            for field in ("drone_mech_sha", "test_definition_sha256", "exact_command", "timestamp_utc", "runner", "check_exit_code"):
                if field not in row: errors.append(f"line {n}: missing required manifest field {field}")
            if row.get("status") == "PASS" and not row.get("artifact_sha256") and row.get("evidence_required", True): errors.append(f"line {n}: PASS lacks evidence artifact SHA")
            manifest_path = ROOT / row.get("run_manifest_path", "")
            if not manifest_path.is_file(): errors.append(f"line {n}: immutable run manifest missing")
            elif sha256(manifest_path) != row.get("run_manifest_sha256"): errors.append(f"line {n}: run manifest SHA mismatch")
            for artifact in row.get("artifacts", []):
                if artifact.get("kind") == "git_commit":
                    repo = ROOT if artifact.get("path") == f"git:{ROOT.name}" else ROOT.parent / artifact["path"].removeprefix("git:")
                    try:
                        subprocess.run(["git", "-C", str(repo), "cat-file", "-e", artifact["sha256"]+"^{commit}"], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    except (OSError, subprocess.CalledProcessError): errors.append(f"line {n}: git artifact commit missing: {artifact['path']}")
                    continue
                path = (ROOT.parent / artifact["path"]).resolve()
                if not path.is_file(): errors.append(f"line {n}: artifact missing: {artifact['path']}")
                elif sha256(path) != artifact.get("sha256"): errors.append(f"line {n}: artifact SHA mismatch: {artifact['path']}")
            combined=hashlib.sha256("".join(a.get("sha256", "") for a in row.get("artifacts", [])).encode()).hexdigest() if row.get("artifacts") else None
            if combined != row.get("artifact_sha256"): errors.append(f"line {n}: artifact set digest mismatch")
        except (ValueError, json.JSONDecodeError) as e: errors.append(f"line {n}: invalid JSON: {e}")
    return errors
