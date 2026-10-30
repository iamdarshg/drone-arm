"""Source-driven audits for claims, native solver receipts, thermal evidence and cloud cost."""
from __future__ import annotations
import hashlib, json, re
from pathlib import Path
from typing import Any

ARM = Path(__file__).resolve().parents[2]
MECH = ARM.parent / "drone-mech"
FSI = MECH / "highspeed_edf/phase15/fsi/results"


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def _result(status: str, measured: Any, errors: list[str] | None = None, paths=None):
    paths = paths or []
    h = hashlib.sha256()
    for p in sorted(set(map(str, paths))):
        try: h.update(p.encode() + b"\0" + digest(Path(p)).encode())
        except OSError: pass
    return {"status": status, "measured": measured, "errors": errors or [],
            "artifact_paths": [str(p) for p in paths], "artifact_sha256": h.hexdigest() if paths else None}


def _source_files(root: Path):
    skip = {".git", "__pycache__", "node_modules"}
    for p in root.rglob("*"):
        if p.is_file() and not any(x in skip for x in p.parts) and p.suffix.lower() in {".md", ".json", ".yaml", ".yml", ".csv", ".txt", ".log"}:
            yield p


def claims(root: Path = MECH, kind: str = "release"):
    patterns = {
        "release": re.compile(r"(?i)\b(?:fabrication[- ]ready|manufacturing[- ]release|flight[- ]ready|certified|validated)\b"),
        "aero": re.compile(r"(?i)\b(?:validated\s+(?:aero|aerodynamic)|pressure[- ]recovery|recovery\s*(?:=|:|of)\s*0\.9)\b"),
    }
    found=[]; sources=[]
    for p in _source_files(root):
        try: text=p.read_text(errors="replace")
        except OSError: continue
        sources.append(p)
        for n,line in enumerate(text.splitlines(),1):
            if patterns[kind].search(line): found.append({"source":str(p.relative_to(root)),"line":n,"text":line.strip()[:240]})
    # This audit deliberately reports the claim inventory and gates every positive claim.
    blockers=[]
    if kind == "release": blockers=["no complete fabrication, structural, thermal, and physical qualification evidence bundle is indexed"]
    else: blockers=["no converged applicable native aero result with raw fields, BCs, residual checks, and sensitivity evidence"]
    # Count explicit affirmative status/claim syntax; prose discussing promotion rules and
    # strings in quoted negative statements are inventory, not assertions.
    affirmative = re.compile(r'''(?i)(?:\b(?:is|are|status|result|pipeline status|claim)\s*[:=]?\s*(?:\*\*)?)(?:fabrication[- ]ready|manufacturing[- ]release|flight[- ]ready|certified|validated)|["\'](?:flight[- ]ready|manufacturing[- ]release|fabrication[- ]ready)["\']\s*[,} ]?''')
    unsupported=[c for c in found if affirmative.search(c["text"]) and not re.search(r"(?i)(?:not|no|unvalidated|remains|without|never|not yet|not .*validated)",c["text"])]
    status="FAIL" if unsupported else "BLOCKED" if found else "PASS"
    return _result(status,{"claims":found,"unsupported_positive_claims":unsupported,"source_files_scanned":len(sources),"gates":blockers},["positive claim lacks required native and physical evidence" ] if unsupported else blockers if found else [])


def _receipt_files(root: Path):
    return sorted(root.glob("attempts/*/worker_output/**/*receipt*.json")) + sorted(root.glob("attempts/*/worker_output/*receipt*.json"))


def fsi(root: Path = FSI):
    receipts=[]; raw=[]; errors=[]; stale=[]; forged=[]
    for rp in sorted(set(_receipt_files(root))):
        try: obj=json.loads(rp.read_text())
        except Exception: errors.append(f"invalid receipt: {rp}"); continue
        status=str(obj.get("status", "")).lower()
        if ("complete" in status and "incomplete" not in status and "missing" not in status) or status in {"pass","passed","coupled_solvers_completed"}:
            logs=[]
            base=rp.parent
            for key in ("fluidLog","solidLog"):
                val=obj.get("coupling",{}).get(key) or obj.get(key)
                if val:
                    candidate=base / val
                    if candidate.is_file(): logs.append(candidate)
            if not logs or not all(p.stat().st_size > 0 for p in logs): forged.append(str(rp))
        for item in obj.get("rawFiles", obj.get("files", [])):
            if isinstance(item,dict) and item.get("path"):
                p=rp.parent/item["path"]
                if not p.is_file(): errors.append(f"missing raw file {p}")
                elif item.get("sha256") and digest(p)!=item["sha256"]: stale.append(str(p))
                else: raw.append(p)
        receipts.append({"path":str(rp),"status":obj.get("status"),"keys":sorted(obj)})
    # Independently inspect actual candidate logs/data; summaries alone never count as solver completion.
    for log in root.glob("attempts/*/worker_output/**/rhoPimpleFoam.log"):
        raw.append(log)
        try: text=log.read_text(errors="replace")
        except OSError: continue
        if re.search(r"End\s*\n",text): pass
    complete=[]
    for r in receipts:
        if str(r["status"]).lower() in {"coupled_solvers_completed","completed","pass","passed"}: complete.append(r)
    if forged: errors.append("completion receipt lacks both nonempty participant logs: " + ", ".join(forged))
    if stale: errors.append("receipt SHA mismatch (stale or altered raw evidence): " + ", ".join(stale))
    if not complete: errors.append("no completed FSI receipt; observed receipts are timed out, failed, or absent")
    # Existing known results include timed-out/failed attempts. Require numeric physical history + convergence records.
    qualified=[]
    for r in complete:
        rp=Path(r["path"]); base=rp.parent
        logs=list(base.rglob("rhoPimpleFoam.log"))+list(base.rglob("ccx_preCICE.log"))
        dat=list(base.rglob("wing.dat"))
        if len(logs)>=2 and dat and all(p.stat().st_size for p in logs+dat): qualified.append(r)
    if complete and not qualified: errors.append("completed label has no paired native participant logs and displacement history")
    status="FAIL" if forged or stale else "PASS" if qualified and not errors else "BLOCKED"
    return _result(status,{"receipts":receipts,"completed_receipts":len(complete),"qualified_runs":len(qualified),"raw_log_count":len(raw),"stale_raw_files":stale},errors,raw+[Path(r["path"]) for r in receipts])


def thermal(arm: Path = ARM):
    cfg=arm/"engineering_acceptance/aircraft_system.yaml"
    try: import yaml; obj=yaml.safe_load(cfg.read_text())
    except Exception as exc: return _result("FAIL",{"config_error":str(exc)},["cannot parse declared system temperature evidence"])
    value=obj.get("motors",{}).get("temperature_measurements",{})
    # Need a real mapped sensor source and raw measurement, values in source units and provenance.
    sensor_sources=[p for p in _source_files(arm) if "sensor" in p.name.lower() or "thermal" in p.name.lower()]
    sourced=any(re.search(r"(?i)(motor|esc).{0,40}(?:thermistor|temperature sensor|NTC)",p.read_text(errors="ignore")) for p in sensor_sources)
    measured=isinstance(value,dict) and value.get("value") is not None and value.get("source") and value.get("raw_measurement")
    errors=[]
    if not sourced: errors.append("no sourced installed motor/ESC temperature sensor map")
    if not measured: errors.append("no sourced raw motor temperature measurement or unit conversion provenance")
    return _result("PASS" if sourced and measured else "BLOCKED",{"declared":value,"sensor_source_files":[str(x) for x in sensor_sources],"sensor_map_found":sourced,"measurement_complete":bool(measured)},errors,[cfg]+sensor_sources)


def cost(arm: Path = ARM, mech: Path = MECH):
    try: import yaml; cfg=yaml.safe_load((arm/"engineering_acceptance/aircraft_system.yaml").read_text())
    except Exception as exc: return _result("FAIL",{"config_error":str(exc)},["cost policy unreadable"])
    policy=cfg.get("cost_policy",{}); jobs=[]; malformed=[]; paths=[]
    fsi_root=mech/"highspeed_edf/phase15/fsi/results"
    for p in sorted(fsi_root.glob("attempts/*/gcp_attempt.log")):
        paths.append(p); text=p.read_text(errors="replace")
        if re.search(r"(?i)(?:instance|operation|job)\s+(?:created|started)|create\s+request|launchRequested",text): jobs.append({"path":str(p),"launched":True})
    for p in (fsi_root/"phase15_fsi_execution_receipt.json",):
        if p.is_file():
            paths.append(p)
            try:
                r=json.loads(p.read_text()); jobs.append({"path":str(p),"status":r.get("status"),"observed_runner_seconds":r.get("observedRunnerSeconds"),"actual_cloud_invoice_usd":r.get("workbenchCostReceipt",{}).get("actualCloudInvoiceUsd"),"reserved_usd":r.get("conservativeBudgetUsd")})
            except Exception: malformed.append(str(p))
    logs_with_launch=sum(1 for p in paths if p.name=="gcp_attempt.log" and re.search(r"(?i)(?:create|launch|instance)",p.read_text(errors="ignore")))
    blocked=not policy.get("allow_remote_compute",False) and logs_with_launch>0
    zero=policy.get("remote_compute_budget_usd")==0 and policy.get("allow_remote_compute") is False
    errors=[]
    if malformed: errors.append("malformed remote execution receipt")
    if blocked: errors.append("remote launch evidence contradicts the disabled/zero-budget acceptance policy")
    elif not zero: errors.append("default policy must explicitly disable remote compute with zero budget")
    if any(x.get("actual_cloud_invoice_usd") is None and x.get("observed_runner_seconds") for x in jobs): errors.append("remote compute was launched but invoice actual is unresolved; observed runner time and reserved ceiling are retained")
    status="FAIL" if blocked or malformed or not zero else "BLOCKED" if jobs else "PASS"
    return _result(status,{"policy":policy,"observed_launch_log_count":logs_with_launch,"jobs_and_receipts":jobs,"malformed":malformed},errors,paths+[(arm/"engineering_acceptance/aircraft_system.yaml")])


def run(check: dict, profile: str = "smoke"):
    """Direct runner API: execute only bounded local evidence inspection; never launches compute."""
    ident=check.get("id","")
    if ident=="BASE-019": return claims()
    if ident=="BASE-020": return claims(kind="aero")
    if ident=="BASE-021": return fsi()
    if ident=="BASE-023": return thermal()
    if ident=="BASE-024": return cost()
    return _result("BLOCKED",{"check":ident,"profile":profile},[f"unsupported evidence check {ident}"])
