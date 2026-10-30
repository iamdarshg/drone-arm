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
    paths = sorted(set(map(str, paths or [])))
    h = hashlib.sha256()
    manifest=[]
    for value in paths:
        try:
            sha=digest(Path(value)); h.update(value.encode() + b"\0" + sha.encode())
            manifest.append({"path":value,"sha256":sha})
        except OSError: manifest.append({"path":value,"sha256":None})
    if isinstance(measured,dict) and paths: measured.setdefault("source_artifacts",manifest)
    return {"status": status, "measured": measured, "errors": errors or [],
            "artifact_paths": paths, "artifact_sha256": h.hexdigest() if paths else None}


def _source_files(root: Path):
    skip = {".git", "__pycache__", "node_modules", ".pytest_cache", ".tox", ".mypy_cache", "build", "dist"}
    extensions={".md", ".json", ".yaml", ".yml", ".csv", ".txt", ".log"}
    for p in root.rglob("*"):
        parts=p.relative_to(root).parts
        if any(part in skip or part.startswith(".venv") or part.startswith("venv") for part in parts): continue
        if p.is_file() and p.suffix.lower() in extensions and p.stat().st_size <= 8*1024*1024:
            yield p


def claims(root: Path = MECH, kind: str = "release"):
    # Release assertions are interpreted in their source context. A string in a
    # limitation/do-not-claim list is evidence against promotion, not a claim.
    patterns = {
        "release": re.compile(r"(?i)\b(?:fabrication[- ]ready|manufacturing[- ]release|flight[- ]ready|certified)\b"),
        "aero": re.compile(r"(?i)\b(?:validated\s+(?:aero|aerodynamic|pressure recovery)|(?:measured|achieved|demonstrated|confirmed)\s+(?:native\s+)?pressure recovery|pressure[- ]recovery\s*(?:=|:|of)\s*0\.9\d*)\b"),
    }
    found=[]; sources=[]; unsupported=[]
    negative_keys={"limitations","donotclaim","prohibitedclaims","claimsnotmade","unvalidatedclaims","notclaims","knownlimitations"}
    positive_keys={"status","claim","claims","readiness","release","promotion","releaseclaim","promotionclaim"}

    def walk(value, keys=()):
        if isinstance(value,dict):
            for key,child in value.items(): yield from walk(child,keys+(str(key),))
        elif isinstance(value,list):
            for index,child in enumerate(value): yield from walk(child,keys+(str(index),))
        elif isinstance(value,str): yield value,keys

    for p in _source_files(root):
        try: text=p.read_text(errors="replace")
        except OSError: continue
        sources.append(p)
        structured=p.suffix.lower() in {".json",".yaml",".yml"}
        if structured:
            try:
                if p.suffix.lower()==".json": data=json.loads(text)
                else:
                    import yaml
                    data=yaml.safe_load(text)
            except Exception: data=None
            for value,keys in walk(data):
                if not patterns[kind].search(value): continue
                normalized=[re.sub(r"[^a-z]","",k.lower()) for k in keys]
                neg=any(k in negative_keys for k in normalized)
                positive=bool(set(normalized)&positive_keys)
                if kind=="aero":
                    tokens=set(re.sub(r"[^a-z]"," ",value.lower()).split())
                    negative=bool(tokens & {"pending","blocked","failed","invalid","unconverged","incomplete","missing","without","never","no","not"}) or "no valid" in value.lower() or "not yet" in value.lower()
                    asserted=bool(patterns[kind].search(value))
                else:
                    negative=False; asserted=positive
                record={"source":str(p.relative_to(root)),"location":".".join(keys),"text":value.strip()[:240],"context":"negative" if neg or negative else "assertion" if asserted else "unclassified"}
                found.append(record)
                if asserted and not neg and not negative:
                    unsupported.append(record)
            continue
        # In prose, require an affirmative predicate tied to a release label.
        for n,line in enumerate(text.splitlines(),1):
            if not patterns[kind].search(line): continue
            record={"source":str(p.relative_to(root)),"line":n,"text":line.strip()[:240],"context":"prose"}
            found.append(record)
            if kind=="release":
                explicit=re.search(r"(?i)\b(?:is|are|was|were|declared|considered|certified as)\b[^().!?\n]{0,40}\b(?:fabrication[- ]ready|manufacturing[- ]release|flight[- ]ready|certified)\b",line)
                negated=re.search(r"(?i)\b(?:not|never|no|without|unvalidated|remains unvalidated|do not claim)\b",line)
                if explicit and not negated: unsupported.append(record)
            else:
                explicit=patterns[kind].search(line)
                negated=re.search(r"(?i)\b(?:not|no|unvalidated|remains|without|never|not yet|pending|blocked|failed|invalid|unconverged|incomplete)\b",line)
                if explicit and not negated: unsupported.append(record)
    blockers=["no complete fabrication, structural, thermal, and physical qualification evidence bundle is indexed"] if kind=="release" else ["no converged applicable native aero result with raw fields, BCs, residual checks, and sensitivity evidence"]
    # Promotion cannot pass from claim text; only an independently verified evidence bundle can.
    status="FAIL" if unsupported else "BLOCKED"
    errors=["positive claim lacks required native and physical evidence"] if unsupported else blockers
    return _result(status,{"claims":found,"unsupported_positive_claims":unsupported,"source_files_scanned":len(sources),"gates":blockers},errors,sources)


def _receipt_files(root: Path):
    return sorted(root.glob("attempts/*/worker_output/**/*receipt*.json")) + sorted(root.glob("attempts/*/worker_output/*receipt*.json"))


def fsi(root: Path = FSI):
    receipts=[]; raw=[]; errors=[]; stale=[]; forged=[]
    for rp in sorted(set(_receipt_files(root))):
        try: obj=json.loads(rp.read_text())
        except Exception: errors.append(f"invalid receipt: {rp}"); continue
        status=str(obj.get("status", "")).lower()
        if ("complete" in status and "incomplete" not in status and "missing" not in status) or status in {"pass","passed","coupled_solvers_completed"}:
            coupling=obj.get("coupling",{})
            candidate_dir=rp.parent/str(obj.get("candidate", ""))
            base=candidate_dir if candidate_dir.is_dir() else rp.parent
            logs=[]
            for key in ("fluidLog","solidLog"):
                val=coupling.get(key) or obj.get(key)
                matches=list(base.rglob(Path(str(val)).name)) if val else []
                if len(matches)==1 and matches[0].is_file(): logs.append(matches[0])
            fluid=next((p for p in logs if p.name==Path(str(coupling.get("fluidLog",""))).name),None)
            strong=(len(logs)==2 and all(p.stat().st_size>0 for p in logs)
                    and coupling.get("fluidReturnCode")==0 and coupling.get("solidReturnCode")==0
                    and coupling.get("timedOut") is False and coupling.get("fluidReachedEnd") is True
                    and isinstance(coupling.get("preCICECompletionLogMarkers"),int) and coupling["preCICECompletionLogMarkers"]>0
                    and isinstance(obj.get("tipDisplacement"),(int,float))
                    and obj.get("forceHistoryLastLine") not in (None,"")
                    and isinstance(obj.get("inputFiles"),list) and bool(obj["inputFiles"]))
            fluid_ok=bool(fluid and re.search(r"(?m)^End\s*$",fluid.read_text(errors="replace"))) if fluid else False
            input_ok=True
            for item in obj.get("inputFiles", []):
                if not isinstance(item,dict) or not item.get("path") or not item.get("sha256"):
                    input_ok=False; break
                source=rp.parent/item["path"]
                if not source.is_file() or digest(source)!=item["sha256"]:
                    stale.append(str(source)); input_ok=False; break
                raw.append(source)
            if not strong or not fluid_ok or not input_ok: forged.append(str(rp))
        raw_entries=obj.get("rawFiles", obj.get("files", []))
        for item in raw_entries:
            if isinstance(item,dict) and item.get("path"):
                p=rp.parent/item["path"]
                if not p.is_file(): errors.append(f"missing raw file {p}")
                elif not item.get("sha256"): errors.append(f"raw evidence has no recorded SHA-256: {p}")
                elif digest(p)!=item["sha256"]: stale.append(str(p))
                else: raw.append(p)
        raw.append(rp)
        receipts.append({"path":str(rp),"status":obj.get("status"),"keys":sorted(obj)})
    # Independently inspect actual candidate logs/data; summaries alone never count as solver completion.
    for pattern in ("rhoPimpleFoam.log", "ccx_preCICE.log"):
        for log in root.glob(f"attempts/*/worker_output/**/{pattern}"):
            raw.append(log)
            # File content is inspected here; receipt claims still decide whether a run completed.
            try: log.read_text(errors="replace")
            except OSError: continue
    complete=[]
    for r in receipts:
        if str(r["status"]).lower() in {"coupled_solvers_completed","completed","pass","passed"}: complete.append(r)
    if forged: errors.append("completion receipt is unsubstantiated: candidate logs, solver return/completion markers, input SHA provenance, or physical histories are missing or inconsistent: " + ", ".join(forged))
    if stale: errors.append("receipt SHA mismatch (stale or altered raw evidence): " + ", ".join(stale))
    failures=[r for r in receipts if any(token in str(r["status"]).lower() for token in ("failed", "timeout", "timed_out", "deadline", "not_converged"))]
    if not complete and failures: errors.append(f"{len(failures)} native FSI receipt(s) record failed, timed out, or unconverged execution")
    elif not complete and not receipts: errors.append("no FSI receipts or native run records were found")
    # Existing known results include timed-out/failed attempts. Require numeric physical history + convergence records.
    qualified=[]
    for r in complete:
        rp=Path(r["path"]); base=rp.parent
        logs=list(base.rglob("rhoPimpleFoam.log"))+list(base.rglob("ccx_preCICE.log"))
        dat=list(base.rglob("wing.dat"))
        if len(logs)>=2 and dat and all(p.stat().st_size for p in logs+dat): qualified.append(r)
    if complete and not qualified: errors.append("completed label has no paired native participant logs and displacement history")
    status="FAIL" if forged or stale or failures else "PASS" if qualified and not errors else "BLOCKED"
    return _result(status,{"receipts":receipts,"completed_receipts":len(complete),"failed_or_timed_out_receipts":len(failures),"qualified_runs":len(qualified),"raw_log_count":len(raw),"stale_raw_files":stale},errors,raw+[Path(r["path"]) for r in receipts])


def thermal(arm: Path = ARM, config=None):
    cfg=arm/"engineering_acceptance/aircraft_system.yaml"
    try:
        if config is None: import yaml; config=yaml.safe_load(cfg.read_text())
        obj=config
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


def cost(arm: Path = ARM, mech: Path = MECH, today=None, config=None):
    """Audit current zero-launch policy separately from historical governed job caps."""
    from datetime import date, datetime, timezone
    if today is None: today = date.today()
    elif isinstance(today, str): today = date.fromisoformat(today)
    try:
        if config is None: import yaml; config=yaml.safe_load((arm/"engineering_acceptance/aircraft_system.yaml").read_text())
        cfg=config
    except Exception as exc: return _result("FAIL",{"config_error":str(exc)},["cost policy unreadable"])
    policy=cfg.get("cost_policy",{}); fsi_root=mech/"highspeed_edf/phase15/fsi/results"
    history_path=fsi_root/"phase15_fsi_cost_history.json"; paths=[arm/"engineering_acceptance/aircraft_system.yaml"]
    history=[]; malformed=[]
    if history_path.is_file():
        paths.append(history_path)
        try:
            loaded=json.loads(history_path.read_text()); history=loaded if isinstance(loaded,list) else loaded.get("jobs",[])
        except Exception: malformed.append(str(history_path))
    by_id={str(row.get("jobId")):row for row in history if isinstance(row,dict) and row.get("jobId")}
    attempts=[]; current=[]; historical_errors=[]; historical_unresolved=[]; current_launches=[]
    launch_re=re.compile(r"(?m)^(\d{4}-\d{2}-\d{2}T[^ ]+) .*VM_CREATE_REQUEST")
    for log in sorted(fsi_root.glob("attempts/*/gcp_attempt.log")):
        paths.append(log); text=log.read_text(errors="replace")
        match=launch_re.search(text)
        if not match:
            if "VM_CREATE_REQUEST" in text or re.search(r"(?i)created instance|create request",text):
                malformed.append(f"launch log lacks an attributable UTC launch timestamp: {log}")
            continue
        try: launched=datetime.fromisoformat(match.group(1)).date()
        except ValueError: malformed.append(str(log)); continue
        attempt_id=log.parent.name
        rec=next((v for k,v in by_id.items() if attempt_id in k),None)
        if rec is None:
            # The worker history timestamp may differ from its attempt directory by a few seconds.
            try: stamp=datetime.strptime(attempt_id.rstrip("Z"),"%Y%m%dT%H%M%S")
            except ValueError: stamp=None
            candidates=[]
            if stamp:
                for key,row in by_id.items():
                    m=re.search(r"(20\d{6}T\d{6})Z?",key)
                    if m:
                        try: delta=abs((datetime.strptime(m.group(1),"%Y%m%dT%H%M%S")-stamp).total_seconds())
                        except ValueError: continue
                        if delta <= 5: candidates.append((delta,row))
            rec=min(candidates,key=lambda pair:pair[0])[1] if candidates else None
        entry={"attempt":attempt_id,"launched_at":match.group(1),"epoch":"current" if launched>=today else "historical","history_receipt":rec}
        attempts.append(entry)
        if launched>=today:
            current.append(entry); current_launches.append(entry)
        else:
            cap_match=re.search(r"workbenchCostCeilingUsd=([0-9.]+)",text)
            total=rec.get("allInConservativeEstimateUsd") if rec else None
            if not cap_match:
                historical_unresolved.append(f"historical launch {attempt_id} has no recorded governed cost ceiling")
            elif isinstance(total,(int,float)) and total>float(cap_match.group(1))+1e-9:
                historical_errors.append(f"historical launch {attempt_id} exceeds its recorded per-job cost ceiling")
            elif total is None:
                historical_unresolved.append(f"historical launch {attempt_id} has no attributable cost receipt")
            deleted=bool(re.search(r"VM_DELETE rc=0|VM_STOPPED status=DELETED",text))
            if rec and rec.get("resourceReleased") is True: deleted=True
            if not deleted:
                historical_unresolved.append(f"historical launch {attempt_id} lacks a verified teardown receipt")
    zero=policy.get("remote_compute_budget_usd")==0 and policy.get("allow_remote_compute") is False
    errors=[]
    if malformed: errors.append("malformed execution or cost-history receipt")
    if not zero: errors.append("current policy must explicitly disable new paid compute with zero budget")
    if current_launches: errors.append("new paid compute was launched during the current zero-budget policy epoch")
    errors.extend(historical_errors)
    errors.extend(historical_unresolved)
    # The standalone latest receipt is useful for the newest run but does not override per-attempt history.
    latest=fsi_root/"phase15_fsi_execution_receipt.json"
    if latest.is_file():
        paths.append(latest)
        try:
            latest_obj=json.loads(latest.read_text())
            latest_status=latest_obj.get("status")
            latest_released=latest_obj.get("workbenchCostReceipt",{}).get("resourceReleased")
            latest_started=latest_obj.get("startedAtUtc")
            attempts_meta={"status":latest_status,"started_at":latest_started,"resource_released":latest_released,"observed_runner_seconds":latest_obj.get("observedRunnerSeconds"),"reserved_usd":latest_obj.get("conservativeBudgetUsd"),"invoice_actual_usd":latest_obj.get("workbenchCostReceipt",{}).get("actualCloudInvoiceUsd")}
        except Exception: malformed.append(str(latest)); attempts_meta={}
    else: attempts_meta={}
    # Missing invoice reconciliation is disclosed, while bounded compute estimates and teardown receipts remain auditable.
    status="FAIL" if malformed or current_launches or historical_errors or not zero else "BLOCKED" if historical_unresolved else "PASS"
    return _result(status,{"current_policy":policy,"policy_epoch_date":today.isoformat(),"current_paid_launches":current_launches,"historical_governed_launches":len(attempts)-len(current_launches),"historical_job_count_with_receipts":len(by_id),"historical_cost_or_teardown_errors":historical_errors,"historical_unresolved_receipts":historical_unresolved,"latest_execution":attempts_meta,"cost_history_job_count":len(history)},errors,paths)


def run(check: dict, profile: str = "smoke", config=None):
    """Direct runner API: execute only bounded local evidence inspection; never launches compute."""
    ident=check.get("id","")
    if ident=="BASE-019": return claims()
    if ident=="BASE-020": return claims(kind="aero")
    if ident=="BASE-021": return fsi()
    if ident=="BASE-023": return thermal(config=config)
    if ident=="BASE-024": return cost(config=config)
    return _result("BLOCKED",{"check":ident,"profile":profile},[f"unsupported evidence check {ident}"])
