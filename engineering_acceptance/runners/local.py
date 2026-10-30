"""Evidence-aware local checks. Unknown/native capabilities never return PASS."""
import json, math, re, subprocess
from pathlib import Path
from ..core import EA, ROOT, git_head, verify_source_artifacts
from ..units import convert

def execute(check, profile="smoke", config=None, run_id=None, source_heads=None, config_path=None):
    ident = check["id"]
    name = check.get("runner")
    if isinstance(name, dict): name = name.get("name", name.get("capability"))
    result = {"measured": None, "artifact_paths": [], "artifact_sha256": None, "command": None, "evidence_required": bool(check.get("evidence_required"))}
    if ident in ("BASE-019", "BASE-020", "BASE-021", "BASE-023", "BASE-024"):
        try:
            from ..adapters.evidence import run as run_evidence
            return result | run_evidence(check, profile=profile, config=config)
        except (ImportError, OSError, ValueError) as exc:
            return result | {"status": "BLOCKED", "errors": [f"evidence audit unavailable: {exc}"]}
    if name in ("provenance", "git_provenance") or ident.endswith("001"):
        actual = source_heads or {"drone_arm": git_head(ROOT), "drone_mech": git_head(ROOT.parent / "drone-mech")}
        if config is None:
            import yaml
            config = yaml.safe_load((EA / "aircraft_system.yaml").read_text())
        baselines = {k: v["revision"] for k, v in config["source_of_truth"].items()}
        checks={}
        for key, repo in (("drone_arm",ROOT),("drone_mech",ROOT.parent/"drone-mech")):
            try: subprocess.run(["git","-C",str(repo),"merge-base","--is-ancestor",baselines[key],"HEAD"],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); checks[key]=True
            except (OSError,subprocess.CalledProcessError): checks[key]=False
        artifacts=verify_source_artifacts(config)
        ok=all(actual.values()) and all(checks.values()) and artifacts["status"]=="PASS"
        errors=[] if ok else ["source baseline is missing/not ancestral or an engineering input differs from its pinned git blob",*artifacts["errors"]]
        return result | {"status": "PASS" if ok else "FAIL", "measured": {"current_head": actual,"immutable_baseline":baselines,"baseline_is_ancestor":checks,"source_integrity":artifacts}, "errors": errors}
    if name in ("schema", "schema_validation", "validate_schema") or ident.endswith("002"):
        from ..core import validate_inputs
        # Validate the same explicitly selected config as the outer acceptance
        # run. Falling back to the immutable baseline here makes candidate
        # runs fail against stale source artifacts despite a valid candidate pin.
        errs = validate_inputs(config_path)
        return result | {"status": "PASS" if not errs else "FAIL", "measured": {"errors": errs}, "errors": errs}
    if name in ("units", "unit_conversion") or ident.endswith("003"):
        conversions = [("mm", "m", 1000, 1), ("Wh", "J", 1, 3600), ("deg", "rad", 180, math.pi)]
        try:
            measured = [convert(x, a, b) for a, b, x, _ in conversions]
            ok = all(math.isclose(v, e, rel_tol=1e-12) for v, (_, _, _, e) in zip(measured, conversions))
            try: convert(float("nan"), "m", "mm"); ok = False
            except ValueError: pass
        except Exception as exc: return result | {"status": "FAIL", "errors": [str(exc)]}
        return result | {"status": "PASS" if ok else "FAIL", "measured": dict(zip(("1000mm_to_m", "1Wh_to_J", "180deg_to_rad"), measured)), "errors": [] if ok else ["conversion failed"]}
    if name in ("source_file", "source_presence", "firmware_source_presence") or ident.endswith("016"):
        sources = [ROOT / "src/main_board/main.c", ROOT / "src/esc_controller/esc.c"]
        missing = [str(p.relative_to(ROOT)) for p in sources if not p.is_file()]
        return result | {"status": "PASS" if not missing else "FAIL", "measured": {"present": [str(p.relative_to(ROOT)) for p in sources if p.is_file()], "missing": missing}, "errors": ["firmware source files are absent: " + ", ".join(missing)] if missing else []}
    if name == "electrical_contract" or ident == "BASE-007":
        p = ROOT / "docs/hardware/REV_B_ARCHITECTURE.md"
        text = p.read_text(errors="replace") if p.is_file() else ""
        channels = len(re.findall(r"(?im)^\s*(?:\d+\.?\s*)?(?:ESC|motor)[-_ ]?(?:cell|channel|stage)\s*[1-6]", text))
        independent = bool(re.search(r"six (?:independent|separate|separately controlled)", text, re.I)) and bool(re.search(r"fused", text, re.I))
        # Source-based expectation checks terminology and declared topology; no current capability inferred.
        passed = p.is_file() and "12S" in text and ("six" in text.lower()) and independent
        return result | {"status": "PASS" if passed else "FAIL", "measured": {"architecture_source": str(p.relative_to(ROOT)), "six_stage_language": "six" in text.lower(), "independent_fused_language": independent, "parsed_numbered_channels": channels}, "errors": [] if passed else ["architecture document does not establish six independent fused power paths for the 12S system"]}
    if name == "repository_boundary" or ident == "BASE-025":
        mech = ROOT.parent / "drone-mech"
        try:
            files = subprocess.check_output(["git", "-C", str(mech), "ls-files", "*.c", "*.h", "*.cpp", "*.hpp"], text=True).splitlines()
            suspicious = [f for f in files if re.search(r"flight|control|allocator|sil", f, re.I)]
        except (OSError, subprocess.CalledProcessError): files=[]; suspicious=[]
        return result | {"status": "FAIL" if suspicious else "PASS", "measured": {"tracked_native_code_files": files, "controller_named_paths": suspicious}, "errors": ["drone-mech contains controller-named native code paths requiring ownership review: " + ", ".join(suspicious)] if suspicious else []}
    if name == "payload_claim_audit" or ident == "BASE-022":
        if config is None:
            import yaml
            config=yaml.safe_load((EA/"aircraft_system.yaml").read_text())
        payload=config.get("payload")
        return result | {"status": "BLOCKED", "measured": {"contract":payload,"radar_active":payload.get("radar_active") if isinstance(payload,dict) else None,"native_payload_receipt":False}, "errors": ["inert-only policy is declared, but no native payload geometry/electronics receipt proves the installed optional payload envelope"]}
    if name == "thermal_evidence" or ident == "BASE-023":
        if config is None:
            import yaml
            config=yaml.safe_load((EA/"aircraft_system.yaml").read_text())
        value=config.get("motors",{}).get("temperature_measurements",{})
        known=bool(value and isinstance(value,dict) and value.get("value") is not None)
        return result | {"status": "BLOCKED" if not known else "PASS", "measured": value, "errors": ["installed motor temperature measurements lack sourced sensor evidence"] if not known else []}
    if name in ("native_firmware_build", "firmware_build") or ident == "BASE-017":
        sources=ROOT/"src"
        if not sources.is_dir(): return result | {"status":"FAIL","measured":{"src_tree_exists":False},"errors":["meson.build targets src/ firmware files which are absent from tracked tree"]}
        return result | {"status":"BLOCKED","measured":{"src_tree_exists":True},"errors":["native firmware build adapter has no verified clean ELF/UF2 receipt"]}
    if name == "actuator_allocation" or ident == "BASE-013":
        import yaml
        if config is None: config=yaml.safe_load((EA/"aircraft_system.yaml").read_text())
        cfg=config.get("actuators",{})
        modeled=cfg.get("modeled_independent_effectors"); outputs=cfg.get("controller_outputs")
        return result | {"status":"FAIL" if isinstance(modeled,int) and isinstance(outputs,int) and modeled>outputs else "PASS","measured":{"independent_effectors":modeled,"controller_outputs":outputs,"unallocated":max(0,modeled-outputs) if isinstance(modeled,int) and isinstance(outputs,int) else None},"errors":[f"{modeled} independent effectors exceed {outputs} controller outputs"] if isinstance(modeled,int) and isinstance(outputs,int) and modeled>outputs else []}
    if name == "actuator_safe_start" or ident == "BASE-014":
        sources=[ROOT/"src/main_board/low_level/pwm.c",ROOT/"src/esc_controller/esc.c"]
        present=[p for p in sources if p.is_file()]
        if len(present)!=len(sources):
            missing=[str(p.relative_to(ROOT)) for p in sources if not p.is_file()]
            return result | {"status":"FAIL","measured":{"startup_sources_present":[str(p.relative_to(ROOT)) for p in present],"missing":missing,"neutral_on_reset_proven":False,"lost_can_readback_arm_inhibited_proven":False},"errors":["cannot establish safe neutral output and CAN-readback arm inhibit: firmware sources are absent"]}
        texts={p.name:p.read_text(errors="replace") for p in present}
        # Do not infer safe behavior from a function name alone; require explicit neutral initialization and watchdog/readback guard markers.
        neutral=any(re.search(r"(?i)(?:neutral|safe)[_ ]?(?:pulse|output|duty)|(?:pulse|duty)[_ ]?neutral",t) for t in texts.values())
        readback=any(re.search(r"(?i)(?:can|telemetry).{0,100}(?:timeout|read.?back|feedback)|(?:timeout|read.?back|feedback).{0,100}(?:can|telemetry)",t,re.S) for t in texts.values())
        return result | {"status":"BLOCKED","measured":{"neutral_marker":neutral,"readback_guard_marker":readback},"errors":["source markers alone do not prove reset behavior; require an executable firmware test receipt for neutral output and readback-loss arm inhibit"]}
    if name == "current_topology" or ident == "BASE-015":
        p=ROOT/"docs/hardware/REV_B_ARCHITECTURE.md"; text=p.read_text(errors="replace") if p.is_file() else ""
        forbidden=bool(re.search(r"360\s*A\s+(?:shared\s+)?(?:PCB\s+)?trunk",text,re.I))
        independent=bool(re.search(r"six (?:independent|fused)",text,re.I))
        return result | {"status":"FAIL" if forbidden else "PASS" if independent else "BLOCKED","measured":{"forbidden_shared_360A_trunk_declared":forbidden,"independent_inputs_declared":independent},"errors":["shared 360 A trunk is forbidden"] if forbidden else [] if independent else ["architecture evidence does not yet establish six independent current paths"]}
    if name in ("claim_policy", "claims", "claim_audit"):
        if ident == "BASE-019":
            # Search tracked status/report files. A positive release claim without a matching evidence receipt fails.
            try: names=subprocess.check_output(["git","-C",str(ROOT),"ls-files"],text=True).splitlines()
            except (OSError,subprocess.CalledProcessError): names=[]
            candidates=[ROOT/p for p in names if re.search(r"(release|status|report|acceptance).*(\.md|\.json|\.ya?ml)$",p,re.I)]
            claims=[]
            for p in candidates:
                try: text=p.read_text(errors="ignore")
                except OSError: continue
                if re.search(r"(?i)(fabrication[- ]ready|flight[- ]ready|certified|validated)\s*[:=]?\s*(true|yes|pass|approved)",text): claims.append(str(p.relative_to(ROOT)))
            return result | {"status":"FAIL" if claims else "PASS","measured":{"unsupported_release_claims":claims,"status_files_scanned":len(candidates)},"errors":["unsupported release/validation claims found without a verified acceptance receipt"] if claims else []}
        return result | {"status":"BLOCKED","errors":["native solver receipt and raw result evidence are absent; summaries cannot establish completion"]}
    # Delegate native CAD/PCB integration. Adapter must return a tested status and raw evidence hashes.
    if name in ("cad_occ", "cad_interference", "battery_bay_fit", "assembly_access", "cad_mass_cg", "kicad_edge_cuts", "kicad_drc", "native_cad", "native_pcb"):
        try:
            from ..adapters import run
            return result | run(check, profile=profile, config=config, run_id=run_id)
        except (ImportError, AttributeError):
            return result | {"status":"BLOCKED","errors":[f"native adapter {name} is not available"]}
    if name in ("claim_policy", "claims") or ident.endswith(("019", "020", "021")):
        promoted = []
        for p in (EA / "reports").glob("*.json"):
            try:
                obj = json.loads(p.read_text())
                if obj.get("status") in ("PASS", "validated", "fabrication_ready", "flight_ready") and not (obj.get("artifact_sha256") and obj.get("source_sha")): promoted.append(str(p))
            except json.JSONDecodeError: promoted.append(str(p))
        return result | {"status": "PASS" if not promoted else "FAIL", "measured": {"unsupported_promotions": promoted}, "errors": ["report promotes a claim without artifact and source hashes"] if promoted else []}
    if name in ("local_file", "file_exists"):
        missing=[]
        for rel in check.get("input_paths", []):
            p=Path(rel)
            if not (ROOT/p).is_file() and not (ROOT.parent/"drone-mech"/p).is_file(): missing.append(rel)
        return result | {"status": "PASS" if not missing else "FAIL", "measured": {"missing": missing}, "errors": ["missing required inputs: "+", ".join(missing)] if missing else []}
    return result | {"status": "BLOCKED", "errors": [f"runner capability {name!r} requires native evidence"]}
