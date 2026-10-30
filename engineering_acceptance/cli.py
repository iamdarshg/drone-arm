"""Command line interface for evidence-gated acceptance."""
import argparse, html, json, sys
from pathlib import Path

from .core import CATALOG, EA, LEDGER, CONFIG_PATH, discover, latest_run, run, validate_inputs, verify_ledger


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="python -m engineering_acceptance")
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("discover")
    vi = sub.add_parser("validate-inputs"); vi.add_argument("--config", type=Path, default=CONFIG_PATH)
    r = sub.add_parser("run"); r.add_argument("--profile", required=True); r.add_argument("--config", type=Path, default=CONFIG_PATH)
    rp = sub.add_parser("report"); rp.add_argument("--json", dest="json_path"); rp.add_argument("--junit"); rp.add_argument("--html")
    sub.add_parser("verify-evidence")
    gm=sub.add_parser("generate-matrix"); gm.add_argument("--input", required=True, type=Path); gm.add_argument("--output", required=True, type=Path); gm.add_argument("--seed", type=int, default=0)
    ex = sub.add_parser("explain"); ex.add_argument("check_id")
    df = sub.add_parser("diff"); df.add_argument("baseline"); df.add_argument("current")
    args = p.parse_args(argv)
    try:
        if args.command == "discover":
            d = discover(); print(json.dumps({"requirements": d["requirements"], "checks": d["checks"], "profiles": d["profiles"]}, indent=2)); return 0
        if args.command == "validate-inputs":
            errors = validate_inputs(args.config)
            for e in errors: print(f"FAIL: {e}")
            if not errors: print("PASS: catalog and aircraft inputs are structurally valid")
            return 1 if errors else 0
        if args.command == "run":
            result = run(args.profile, args.config)
            print(json.dumps({k: v for k, v in result.items() if k != "results"}, indent=2))
            for row in sorted(result["results"], key=lambda x: (x.get("severity") not in ("P0", "critical"), x["status"] != "FAIL", x["id"])): print(f"{row['status']:18} {row['id']}: {'; '.join(row.get('errors', []))}")
            selected_ids={r["id"] for r in result["results"]}
            blocking_ids={c["id"] for c in discover()["checks"] if c.get("blocking") and c.get("id") in selected_ids}
            blocked_required=any(r["id"] in blocking_ids and r["status"] in ("FAIL","BLOCKED","NOT_RUN") for r in result["results"])
            return 1 if blocked_required else 0
        if args.command == "verify-evidence":
            errors = verify_ledger()
            for e in errors: print(f"FAIL: {e}")
            if not errors: print("PASS: ledger records are structurally valid")
            return 1 if errors else 0
        if args.command == "generate-matrix":
            import yaml
            from .scenarios import generate_pairwise
            spec=yaml.safe_load(args.input.read_text(encoding="utf-8"))
            matrix=generate_pairwise(spec["axes"], seed=args.seed, applicability=spec.get("applicability"))
            args.output.parent.mkdir(parents=True,exist_ok=True)
            args.output.write_text(json.dumps(matrix,indent=2,sort_keys=True)+"\n",encoding="utf-8")
            print(json.dumps({k:v for k,v in matrix.items() if k!="cases"},indent=2))
            return 0
        if args.command == "explain":
            for c in discover()["checks"]:
                if c.get("id") == args.check_id: print(json.dumps(c, indent=2)); return 0
            print(f"unknown check {args.check_id}", file=sys.stderr); return 2
        if args.command == "diff":
            old, new = json.loads(Path(args.baseline).read_text()), json.loads(Path(args.current).read_text())
            old_rows = {x["id"]: x for x in old.get("results", [])}; new_rows = {x["id"]: x for x in new.get("results", [])}
            changes = [{"id": k, "from": old_rows.get(k, {}).get("status"), "to": new_rows.get(k, {}).get("status")} for k in sorted(old_rows.keys() | new_rows.keys()) if old_rows.get(k, {}).get("status") != new_rows.get(k, {}).get("status")]
            print(json.dumps({"changes": changes}, indent=2)); return 0
        if args.command == "report":
            rows = latest_run()
            if not rows: print("no run found", file=sys.stderr); return 2
            states = ("PASS", "FAIL", "BLOCKED", "NOT_RUN", "NOT_APPLICABLE")
            counts = {s: sum(r["status"] == s for r in rows) for s in states}
            reqs=discover()["requirements"]; reqs=reqs.get("requirements",[]) if isinstance(reqs,dict) else reqs
            report = {"run_id": rows[-1]["run_id"], "drone_arm_sha": rows[-1].get("drone_arm_sha"), "drone_mech_sha": rows[-1].get("drone_mech_sha"), "profile": rows[-1].get("profile"), "requirements_declared": len(reqs), "checks_discovered": len(discover()["checks"]), "checks_attempted": len(rows), "counts": counts, "results": rows, "notice": "Not a flight or certification approval."}
            blob = json.dumps(report, indent=2)
            if args.json_path: Path(args.json_path).write_text(blob + "\n")
            if args.junit:
                failed = counts["FAIL"]; blocked = counts["BLOCKED"] + counts["NOT_RUN"]
                tests = [f'<testcase classname="acceptance" name="{html.escape(x["id"])}">' + (f'<failure message="FAIL">{html.escape("; ".join(x.get("errors", [])))}</failure>' if x["status"] == "FAIL" else '<skipped message="' + x["status"] + '"/>' if x["status"] in ("BLOCKED", "NOT_RUN", "NOT_APPLICABLE") else '') + '</testcase>' for x in rows]
                Path(args.junit).write_text(f'<testsuite tests="{len(rows)}" failures="{failed}" skipped="{blocked}">' + ''.join(tests) + '</testsuite>\n')
            if args.html:
                ordered=sorted(rows, key=lambda x: (x.get("severity") not in ("P0", "critical"), x["status"] != "FAIL", x["id"]))
                def cell(v): return html.escape(json.dumps(v, sort_keys=True) if isinstance(v,(dict,list)) else str(v))
                table = ''.join('<tr>'+''.join(f'<td>{cell(v)}</td>' for v in (x.get("id"),x.get("system"),x.get("variant"),x.get("severity"),x.get("applicability"),x.get("measured"),x.get("unit"),x.get("required"),x.get("source"),x.get("runner"),x.get("exact_command"),x.get("status"),x.get("artifact_paths"),x.get("artifact_sha256")))+'</tr>' for x in ordered)
                headings=''.join(f'<th>{h}</th>' for h in ('ID','System','Variant','Severity','Applicability','Measured','Unit','Required','Source','Runner','Exact command','Status','Artifact path','Artifact SHA-256'))
                attempted=report["checks_attempted"]
                Path(args.html).write_text('<!doctype html><meta charset="utf-8"><title>Acceptance report</title><style>body{font:14px system-ui;margin:2rem}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:.35rem;vertical-align:top}th{position:sticky;top:0;background:#eee}</style><h1>Engineering acceptance scorecard</h1><p>Run ID: '+report["run_id"]+'</p><p>Drone mech HEAD: '+str(report["drone_mech_sha"])+'; drone arm HEAD: '+str(report["drone_arm_sha"])+'</p><p>Profile: '+html.escape(str(report["profile"]))+'; requirements declared: '+str(report["requirements_declared"])+'; checks discovered: '+str(report["checks_discovered"])+'; checks attempted: '+str(attempted)+'</p><p>'+html.escape(json.dumps(counts))+'</p><p>Release-critical gates PASS: '+str(sum(x["status"]=="PASS" and x.get("severity")=="P0" and x.get("blocking") for x in rows))+' of '+str(sum(x.get("severity")=="P0" and x.get("blocking") for x in rows))+'</p><p><strong>Not a flight or certification approval.</strong></p><table><thead><tr>'+headings+'</tr></thead><tbody>'+table+'</tbody></table>')
            if not any((args.json_path, args.junit, args.html)): print(blob)
            return 0
    except (ValueError, OSError, KeyError) as e:
        print(f"ERROR: {e}", file=sys.stderr); return 2
    return 2

if __name__ == "__main__": raise SystemExit(main())
