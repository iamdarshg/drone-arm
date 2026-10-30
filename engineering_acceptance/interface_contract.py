"""Generate the pinned cross-repository system interface contract from aircraft_system.yaml."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
from typing import Any
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "engineering_acceptance" / "aircraft_system.yaml"
SCHEMA = ROOT / "engineering_acceptance" / "schema" / "interfaces.schema.json"
DEFAULT_OUTPUT = ROOT / "engineering_acceptance" / "reports" / "system_interface.json"

def _sha256(path: Path) -> str | None:
    if path.is_file():
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""): digest.update(block)
        return digest.hexdigest()
    if path.is_dir():
        # Directory artifacts use a deterministic tree digest over relative paths and file hashes.
        files=sorted(p for p in path.rglob("*") if p.is_file() and not p.is_symlink())
        if not files: return None
        tree=hashlib.sha256()
        for item in files:
            relative=item.relative_to(path).as_posix()
            tree.update(relative.encode("utf-8"));tree.update(b"\0");tree.update((_sha256(item) or "").encode("ascii"));tree.update(b"\n")
        return tree.hexdigest()
    return None

def build_contract(config: dict[str, Any] | None = None) -> dict[str, Any]:
    if config is None:
        config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    from .contracts import validate
    validate(config)
    root_refs = config["source_of_truth"]
    refs = {
        "drone-arm": {"repo": "drone-arm", "revision": root_refs["drone_arm"]["revision"]},
        "drone-mech": {"repo": "drone-mech", "revision": root_refs["drone_mech"]["revision"]},
    }
    artifacts=[]
    for name, source in config["source_artifacts"].items():
        repo_root = ROOT if source["repo"] == "drone-arm" else ROOT.parent / "drone-mech"
        digest = _sha256(repo_root / source["path"])
        artifacts.append({"id": name, "repo": source["repo"], "revision": source["revision"], "path": source["path"],
                          "sha256": digest, "status": "KNOWN" if digest else "BLOCKED",
                          "reason": None if digest else "configured source artifact is absent from the working checkout"})
    propulsion=config["propulsion"]; motors=config["motors"]; actuators=config["actuators"]
    main_source=config["source_artifacts"]["main_board"]
    mech_source=config["coordinates"]["source"]
    arm_arch=config["source_artifacts"]["architecture"]
    actuator_source=config["source_artifacts"]["actuator_interface"]
    return {
      "schema_version":1,
      "contract_id":"drone-arm-issue21-system-interface",
      "contract_status":"PARTIAL_BLOCKED",
      "sources":refs,
      "source_artifacts":artifacts,
      "coordinate_frame":{"frame_id":config["coordinates"]["frame_id"],"datum":config["coordinates"]["datum"],
        "axis_convention":config["coordinates"]["axis_convention"],"units":config["coordinates"]["units"],
        "origin":config["coordinates"]["origin"],"source":mech_source},
      "propulsion":{"architecture":propulsion["architecture"],"ducts":propulsion["ducts"],
        "stages_per_duct":propulsion["nominal_stages_per_duct"],"motor_count":motors["count"],
        "independent_power_stages":motors["independent_power_stages"],"motor_identity":{"value":None,"status":"BLOCKED","reason":"installed motor identity and source datasheet are unresolved"}},
      "power_and_control":{"esc_control":{"path":"CAN-FD","status":"KNOWN","source":arm_arch},
        "pwm_capacity":{"value":6,"unit":"1","status":"KNOWN","source":main_source},
        "bus_current_limit":{"value":None,"unit":"A","status":"BLOCKED","reason":"six independent fused inputs are required; no shared PCB trunk limit is established"}},
      "actuators":{"modeled_effector_count":actuators["modeled_independent_effectors"],
        "available_pwm_output_count":actuators["controller_outputs"],
        "effector_classes":{"value":None,"status":"BLOCKED","reason":"source configuration records seven independent effectors; stable per-effector IDs require cross-repository source reconciliation","source":actuator_source},
        "order":{"value":None,"status":"BLOCKED","reason":"actuator execution and allocation order is not established by the pinned firmware and airframe contract","source":actuator_source},
        "command_limits":{"value":None,"unit":"deg","status":"BLOCKED","reason":"per-effector command signs, ranges, rates, and failsafe positions require sourced interface definitions","source":actuator_source},
        "feedback_interfaces":{"value":None,"status":"BLOCKED","reason":"position feedback sensors and telemetry IDs are not evidenced","source":main_source},
        "fault_behavior":{"value":None,"status":"BLOCKED","reason":"fault response and safe-state transitions are not verified in executable firmware","source":main_source},
        "firmware_capabilities":{"value":None,"status":"BLOCKED","reason":"production firmware source and built target image are absent or not verified","source":main_source}},
      "interfaces":[
        {"id":"propulsion.esc_control","producer":"flight_controller","consumer":"six_esc_power_stages","signal":"motor_command_and_telemetry","protocol":"CAN-FD","status":"KNOWN","source":arm_arch},
        {"id":"actuators.pwm_capacity","producer":"flight_controller","consumer":"servo_outputs","signal":"independent_pwm_command_paths","protocol":"PWM","capacity":6,"unit":"1","status":"KNOWN","source":main_source},
        {"id":"actuators.mapping","producer":"flight_controller","consumer":"seven_independent_effectors","signal":"effector_command_and_feedback_assignment","protocol":None,"capacity":None,"unit":None,"status":"BLOCKED","reason":"seven modeled effectors exceed six PWM outputs; a unique realizable path and feedback assignment is unresolved","source":actuator_source}
      ],
      "limitations":["This contract records source-backed interfaces and unresolved gates; it does not claim the actuator mapping or hardware interface is solved."]
    }

def validate_contract(contract: dict[str, Any]) -> None:
    import jsonschema
    schema=json.loads(SCHEMA.read_text(encoding="utf-8"))
    errors=list(jsonschema.Draft202012Validator(schema,format_checker=jsonschema.FormatChecker()).iter_errors(contract))
    if errors:
        raise ValueError("system interface contract invalid: " + "; ".join(f"{list(e.absolute_path)} {e.message}" for e in errors))
    if contract.get("propulsion", {}).get("motor_count") != 6 or contract.get("propulsion", {}).get("independent_power_stages") != 6:
        raise ValueError("system interface must preserve six separately controlled motor power stages")
    if contract.get("power_and_control", {}).get("esc_control", {}).get("path") != "CAN-FD":
        raise ValueError("ESC control interface must remain CAN-FD")
    actuators=contract.get("actuators", {})
    mapping=next((row for row in contract.get("interfaces", []) if row.get("id") == "actuators.mapping"), {})
    if actuators.get("modeled_effector_count") == 7 and actuators.get("available_pwm_output_count") == 6 and mapping.get("status") != "BLOCKED":
        raise ValueError("seven effectors cannot be presented as mapped onto six PWM outputs")
    for item in contract.get("source_artifacts", []):
        digest=item.get("sha256")
        if item.get("status") == "KNOWN" and not isinstance(digest,str):
            raise ValueError(f"source artifact {item.get('id')} is marked KNOWN without a SHA-256")
        if digest is not None and (len(digest)!=64 or any(c not in "0123456789abcdef" for c in digest)):
            raise ValueError(f"source artifact {item.get('id')} has an invalid SHA-256")

def generate(output: Path = DEFAULT_OUTPUT, config_path: Path = CONFIG) -> Path:
    config=yaml.safe_load(config_path.read_text(encoding="utf-8"))
    contract=build_contract(config); validate_contract(contract)
    output.parent.mkdir(parents=True,exist_ok=True)
    payload=json.dumps(contract,sort_keys=True,indent=2,allow_nan=False)+"\n"
    output.write_text(payload,encoding="utf-8")
    return output

def main(argv: list[str] | None = None) -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument("--config",type=Path,default=CONFIG)
    args=parser.parse_args(argv)
    print(generate(args.output,args.config))
    return 0

if __name__ == "__main__": raise SystemExit(main())
