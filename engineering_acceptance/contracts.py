"""Strict, deterministic validation for engineering_acceptance/aircraft_system.yaml."""
from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent
SCHEMA_PATH = ROOT / "schema" / "aircraft_system.schema.json"
SHA_RE = re.compile(r"^[0-9a-f]{40}$")
# Immutable source revisions from the issue-21 acceptance baseline. Change only with a reviewed contract revision.
KNOWN_REVISIONS = {"drone_arm": {"744994a096ca73e872b396a9b0636c58e7c1afe8"}, "drone_mech": {"5246c90657071772e220b118967fa5f3e7c9ff7c"}}

class ContractError(ValueError):
    """Input violates the versioned engineering contract."""
    def __init__(self, errors: list[str]):
        self.errors = tuple(errors)
        super().__init__("Engineering contract violations:\n" + "\n".join(f" - {e}" for e in errors))

def load_config(path: str | Path = ROOT / "aircraft_system.yaml") -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    validate(value)
    return value

def validate(config: Any) -> None:
    errors: list[str] = []
    if not isinstance(config, dict):
        raise ContractError(["$: expected object"])
    try:
        from jsonschema import Draft202012Validator
        schema = json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))
        errors.extend(f"{_path(e.absolute_path)}: {e.message}" for e in Draft202012Validator(schema).iter_errors(config))
    except ImportError:
        errors.extend(_validate_shape(config))
    _finite_numbers(config, "$", errors)
    if set(config) - _allowed_top():
        errors.append("$: unknown properties are forbidden")
    refs = {v.get("revision") for v in config.get("source_of_truth", {}).values() if isinstance(v, dict)}
    for name, source in config.get("source_of_truth", {}).items() if isinstance(config.get("source_of_truth"), dict) else ():
        if isinstance(source, dict):
            revision = str(source.get("revision", ""))
            if not SHA_RE.fullmatch(revision):
                errors.append(f"$.source_of_truth.{name}.revision: must be a full 40-character lowercase git SHA")
            elif revision not in KNOWN_REVISIONS.get(name, set()):
                errors.append(f"$.source_of_truth.{name}.revision: revision is not registered in the pinned contract")
    for i, item in enumerate(config.get("overrides", []) if isinstance(config.get("overrides"), list) else []):
        rev = item.get("source", {}).get("revision") if isinstance(item, dict) and isinstance(item.get("source"), dict) else None
        if rev not in refs:
            errors.append(f"$.overrides[{i}].source.revision: revision must match a declared source_of_truth revision")
    propulsion = config.get("propulsion")
    if isinstance(propulsion, dict) and (type(propulsion.get("ducts")) is not int or propulsion.get("ducts") != 2):
        errors.append("$.propulsion.ducts: expected integer 2")
    if isinstance(propulsion, dict) and (type(propulsion.get("nominal_stages_per_duct")) is not int or propulsion.get("nominal_stages_per_duct") != 3):
        errors.append("$.propulsion.nominal_stages_per_duct: expected integer 3")
    for path, item in _walk(config):
        if isinstance(item, dict) and "value" in item and item["value"] is None:
            if item.get("status") != "BLOCKED" or not isinstance(item.get("reason"), str) or not item["reason"].strip():
                errors.append(f"{path}: null values require status BLOCKED and an explicit reason")
        if isinstance(item, dict) and "unit" in item and item["unit"] is not None:
            if item["unit"] not in KNOWN_UNITS:
                errors.append(f"{path}.unit: unrecognized physical unit {item['unit']!r}")
    if errors:
        raise ContractError(sorted(set(errors)))

def _allowed_top() -> set[str]:
    return {"schema_version", "source_of_truth", "propulsion", "battery", "motors", "boards", "actuators", "payload", "limits", "cost_policy", "overrides"}
def _path(parts: Any) -> str:
    out="$"
    for p in parts: out += f"[{p}]" if isinstance(p,int) else f".{p}"
    return out
def _finite_numbers(obj: Any, path: str, errors: list[str]) -> None:
    if isinstance(obj, bool) or obj is None: return
    if isinstance(obj, (int,float)):
        if not math.isfinite(obj): errors.append(f"{path}: numeric values must be finite")
    elif isinstance(obj, dict):
        for k,v in obj.items(): _finite_numbers(v, f"{path}.{k}", errors)
    elif isinstance(obj,list):
        for i,v in enumerate(obj): _finite_numbers(v, f"{path}[{i}]", errors)
def _walk(obj: Any, path="$"):
    yield path,obj
    if isinstance(obj,dict):
        for k,v in obj.items(): yield from _walk(v,f"{path}.{k}")
    elif isinstance(obj,list):
        for i,v in enumerate(obj): yield from _walk(v,f"{path}[{i}]")

def _validate_shape(config: dict[str, Any]) -> list[str]:
    errors=[]
    required={"schema_version","source_of_truth","propulsion","battery","motors","boards","actuators","payload","limits","cost_policy","overrides"}
    for k in sorted(required-set(config)): errors.append(f"$.{k}: required property missing")
    for group, fields in {"source_of_truth":{"drone_arm","drone_mech"},"propulsion":{"architecture","ducts","nominal_stages_per_duct"},"battery":{"series_cells","nominal_voltage_V","maximum_voltage_V","capacity_Ah","dc_resistance_ohm"},"motors":{"count","independent_power_stages","installed_can_length_mm","temperature_measurements"},"boards":{"active_variant","dimensions_mm","main_board_source","esc_board_candidates"},"actuators":{"modeled_independent_effectors","controller_outputs","source"},"payload":{"modeled_payload_mass_kg","cg_m"},"limits":{"per_motor_battery_current_A","bus_current_A","temperature_C"},"cost_policy":{"remote_compute_budget_usd","allow_remote_compute"}}.items():
        if isinstance(config.get(group),dict):
            for field in sorted(fields-set(config[group])): errors.append(f"$.{group}.{field}: required property missing")
    return errors

KNOWN_UNITS={"1","mm","m","kg","g","s","ms","A","mA","V","W","Wh","Ah","ohm","mohm","C","K","deg","rad","N","Nm","Pa","Hz","rpm","m/s","m2","m3","kg/m3","N/m2","USD"}
