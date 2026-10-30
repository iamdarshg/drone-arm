"""Strict, deterministic validation for engineering_acceptance/aircraft_system.yaml."""
from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

from .units import UNITS

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
    for path, item in _walk(config):
        if isinstance(item, dict) and {"repo", "revision", "path"}.issubset(item):
            repo_name = "drone_arm" if item.get("repo") == "drone-arm" else "drone_mech" if item.get("repo") == "drone-mech" else None
            if not SHA_RE.fullmatch(str(item.get("revision", ""))):
                errors.append(f"{path}.revision: source reference needs a full git SHA")
            elif repo_name and item["revision"] not in KNOWN_REVISIONS[repo_name]:
                errors.append(f"{path}.revision: source revision is not registered in the pinned contract")
    # Quantities whose source has not been established remain blocked; negative current or
    # thermal limits are never meaningful engineering inputs.
    for container, field_names in ((config.get("battery", {}), ("capacity_Ah", "dc_resistance_ohm")),
                                   (config.get("motors", {}), ("installed_can_length_mm", "temperature_measurements")),
                                   (config.get("limits", {}), ("per_motor_battery_current_A", "bus_current_A", "temperature_C"))):
        if not isinstance(container, dict):
            continue
        for field in field_names:
            item = container.get(field)
            if not isinstance(item, dict) or "value" not in item:
                continue
            value = item.get("value")
            if value is None and item.get("status") != "BLOCKED":
                errors.append(f"$.{field}: unknown battery/thermal quantity must remain BLOCKED")
            if value is not None and isinstance(value, (int, float)) and not isinstance(value, bool):
                if field in ("per_motor_battery_current_A", "bus_current_A") and value < 0:
                    errors.append(f"$.limits.{field}: current cannot be negative")
                if field in ("capacity_Ah", "dc_resistance_ohm", "installed_can_length_mm") and value <= 0:
                    errors.append(f"$.{field}: known physical quantity must be positive")
                expected_unit = "A" if field in ("per_motor_battery_current_A", "bus_current_A") else "Ah" if field == "capacity_Ah" else "ohm" if field == "dc_resistance_ohm" else "mm" if field == "installed_can_length_mm" else None
                if expected_unit and item.get("unit") is not None and item["unit"] != expected_unit:
                    errors.append(f"$.{field}.unit: expected {expected_unit} for field {field}")
    current_limits = config.get("limits", {})
    if isinstance(current_limits, dict):
        per_motor = current_limits.get("per_motor_battery_current_A", {})
        bus = current_limits.get("bus_current_A", {})
        if isinstance(per_motor, dict) and isinstance(bus, dict) and isinstance(per_motor.get("value"), (int, float)) and isinstance(bus.get("value"), (int, float)) and per_motor.get("value") > 0 and bus.get("value") == 6 * per_motor.get("value"):
            errors.append("$.limits.bus_current_A: cannot promote six per-motor limits into an unsupported shared PCB trunk limit")
    propulsion = config.get("propulsion")
    if isinstance(propulsion, dict) and (type(propulsion.get("ducts")) is not int or propulsion.get("ducts") != 2):
        errors.append("$.propulsion.ducts: expected integer 2")
    if isinstance(propulsion, dict) and (type(propulsion.get("nominal_stages_per_duct")) is not int or propulsion.get("nominal_stages_per_duct") != 3):
        errors.append("$.propulsion.nominal_stages_per_duct: expected integer 3")
    expected_field_units = {"capacity_Ah": "Ah", "energy_Wh": "Wh", "energy_J": "J", "dc_resistance_ohm": "ohm", "nominal_voltage_V": "V", "maximum_voltage_V": "V", "installed_can_length_mm": "mm", "dimensions_mm": "mm", "thickness_mm": "mm", "per_motor_battery_current_A": "A", "bus_current_A": "A", "temperature_C": "C", "temperature_measurements": "C"}
    for path, item in _walk(config):
        if isinstance(item, dict) and "value" in item and "unit" in item:
            name = path.rsplit(".", 1)[-1].split("[")[0]
            expected_unit = expected_field_units.get(name)
            if expected_unit and item.get("unit") != expected_unit:
                errors.append(f"{path}.unit: expected {expected_unit} for {name}")
        if isinstance(item, dict) and "value" in item and item["value"] is None:
            if not ((item.get("status") == "BLOCKED" and isinstance(item.get("reason"), str) and item["reason"].strip()) or (item.get("provenance") == "unknown" and isinstance(item.get("unit"), str) and item.get("evidence_id") is None)):
                errors.append(f"{path}: null values require status BLOCKED and an explicit reason")
        if isinstance(item, dict) and "unit" in item and item["unit"] is not None:
            if item["unit"] not in KNOWN_UNITS:
                errors.append(f"{path}.unit: unrecognized physical unit {item['unit']!r}")
    if errors:
        raise ContractError(sorted(set(errors)))

def _allowed_top() -> set[str]:
    return {"schema_version", "source_of_truth", "propulsion", "battery", "motors", "boards", "actuators", "payload", "limits", "cost_policy", "overrides", "coordinates", "source_artifacts", "cad", "materials", "battery_installation", "simulation"}
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
    required={"schema_version","source_of_truth","propulsion","battery","motors","boards","actuators","payload","limits","cost_policy","overrides", "coordinates", "source_artifacts", "cad", "materials", "battery_installation", "simulation"}
    for k in sorted(required-set(config)): errors.append(f"$.{k}: required property missing")
    for group, fields in {"source_of_truth":{"drone_arm","drone_mech"},"propulsion":{"architecture","ducts","nominal_stages_per_duct"},"battery":{"series_cells","nominal_voltage_V","maximum_voltage_V","capacity_Ah","dc_resistance_ohm"},"motors":{"count","independent_power_stages","installed_can_length_mm","temperature_measurements"},"boards":{"active_variant","dimensions_mm","main_board_source","esc_board_candidates"},"actuators":{"modeled_independent_effectors","controller_outputs","source"},"payload":{"modeled_payload_mass_kg","cg_m"},"limits":{"per_motor_battery_current_A","bus_current_A","temperature_C"},"cost_policy":{"remote_compute_budget_usd","allow_remote_compute"}}.items():
        if isinstance(config.get(group),dict):
            for field in sorted(fields-set(config[group])): errors.append(f"$.{group}.{field}: required property missing")
    return errors

KNOWN_UNITS=set(UNITS)

RESULT_SCHEMA_PATH = ROOT / "schema" / "testresult.schema.json"
MANIFEST_SCHEMA_PATH = ROOT / "schema" / "run_manifest.schema.json"

def validate_result(row: Any) -> None:
    """Validate one actual runner result before it is appended to the ledger."""
    _validate_json_schema(row, RESULT_SCHEMA_PATH, "result")
    errors: list[str] = []
    if not isinstance(row, dict):
        raise ContractError(["result: expected object"])
    if row.get("status") not in {"PASS", "FAIL", "BLOCKED", "NOT_RUN", "NOT_APPLICABLE"}:
        errors.append("result.status: invalid evidence state")
    for field in ("run_id", "drone_arm_sha", "drone_mech_sha", "test_definition_sha256"):
        value = row.get(field)
        length = 64 if field in ("run_id", "test_definition_sha256", "config_sha256") else 40
        if not isinstance(value, str) or not re.fullmatch(rf"[0-9a-f]{{{length}}}", value):
            errors.append(f"result.{field}: expected {length}-character lowercase hexadecimal digest")
    if row.get("config_present") is True and not re.fullmatch(r"[0-9a-f]{64}", str(row.get("config_sha256", ""))):
        errors.append("result.config_sha256: present config requires a SHA-256")
    if row.get("config_present") is False and (row.get("status") != "FAIL" or row.get("config_sha256") is not None):
        errors.append("result: missing config is represented only by config_present=false, null digest, and FAIL")
    source = row.get("source")
    if not isinstance(source, dict) or not SHA_RE.fullmatch(str(source.get("revision", ""))):
        errors.append("result.source.revision: full source revision is required")
    if row.get("unit") is not None and row["unit"] not in UNITS:
        errors.append(f"result.unit: unsupported unit {row['unit']!r}")
    if isinstance(row.get("measured"), (int, float)) and not isinstance(row.get("measured"), bool) and row.get("unit") is None:
        errors.append("result.unit: scalar measured quantities require an explicit unit")
    artifacts = row.get("artifacts")
    if not isinstance(artifacts, list): artifacts = []
    artifact_hashes = []
    for i, item in enumerate(artifacts):
        if not isinstance(item, dict) or not isinstance(item.get("path"), str) or not re.fullmatch(r"(?:[0-9a-f]{40}|[0-9a-f]{64})", str(item.get("sha256", ""))):
            errors.append(f"result.artifacts[{i}]: each evidence artifact needs a path and SHA-256")
        else: artifact_hashes.append(item["sha256"])
    expected_set_hash = __import__("hashlib").sha256("".join(artifact_hashes).encode()).hexdigest() if artifact_hashes else None
    if row.get("artifact_sha256") != expected_set_hash:
        errors.append("result.artifact_sha256: does not match ordered artifact SHA-256 receipt set")
    if row.get("status") == "PASS" and row.get("evidence_required") and not artifacts:
        errors.append("result: PASS requires at least one raw artifact receipt when evidence_required is true")
    if errors:
        raise ContractError(errors)

def validate_manifest(manifest: Any) -> None:
    _validate_json_schema(manifest, MANIFEST_SCHEMA_PATH, "run manifest")
    if not isinstance(manifest, dict):
        raise ContractError(["run manifest: expected object"])

def _validate_json_schema(value: Any, schema_path: Path, label: str) -> None:
    try:
        import jsonschema
    except ImportError as exc:
        raise ContractError([f"{label}: jsonschema is required for contract validation"]) from exc
    try:
        schema = json.loads(schema_path.read_text(encoding="utf-8"))
        errors = sorted(jsonschema.Draft202012Validator(schema, format_checker=jsonschema.FormatChecker()).iter_errors(value), key=lambda e: list(map(str,e.absolute_path)))
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractError([f"{label}: invalid installed schema: {exc}"]) from exc
    if errors:
        raise ContractError([f"{label}{_path(e.absolute_path)[1:]}: {e.message}" for e in errors])

def validate_component(component: Any) -> None:
    """Validate a typed component record, including unit and evidence semantics."""
    path = ROOT / "schema" / "component.schema.json"
    _validate_json_schema(component, path, "component")
    errors: list[str] = []
    if not isinstance(component, dict):
        raise ContractError(["component: expected object"])
    for name, quantity in component.get("properties", {}).items():
        if not isinstance(quantity, dict):
            continue
        value = quantity.get("value")
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if not math.isfinite(value):
                errors.append(f"component.properties.{name}.value: numeric values must be finite")
            if re.search(r"(?i)(stiffness|modulus|density|mass|length|diameter|voltage|capacity|resistance)", name) and value <= 0:
                errors.append(f"component.properties.{name}.value: sourced physical property must be positive")
        provenance = quantity.get("provenance")
        if provenance in ("measured", "sourced") and not quantity.get("evidence_id"):
            errors.append(f"component.properties.{name}.evidence_id: measured/sourced values require evidence")
        if provenance == "unknown" and (value is not None or quantity.get("evidence_id") is not None):
            errors.append(f"component.properties.{name}: unknown provenance requires null value and no evidence id")
    if errors:
        raise ContractError(errors)
