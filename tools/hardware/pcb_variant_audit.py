#!/usr/bin/env /usr/bin/python3
"""Source-pinned native audit of Rev-B PCB variants.

Run this from an exact drone-arm source checkout with KiCad 9 pcbnew and
kicad-cli installed. The current-main source is a comparison reference only;
this tool does not designate any source as a fabrication release.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

DEFAULT_MANIFEST = Path(__file__).resolve().parents[2] / "docs/hardware/pcb-variant-audit-sources.json"
MAIN_BOARD = "hardware/main/rev_b/main_rev_b.kicad_pcb"
ESC_BOARD = "hardware/esc/rev_b/esc_rev_b.kicad_pcb"
MAIN_SCHEMATIC = "hardware/main/rev_b/main_rev_b.kicad_sch"
ESC_SCHEMATIC = "hardware/esc/rev_b/esc_rev_b.kicad_sch"
MAIN_PINMAP = "hardware/main/rev_b/mcu_pinmap.json"
CAD_PLACEHOLDERS_MM = {"main": [140.0, 100.0], "esc": [150.0, 150.0]}
EXPECTED_EFFECTORS = {
    "independent_count": 7,
    "source": "drone-mech#3 and drone-arm#20; Phase-15 set is 2 elevons + 4 exit vanes + 1 rudder",
    "composition": {"elevons": 2, "edf_exit_vector_vanes": 4, "rudder": 1},
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def check_source_revision(expected: str, observed: str) -> dict[str, Any]:
    """Reject a stale/mis-selected checkout instead of silently auditing it."""
    ok = bool(expected) and expected == observed
    return {"expected": expected, "observed": observed,
            "status": "PASS" if ok else "FAIL",
            "error": None if ok else "source checkout HEAD differs from pinned variant SHA"}


def compare_envelope(actual_mm: list[float], reference_mm: list[float], tolerance_mm: float = 0.05) -> dict[str, Any]:
    if len(actual_mm) != 2 or len(reference_mm) != 2:
        raise ValueError("board envelope must have width and height")
    delta = [float(actual_mm[i]) - float(reference_mm[i]) for i in range(2)]
    return {"actual_mm": [float(x) for x in actual_mm],
            "reference_mm": [float(x) for x in reference_mm], "difference_mm": delta,
            "tolerance_mm": tolerance_mm,
            "status": "PASS" if all(abs(x) <= tolerance_mm for x in delta) else "FAIL"}


def reconcile_actuators(expected_count: int, pwm_channels: list[dict[str, Any]]) -> dict[str, Any]:
    actual_nets = [str(item.get("net", "")) for item in pwm_channels]
    unique_nets = sorted(set(actual_nets))
    capacity = len(unique_nets)
    return {"expected_independent_effectors": expected_count,
            "available_pwm_channel_count": capacity,
            "pwm_channels": pwm_channels,
            "unmapped_effector_count": max(0, expected_count - capacity),
            "unassigned_pwm_channel_count": max(0, capacity - expected_count),
            "phantom_gpio_created": False,
            "status": "PASS" if expected_count == capacity else "FAIL",
            "reason": ("independent effector count equals distinct PCB-mapped PWM nets" if expected_count == capacity
                       else "source maps fewer PWM outputs than independent effectors; no output is invented")}


def _git_head(root: Path) -> str:
    proc = subprocess.run(["git", "-C", str(root), "rev-parse", "HEAD"],
                          text=True, capture_output=True, check=True)
    return proc.stdout.strip()


def _pcbnew_module():
    try:
        import pcbnew  # type: ignore
    except ImportError as exc:
        raise RuntimeError("BLOCKED: import pcbnew with /usr/bin/python3 and KiCad 9.0.2") from exc
    return pcbnew


def _to_mm(pcbnew, value: int | float) -> float:
    return round(float(pcbnew.ToMM(value)), 6)


def _bbox_mm(pcbnew, item) -> list[float]:
    box = item.GetBoundingBox()
    return [_to_mm(pcbnew, box.GetX()), _to_mm(pcbnew, box.GetY()),
            _to_mm(pcbnew, box.GetRight()), _to_mm(pcbnew, box.GetBottom())]


def _model_paths(pcbnew, footprint) -> tuple[list[dict[str, Any]], bool]:
    models: list[dict[str, Any]] = []
    all_resolved = True
    model_env = {
        "KICAD9_3DMODEL_DIR": os.environ.get("KICAD9_3DMODEL_DIR", "/usr/share/kicad/3dmodels"),
        "KICAD_3DMODEL_DIR": os.environ.get("KICAD_3DMODEL_DIR", "/usr/share/kicad/3dmodels"),
    }
    for model in footprint.Models():
        filename = str(model.m_Filename)
        resolved = filename
        for key, value in model_env.items():
            resolved = resolved.replace("${" + key + "}", value)
        path = Path(resolved).expanduser()
        exists = path.is_file()
        all_resolved &= exists
        models.append({"reference": filename, "resolved_path": str(path), "file_exists": exists,
                       "offset_mm": [_to_mm(pcbnew, model.m_Offset.x), _to_mm(pcbnew, model.m_Offset.y),
                                     _to_mm(pcbnew, model.m_Offset.z)],
                       "scale": [float(model.m_Scale.x), float(model.m_Scale.y), float(model.m_Scale.z)],
                       "rotation_deg": [float(model.m_Rotation.x), float(model.m_Rotation.y),
                                        float(model.m_Rotation.z)]})
    return models, all_resolved


def _footprint_record(pcbnew, footprint) -> dict[str, Any]:
    ref = footprint.GetReference()
    value = footprint.GetValue()
    fpid = str(footprint.GetFPID().GetLibItemName())
    box = _bbox_mm(pcbnew, footprint)
    models, resolved = _model_paths(pcbnew, footprint)
    pads = list(footprint.Pads())
    return {"reference": ref, "value": value, "footprint_id": fpid,
            "position_mm": [_to_mm(pcbnew, footprint.GetPosition().x), _to_mm(pcbnew, footprint.GetPosition().y)],
            "orientation_deg": float(footprint.GetOrientationDegrees()),
            "board_graphic_bbox_mm": box,
            "pad_count": len(pads), "model_refs": models,
            "native_3d_height": {"status": "BLOCKED", "value_mm": None,
                                 "reason": "model geometry bounds are not imported; missing referenced models also block height" if not resolved
                                           else "referenced model exists but this exporter does not derive a solid height"},
            "connector_candidate": bool(re.search(r"(^J\d|^P\d|connector|terminal|usb|coax|ufl|sma)",
                                                     str(ref) + " " + str(value) + " " + str(fpid), re.IGNORECASE))}


def _pinmap(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"status": "BLOCKED", "source": str(path), "channels": [], "count": None,
                "reason": "pinned MCU pin map is absent"}
    raw = json.loads(path.read_text())
    channels = []
    for gpio, entry in raw.get("pins", {}).items():
        net = str(entry.get("net", ""))
        match = re.search(r"(?:^|/)PWM_ESC(\d+)$", net)
        if match:
            channels.append({"channel": int(match.group(1)), "gpio": str(gpio),
                             "pad": str(entry.get("pad", "")),
                             "pinfunction": str(entry.get("pinfunction", "")), "net": net})
    channels.sort(key=lambda x: x["channel"])
    distinct = len({x["net"] for x in channels})
    return {"status": "PASS" if distinct == len(channels) else "FAIL", "source": str(path),
            "source_sha256": sha256_file(path), "channels": channels, "count": distinct}


def extract_board(board_path: Path, board_name: str, pinmap_path: Path | None = None) -> dict[str, Any]:
    pcbnew = _pcbnew_module()
    board = pcbnew.LoadBoard(str(board_path))
    edge_items = [drawing for drawing in board.GetDrawings() if drawing.GetLayer() == pcbnew.Edge_Cuts]
    if not edge_items:
        raise ValueError(f"no Edge.Cuts primitives in {board_path}")
    # Bounding boxes include arc extrema, unlike a min/max of arc endpoints.
    edge_boxes = [_bbox_mm(pcbnew, item) for item in edge_items]
    outline_bbox = [min(b[0] for b in edge_boxes), min(b[1] for b in edge_boxes),
                    max(b[2] for b in edge_boxes), max(b[3] for b in edge_boxes)]
    width = outline_bbox[2] - outline_bbox[0]
    height = outline_bbox[3] - outline_bbox[1]
    edge_records = []
    for item in edge_items:
        shape = item.GetShape()
        record: dict[str, Any] = {"shape": str(shape), "bbox_mm": _bbox_mm(pcbnew, item)}
        if shape == pcbnew.SHAPE_T_SEGMENT:
            record.update(start_mm=[_to_mm(pcbnew, item.GetStart().x), _to_mm(pcbnew, item.GetStart().y)],
                          end_mm=[_to_mm(pcbnew, item.GetEnd().x), _to_mm(pcbnew, item.GetEnd().y)])
        elif shape == pcbnew.SHAPE_T_ARC:
            record.update(start_mm=[_to_mm(pcbnew, item.GetStart().x), _to_mm(pcbnew, item.GetStart().y)],
                          end_mm=[_to_mm(pcbnew, item.GetEnd().x), _to_mm(pcbnew, item.GetEnd().y)],
                          center_mm=[_to_mm(pcbnew, item.GetCenter().x), _to_mm(pcbnew, item.GetCenter().y)],
                          radius_mm=_to_mm(pcbnew, item.GetRadius()))
        elif shape == pcbnew.SHAPE_T_CIRCLE:
            record.update(center_mm=[_to_mm(pcbnew, item.GetCenter().x), _to_mm(pcbnew, item.GetCenter().y)],
                          radius_mm=_to_mm(pcbnew, item.GetRadius()))
        edge_records.append(record)

    footprints = [_footprint_record(pcbnew, fp) for fp in board.GetFootprints()]
    drilled_features = []
    mounting_holes = []
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        fpid = str(fp.GetFPID().GetLibItemName())
        for pad in fp.Pads():
            drill = pad.GetDrillSize()
            if drill.x <= 0 or drill.y <= 0:
                continue
            pos = pad.GetPosition()
            feature = {"reference": ref, "pad": pad.GetNumber(), "attribute": str(pad.GetAttribute()),
                       "center_mm": [_to_mm(pcbnew, pos.x), _to_mm(pcbnew, pos.y)],
                       "drill_mm": [_to_mm(pcbnew, drill.x), _to_mm(pcbnew, drill.y)],
                       "footprint_id": fpid}
            drilled_features.append(feature)
            if (ref.upper().startswith(("H", "MH")) or "mountinghole" in fpid.lower()
                    or pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH):
                mounting_holes.append(feature)

    layers = [board.GetLayerName(layer) for layer in range(pcbnew.F_Cu, pcbnew.B_Cu + 1)
              if board.IsLayerEnabled(layer) and board.GetLayerName(layer).endswith(".Cu")]
    thickness = _to_mm(pcbnew, board.GetDesignSettings().GetBoardThickness())
    connectors = [fp for fp in footprints if fp["connector_candidate"]]
    missing_models = [fp["reference"] for fp in footprints
                      if not fp["model_refs"] or any(not x["file_exists"] for x in fp["model_refs"])]
    height_blocked = [fp["reference"] for fp in footprints if fp["native_3d_height"]["status"] != "PASS"]
    return {"board": board_name, "board_path": str(board_path), "board_sha256": sha256_file(board_path),
            "pcbnew_version": pcbnew.GetBuildVersion(),
            "outline": {"source_layer": "Edge.Cuts", "primitive_count": len(edge_items),
                        "primitives": edge_records, "bbox_mm": outline_bbox,
                        "width_mm": round(width, 6), "height_mm": round(height, 6)},
            "thickness_mm": thickness, "copper_layer_count": board.GetCopperLayerCount(),
            "enabled_copper_layers": layers,
            "footprint_count": len(footprints), "footprints": footprints,
            "connector_count": len(connectors), "connectors": connectors,
            "drilled_feature_count": len(drilled_features), "drilled_features": drilled_features,
            "mounting_hole_candidate_count": len(mounting_holes), "mounting_hole_candidates": mounting_holes,
            "native_3d": {"status": "BLOCKED" if height_blocked else "BLOCKED",
                          "component_heights_mm": None, "height_unknown_references": height_blocked,
                          "missing_model_references": missing_models,
                          "reason": "pcbnew footprint model references are not a native solid-height measurement"},
            "placeholder_comparison": compare_envelope([width, height], CAD_PLACEHOLDERS_MM[board_name]),
            "pinmap": _pinmap(pinmap_path) if pinmap_path and board_name == "main" else None}


def _count_by(items: list[dict[str, Any]], key: str) -> dict[str, int]:
    result: dict[str, int] = {}
    for item in items:
        val = str(item.get(key, "unknown"))
        result[val] = result.get(val, 0) + 1
    return dict(sorted(result.items()))


def _summarize_native_report(data: dict[str, Any], kind: str) -> dict[str, Any]:
    # KiCad PCB DRC reports use top-level violations/unconnected_items. ERC
    # reports place violations under each sheet, so flatten them explicitly.
    if kind == "erc":
        violations = [violation for sheet in data.get("sheets", [])
                      for violation in sheet.get("violations", [])]
        unconnected: list[dict[str, Any]] = []
    else:
        violations = data.get("violations", [])
        unconnected = data.get("unconnected_items", [])
    categories = _count_by(violations, "type")
    severities = _count_by(violations, "severity")
    unconnected_nets = len({str(item.get("net", item.get("net_name", "unknown"))) for item in unconnected})
    findings = len(violations) + len(unconnected)
    return {"status": "FAIL" if findings else "PASS",
            "kicad_report_version": data.get("kicad_version"),
            "violation_count": len(violations), "violation_count_by_type": categories,
            "violation_count_by_severity": severities,
            "unconnected_item_count": len(unconnected),
            "unconnected_net_count_observed": unconnected_nets,
            "findings": findings,
            "unconnected_item_samples": unconnected[:20]}


def _native_check(cli: str, kind: str, source: Path, output: Path) -> dict[str, Any]:
    command = [cli, "pcb", "drc"] if kind == "drc" else [cli, "sch", "erc"]
    command += ["--severity-all", "--exit-code-violations", "--format", "json", "--output", str(output), str(source)]
    proc = subprocess.run(command, text=True, capture_output=True)
    base = {"kind": kind, "source": str(source), "source_sha256": sha256_file(source),
            "command": command, "exit_code": proc.returncode, "stdout": proc.stdout,
            "stderr": proc.stderr, "raw_output": str(output)}
    if not output.is_file():
        return {**base, "status": "BLOCKED", "reason": "KiCad did not create JSON output"}
    try:
        data = json.loads(output.read_text())
    except json.JSONDecodeError as exc:
        return {**base, "status": "FAIL", "reason": f"invalid native JSON output: {exc}"}
    summary = _summarize_native_report(data, kind)
    return {**base, **summary, "native_report_sha256": sha256_file(output)}


def _run_native_checks(root: Path, report_dir: Path) -> dict[str, Any]:
    cli = os.environ.get("KICAD_CLI", "kicad-cli")
    try:
        version = subprocess.run([cli, "version"], text=True, capture_output=True, check=True).stdout.strip()
    except (OSError, subprocess.SubprocessError) as exc:
        return {"status": "BLOCKED", "tool": cli, "error": f"{type(exc).__name__}: {exc}"}
    files = [("main", "drc", MAIN_BOARD), ("esc", "drc", ESC_BOARD),
             ("main", "erc", MAIN_SCHEMATIC), ("esc", "erc", ESC_SCHEMATIC)]
    checks = {}
    for board, kind, rel in files:
        source = root / rel
        if not source.is_file():
            checks[f"{board}_{kind}"] = {"status": "BLOCKED", "source": str(source), "reason": "source absent"}
            continue
        output = report_dir / f"{board}-{kind}.json"
        checks[f"{board}_{kind}"] = _native_check(cli, kind, source, output)
    return {"tool_version": version, "checks": checks,
            "status": "FAIL" if any(x.get("status") == "FAIL" for x in checks.values())
                     else "BLOCKED" if any(x.get("status") == "BLOCKED" for x in checks.values()) else "PASS"}


def audit_variant(root: Path, variant: dict[str, Any], report_dir: Path, manifest_path: Path,
                  run_native: bool = True) -> dict[str, Any]:
    expected_sha = str(variant["head_sha"])
    observed_sha = _git_head(root)
    revision = check_source_revision(expected_sha, observed_sha)
    if revision["status"] != "PASS":
        return {"variant": variant["id"], "source_revision": revision, "status": "FAIL",
                "error": revision["error"], "geometry": None, "native_checks": None}
    manifest_revision = variant.get("head_sha")
    report_dir.mkdir(parents=True, exist_ok=True)
    geometry = {}
    for board_name, board_path in (("main", variant["main_board"]), ("esc", variant["esc_board"])):
        path = root / board_path
        if not path.is_file():
            geometry[board_name] = {"status": "BLOCKED", "source": board_path, "reason": "PCB file missing"}
            continue
        pinmap = root / variant.get("pinmap", MAIN_PINMAP) if board_name == "main" else None
        try:
            measured = extract_board(path, board_name, pinmap)
            measured["source_revision"] = manifest_revision
            if board_name == "main" and measured["pinmap"] and measured["pinmap"]["count"] is not None:
                measured["actuator_reconciliation"] = reconcile_actuators(
                    EXPECTED_EFFECTORS["independent_count"], measured["pinmap"]["channels"])
            geometry[board_name] = measured
        except Exception as exc:
            geometry[board_name] = {"status": "FAIL", "source": board_path,
                                    "reason": f"{type(exc).__name__}: {exc}"}
    native = _run_native_checks(root, report_dir) if run_native else {"status": "NOT_RUN", "checks": {}}
    blockers = []
    for name, board in geometry.items():
        if board.get("status") in ("BLOCKED", "FAIL"):
            blockers.append(f"{name} geometry extraction {board['status']}")
        if board.get("placeholder_comparison", {}).get("status") == "FAIL":
            blockers.append(f"{name} native Edge.Cuts differs from CAD placeholder")
        if board.get("native_3d", {}).get("status") == "BLOCKED":
            blockers.append(f"{name} component heights are BLOCKED (native solid height not established)")
        actuator = board.get("actuator_reconciliation")
        if actuator and actuator["status"] != "PASS":
            blockers.append("main board PWM channel capacity is fewer than seven effectors")
    if native.get("status") in ("FAIL", "BLOCKED"):
        blockers.append(f"native ERC/DRC audit {native['status']}")
    status = "FAIL" if any("FAIL" in b or "differs" in b or "fewer" in b for b in blockers) else "BLOCKED" if blockers else "PASS"
    return {"variant": variant["id"], "title": variant.get("title"), "role": variant.get("role"),
            "release_authority": "NONE_DECLARED", "source_revision": revision,
            "source_manifest": str(manifest_path), "source_manifest_sha256": sha256_file(manifest_path),
            "geometry": geometry, "native_checks": native, "blockers": blockers, "status": status}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=Path.cwd(), help="exact source checkout to audit")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--variant", required=True, help="variant key from source manifest")
    parser.add_argument("--output-dir", type=Path, required=True, help="ignored directory for raw native reports")
    parser.add_argument("--json", type=Path, required=True, help="path for machine-readable audit summary")
    parser.add_argument("--skip-native", action="store_true", help="export geometry only; DRC/ERC are NOT_RUN")
    args = parser.parse_args()
    manifest_path = args.manifest.resolve()
    manifest = json.loads(manifest_path.read_text())
    variants = {item["id"]: item for item in manifest["variants"]}
    if args.variant not in variants:
        parser.error(f"unknown variant {args.variant!r}; choose from {', '.join(variants)}")
    root = args.source_root.resolve()
    report = audit_variant(root, variants[args.variant], args.output_dir.resolve(), manifest_path,
                           run_native=not args.skip_native)
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"variant": report["variant"], "status": report["status"],
                      "source_revision": report["source_revision"], "blockers": report.get("blockers", []),
                      "json": str(args.json.resolve())}, sort_keys=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
