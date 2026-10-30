"""Unit checks for the source-pinned PCB variant audit logic."""
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools/hardware/pcb_variant_audit.py"
spec = importlib.util.spec_from_file_location("pcb_variant_audit", SCRIPT)
audit = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = audit
spec.loader.exec_module(audit)


def test_source_revision_rejects_mismatched_checkout():
    assert audit.check_source_revision("abc", "abc")["status"] == "PASS"
    result = audit.check_source_revision("abc", "def")
    assert result["status"] == "FAIL"
    assert result["error"]


def test_envelope_uses_explicit_tolerance():
    assert audit.compare_envelope([140.04, 100.0], [140.0, 100.0])["status"] == "PASS"
    assert audit.compare_envelope([140.06, 100.0], [140.0, 100.0])["status"] == "FAIL"


def test_actuator_reconciliation_does_not_invent_outputs():
    channels = [{"net": f"PWM_ESC{i}"} for i in range(1, 7)]
    result = audit.reconcile_actuators(7, channels)
    assert result["status"] == "FAIL"
    assert result["unmapped_effector_count"] == 1
    assert result["phantom_gpio_created"] is False


def test_manifest_has_comparison_only_variants():
    manifest = audit.json.loads(audit.DEFAULT_MANIFEST.read_text())
    variants = manifest["variants"]
    assert {variant["id"] for variant in variants} == {"main_baseline", "pr10", "pr11", "pr17"}
    assert manifest["selection_policy"]["fabrication_release_authority"] is None
    assert all(variant["head_sha"] for variant in variants)


def test_erc_summary_flattens_sheet_violations_and_severities():
    report = {
        "kicad_version": "9.0.2",
        "sheets": [
            {"violations": [{"type": "symbol_issue", "severity": "warning"}]},
            {"violations": [{"type": "pin_not_driven", "severity": "error"}]},
            {"path": "/empty", "violations": []},
        ],
    }
    result = audit._summarize_native_report(report, "erc")
    assert result["status"] == "FAIL"
    assert result["violation_count"] == 2
    assert result["violation_count_by_severity"] == {"error": 1, "warning": 1}
    assert result["findings"] == 2


def test_rectangular_edge_centerlines_exclude_stroke_and_match_pr34_reader():
    # PR34's native reader bounds straight Edge.Cuts paths from GetStart/GetEnd.
    rectangle_edges = [
        {"start_mm": [0.0, 0.0], "end_mm": [140.0, 0.0]},
        {"start_mm": [140.0, 0.0], "end_mm": [140.0, 100.0]},
        {"start_mm": [140.0, 100.0], "end_mm": [0.0, 100.0]},
        {"start_mm": [0.0, 100.0], "end_mm": [0.0, 0.0]},
    ]
    points = [point for edge in rectangle_edges for point in (edge["start_mm"], edge["end_mm"])]
    measured = audit._point_bounds(points)
    # This is PR34's endpoint/center envelope algorithm applied to straight paths.
    pr34_points = [point for edge in rectangle_edges
                   for point in (edge["start_mm"], edge["end_mm"], edge.get("center_mm"))
                   if point is not None]
    assert measured == audit._point_bounds(pr34_points) == [0.0, 0.0, 140.0, 100.0]
    audit._validate_closed_edge_paths(rectangle_edges)
    # A 0.15 mm graphic line has a separate stroke envelope.
    assert [-0.075, -0.075, 140.075, 100.075] != audit._point_bounds(points)


def test_arc_centerline_bounds_include_only_swept_cardinal_extrema():
    points = audit._arc_centerline_points([10.0, 0.0], [0.0, 10.0], [0.0, 0.0], 10.0, False)
    assert audit._point_bounds(points) == [0.0, 0.0, 10.0, 10.0]


def test_open_edge_cuts_contour_fails_closed():
    try:
        audit._validate_closed_edge_paths([{"start_mm": [0.0, 0.0], "end_mm": [1.0, 0.0]}])
    except ValueError as exc:
        assert "open or branched" in str(exc)
    else:
        raise AssertionError("open Edge.Cuts path must not produce an outline measurement")


def test_zero_length_and_invalid_edge_primitives_fail_closed():
    invalid_primitives = [
        [{"start_mm": [0.0, 0.0], "end_mm": [0.0, 0.0]}],
        [{"center_mm": [0.0, 0.0], "radius_mm": 0.0}],
        [{"center_mm": [0.0, 0.0], "radius_mm": -1.0}],
        [{"center_mm": [0.0, 0.0], "radius_mm": float("nan")}],
        [{"center_mm": [float("inf"), 0.0], "radius_mm": 1.0}],
        [{"center_mm": [0.0, 0.0]}],
    ]
    for primitives in invalid_primitives:
        try:
            audit._validate_closed_edge_paths(primitives)
        except ValueError:
            continue
        raise AssertionError(f"invalid Edge.Cuts primitive was accepted: {primitives!r}")
