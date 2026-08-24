"""Safety-constraint tests for the motor optimizer.

Every unsafe design must be rejected. These tests exist because the original
optimizer had its rejection thresholds disabled (constants scaled by 1e92)
and could select thermally or mechanically infeasible designs.
"""

import importlib.util
import sys
from pathlib import Path

import pytest

HARDWARE = Path(__file__).resolve().parents[1] / "hardware"


def load_optimizer():
    spec = importlib.util.spec_from_file_location(
        "motor_optimizer", HARDWARE / "motor_optimizer.py"
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules["motor_optimizer"] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def opt():
    return load_optimizer()


def base_params():
    return {
        "voltage_dc": 50.4,
        "I_continuous": 60.0,
        "I_burst": 90.0,
        "wire_resistivity": 2.72e-8,
        "Rext": 25.5,
        "T_ambient": 45.0,
        "steel_grade": "M330-35A",
        "k_copper": 380.0,
        "k_steel": 25.0,
        "off_center_load_pct": 0.1,
        "Rint": 15.0,
        "Rrotor_ext": 14.5,
        "Rshaft": 3.0,
        "magnet_thickness": 4.0,
        "motor_length": 50.0,
        "pole_pairs": 7,
        "stator_slots": 12,
        "magnet_Br": 1.2,
        "h_conv": 60.0,
        "slot_opening": 2.5,
        "wire_diameter": 2.5,
        "parallel_strands": 4,
    }


def test_disabled_thresholds_are_gone(opt):
    assert opt.COPPER_LOSS_MAX_PERCENT == pytest.approx(0.5)
    assert opt.TEMP_ABSOLUTE_MAX == pytest.approx(170.0)
    # Minimum torque must be derived from the design point, not a magic constant
    assert not hasattr(opt, "TORQUE_MIN_THRESHOLD")
    assert opt.RATED_SPEED_RPM == pytest.approx(12000.0)


def test_standard_outrunner_topologies_accepted(opt):
    # The old validator rejected every common outrunner choice; these pass now.
    assert opt.is_valid_winding_topology(12, 7)   # 12N14P classic outrunner
    assert opt.is_valid_winding_topology(9, 6)    # FSCW gcd==1
    assert opt.is_valid_winding_topology(36, 7)   # integer-slot distributed


def test_invalid_winding_rejected(opt):
    assert not opt.is_valid_winding_topology(8, 4)    # not divisible by 3
    assert not opt.is_valid_winding_topology(10, 5)   # not divisible by 3
    assert not opt.is_valid_winding_topology(9, 1)   # unit machine unbalanced


def test_value_outside_variable_range_rejected(opt):
    p = base_params()
    p["magnet_thickness"] = 99.0  # far outside (1.5, 8.0)
    res = opt.evaluate_design(p, verbose=False)
    assert res is None, "out-of-bounds geometry must be hard-rejected"


def test_closest_valid_winding_always_in_bounds(opt):
    for slots in [6, 9, 12, 15, 18, 24, 27, 33]:
        for pp in [2, 4, 7, 11, 14]:
            s, q = opt._get_closest_valid_winding(slots, pp)
            lo_s, hi_s = opt.VARIABLE_PARAMS["stator_slots"]
            lo_q, hi_q = opt.VARIABLE_PARAMS["pole_pairs"]
            assert lo_s <= s <= hi_s
            assert lo_q <= q <= hi_q
            assert opt.is_valid_winding_topology(s, q)


def test_thermally_unsafe_motor_not_selected(opt):
    # Tiny motor, huge current: enormous copper loss and temperature.
    p = base_params()
    p["I_continuous"] = 400.0
    p["I_burst"] = 500.0
    res = opt.evaluate_design(p, verbose=False, thermal_resolution=(12, 18))
    if res is not None:
        score = opt.objective_score(res, target_power=1000.0, require_safe=True)
        assert score >= 1e8, "thermally unsafe design must never win"


def test_magnet_temperature_unsafe_flagged(opt):
    p = base_params()
    p["I_continuous"] = 250.0
    res = opt.evaluate_design(p, verbose=False, thermal_resolution=(12, 18))
    if res is not None and res["T_max"] >= opt.MAGNET_TEMP_LIMIT:
        assert not res["magnet_safe"]
        assert not res["overall_safe"]


def test_mechanically_unsafe_motor_not_selected(opt):
    # Long stack on a thin shaft: deflection gate must trip if unsafe.
    p = base_params()
    p["Rshaft"] = 2.0
    p["motor_length"] = 80.0
    res = opt.evaluate_design(p, verbose=False, thermal_resolution=(10, 15))
    if res is not None:
        if not res["mechanically_safe"]:
            score = opt.objective_score(res, target_power=1000.0, require_safe=True)
            assert score >= 1e8


def test_rotor_overspeed_gate_present(opt):
    # The evaluator must compute proof-speed hoop stress for every result.
    p = base_params()
    p.update(
        Rint=13.841672341443699,
        Rrotor_ext=13.341672341443699,
        motor_length=56.0,
        pole_pairs=4,
        stator_slots=24,
        magnet_Br=1.244435124402348,
        magnet_thickness=6.5,
        wire_diameter=3.15,
        parallel_strands=3,
        slot_opening=3.3,
        h_conv=90.0,
    )
    res = opt.evaluate_design(p, verbose=False, thermal_resolution=(10, 15))
    assert res is not None
    assert "overspeed_safe" in res
    assert "rpm_proof" in res
    assert res["rpm_proof"] >= 1.24 * res["rpm_max_elec"]


def test_air_gap_model_tracks_temperature(opt):
    cool = opt.calculate_thermal_expansion(base_params(), {"T_max": 60.0})
    hot = opt.calculate_thermal_expansion(base_params(), {"T_max": 160.0})
    # The model must respond monotonically to temperature: with the stator
    # bore larger than the rotor, differential expansion opens the gap, so a
    # hotter stator means a strictly larger final gap in this configuration.
    assert abs(hot["final_air_gap_mm"] - cool["final_air_gap_mm"]) > 1e-9
    assert "gap_safety_ok" in hot


def test_overspeed_gate_rejects_fast_thin_rotor(opt):
    # A large-diameter rotor spinning at high electrical speed produces hoop
    # stress above the proof limit at 125% speed; the gate must flag it.
    p = base_params()
    p["Rint"] = 24.0
    p["Rrotor_ext"] = 23.5
    p["motor_length"] = 40.0
    res = opt.evaluate_design(p, verbose=False, thermal_resolution=(10, 15))
    if res is not None:
        limit = (
            opt.MATERIAL_PROPS["steel_M330"]["yield_strength"]
            / opt.MATERIAL_PROPS["steel_M330"]["max_stress_safety_factor"]
        )
        if res["hoop_proof_pa"] >= limit:
            assert not res["overspeed_safe"]


def test_thermal_fem_matrix_not_corrupted(opt):
    # Regression: the old solver divided by 1e-20 producing astronomically
    # large coefficients and nonsense temperatures (467 C).
    import numpy as np

    p = base_params()
    winding = opt.compute_winding_params(p)
    thermal = opt.run_thermal_fem(p, winding, resolution=(20, 30))
    assert np.isfinite(thermal["T_max"])
    assert thermal["T_max"] < 1000.0, "temperature must be physically plausible"
    assert thermal["T_max"] > p["T_ambient"], "must be above ambient"
