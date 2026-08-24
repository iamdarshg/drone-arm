"""Bounded design search for the motor release package.

The original iterative-deepening driver materializes the full cartesian
product of its parameter grid before applying its evaluation cap, so it can
never run as written. This driver samples the same constraint-checked
evaluate_design/objective_score functions with a fixed random budget and a
local refinement pass around the best candidate.

Usage:  python tools/motor_release_search.py [n_samples] [seed]
"""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load_optimizer():
    spec = importlib.util.spec_from_file_location(
        "motor_optimizer", ROOT / "hardware" / "motor_optimizer.py"
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules["motor_optimizer"] = mod
    spec.loader.exec_module(mod)
    return mod


def main():
    n_samples = int(sys.argv[1]) if len(sys.argv) > 1 else 4000
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 42
    rng = np.random.default_rng(seed)
    mo = load_optimizer()

    base = dict(mo.BASE_PARAMS)
    target_power = base["voltage_dc"] * base["I_continuous"] * mo.TARGET_EFFICIENCY

    keys = list(mo.VARIABLE_PARAMS.keys())
    def sample():
        p = dict(base)
        for k in keys:
            lo, hi = mo.VARIABLE_PARAMS[k]
            if k in ("pole_pairs", "stator_slots", "parallel_strands"):
                p[k] = int(rng.integers(lo, hi + 1))
            else:
                p[k] = float(rng.uniform(lo, hi))
        # keep rotor/stator geometry consistent
        p["Rrotor_ext"] = p["Rint"] - 0.5
        if p["Rrotor_ext"] <= p["Rshaft"]:
            p["Rrotor_ext"] = p["Rshaft"] + 0.25
            p["Rint"] = p["Rrotor_ext"] + 0.5
        return p

    best = None
    best_score = float("inf")
    for i in range(n_samples):
        p = sample()
        res = mo.evaluate_design(p, verbose=False, thermal_resolution=(10, 15))
        if res is None:
            continue
        score = mo.objective_score(res, target_power, require_safe=True)
        if score < best_score:
            best_score = score
            best = (p.copy(), res)
            print(f"[{i}] new best score={score:.1f}", flush=True)

    if best is None:
        print("NO FEASIBLE DESIGN FOUND")
        return 1

    # Local refinement around best
    p_best, res_best = best
    # Bias refinement toward thermally safer neighbors: require strict margin
    def strictly_safe(res):
        return (
            res["overall_safe"]
            and res["T_max"] < mo.MAGNET_TEMP_LIMIT - mo.TEMP_SAFETY_MARGIN
            and res["mech_safety"]["min_margin"] > 0.05
        )

    best_strict = None
    best_strict_score = float("inf")
    for it in range(400):
        p = p_best.copy()
        k = keys[rng.integers(len(keys))]
        lo, hi = mo.VARIABLE_PARAMS[k]
        span = (hi - lo) * 0.05
        v = float(p[k]) + rng.normal(0, span)
        if k in ("pole_pairs", "stator_slots", "parallel_strands"):
            v = int(np.clip(round(v), lo, hi))
        else:
            v = float(np.clip(v, lo, hi))
            p[k] = v
        if k == "pole_pairs":
            p["stator_slots"] = int(
                np.clip(p["stator_slots"], *mo.VARIABLE_PARAMS["stator_slots"])
            )
        if "Rint" in p and "Rshaft" in p:
            p["Rrotor_ext"] = p["Rint"] - 0.5
            if p["Rrotor_ext"] <= p["Rshaft"]:
                continue
        res = mo.evaluate_design(p, verbose=False, thermal_resolution=(14, 20))
        if res is None:
            continue
        score = mo.objective_score(res, target_power, require_safe=True)
        if score < best_score:
            best_score = score
            p_best, res_best = p.copy(), res
        if score < best_strict_score and res["overall_safe"]:
            if strictly_safe(res):
                best_strict_score = score
                best_strict = (p.copy(), res)

    if best_strict is not None:
        p_best, res_best = best_strict
        best_score = best_strict_score

    out = {
        "score": float(best_score),
        "params": {k: (v if not isinstance(v, (np.floating, np.integer)) else v.item())
                   for k, v in p_best.items()},
        "result": {
            "overall_safe": bool(res_best["overall_safe"]),
            "torque_Nm": float(res_best["torque_Nm"]),
            "T_max_C": float(res_best["T_max"]),
            "T_max_winding_C": float(res_best["T_max_winding"]),
            "efficiency": float(res_best["efficiency"]),
            "rpm_max_elec": float(res_best["rpm_max_elec"]),
            "rpm_proof": float(res_best["rpm_proof"]),
            "overspeed_safe": bool(res_best["overspeed_safe"]),
            "mechanically_safe": bool(res_best["mechanically_safe"]),
            "mechanical_margin": float(res_best["mech_safety"]["min_margin"]),
            "mechanical_limiting_factor": str(res_best["mech_safety"]["limiting_factor"]),
            "magnet_safe": bool(res_best["magnet_safe"]),
            "axial_safe": bool(res_best["axial_safe"]),
            "copper_loss_W": float(res_best["copper_loss_W"]),
            "electrical_power_W": float(res_best["electrical_power_W"]),
            "winding": {k: float(v) for k, v in res_best["winding"].items()},
        },
    }
    outpath = ROOT / "hardware" / "motor_release" / "optimizer_result.json"
    with open(outpath, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print(json.dumps(out["result"], indent=2))
    print(f"saved: {outpath}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
