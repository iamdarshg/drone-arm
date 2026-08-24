"""Manufacturing tolerance sensitivity check for the final motor design.

Perturbs each critical parameter by its manufacturing tolerance in both
directions, reruns the full safety evaluation per case, then checks the
worst-case combination. Writes hardware/motor_release/sensitivity_results.txt.

Usage: python tools/motor_release_sensitivity.py
"""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

spec = importlib.util.spec_from_file_location(
    "mo", ROOT / "hardware" / "motor_optimizer.py"
)
mo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mo)


def main():
    with open(
        ROOT / "hardware" / "motor_release" / "optimizer_result.json",
        encoding="utf-8",
    ) as f:
        base = json.load(f)["params"]

    perturbs = {
        "Rint": 0.05,
        "Rrotor_ext": 0.05,
        "magnet_thickness": 0.10,
        "motor_length": 0.30,
        "wire_diameter": 0.02,
    }

    lines = ["Manufacturing tolerance sensitivity - final design", "=" * 50, ""]
    all_safe = True
    for key, delta in perturbs.items():
        results = []
        for sign in (+1, -1):
            p = dict(base)
            p[key] = p[key] + sign * delta
            if key == "Rint":
                p["Rrotor_ext"] = p["Rint"] - 0.5
            res = mo.evaluate_design(p, verbose=False, thermal_resolution=(14, 20))
            ok = res is not None and res["overall_safe"]
            tmax = "-" if res is None else round(res["T_max"], 1)
            results.append((sign, delta, ok, tmax))
            all_safe &= ok
        row = ", ".join(f"{s:+}{d}: {'SAFE' if ok else 'UNSAFE'} Tmax={t} C"
                        for s, d, ok, t in results)
        lines.append(f"{key}: {row}")
        print(lines[-1])

    # Worst-case combined: minimum airgap + thin magnets
    p = dict(base)
    p["Rrotor_ext"] += 0.05
    p["Rint"] -= 0.05
    p["magnet_thickness"] -= 0.10
    res = mo.evaluate_design(p, verbose=False, thermal_resolution=(20, 30))
    ok = res is not None and res["overall_safe"]
    gap_ok = "-" if res is None else res["thermally_safe"]
    tmax = "-" if res is None else round(res["T_max"], 1)
    worst = f"Worst-case combined: {'SAFE' if ok else 'UNSAFE'} Tmax={tmax} C gap_ok={gap_ok}"
    lines += ["", worst]
    print(worst)
    summary = f"ALL PERTURBATIONS SAFE: {all_safe}"
    lines += [summary]
    print(summary)

    with open(
        ROOT / "hardware" / "motor_release" / "sensitivity_results.txt",
        "w",
        encoding="utf-8",
    ) as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
