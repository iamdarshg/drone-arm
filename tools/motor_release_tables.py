"""Generate performance.json, winding_table.csv, thermal_results.csv,
efficiency_map.csv from the final optimizer result, with values rounded to
manufacturable increments."""

import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REL = ROOT / "hardware" / "motor_release"


def load_optimizer():
    spec = importlib.util.spec_from_file_location(
        "mo", ROOT / "hardware" / "motor_optimizer.py"
    )
    mod = importlib.util.module_from_spec(spec)
    sys.modules["mo"] = mod
    spec.loader.exec_module(mod)
    return mod


def main():
    mo = load_optimizer()
    with open(REL / "optimizer_result.json", encoding="utf-8") as f:
        data = json.load(f)
    p = dict(data["params"])

    # Round to manufacturable values
    p["Rext"] = 25.5
    p["Rshaft"] = 4.0
    p["magnet_thickness"] = 6.5
    p["motor_length"] = 56.0
    p["wire_diameter"] = 3.15  # AWG-reachable nominal with heavy insulation
    p["parallel_strands"] = 3
    p["slot_opening"] = 3.3
    p["h_conv"] = 90.0  # conservative forced-air coefficient at 10 m/s

    res = mo.evaluate_design(p, verbose=False, thermal_resolution=(30, 40))
    if res is None:
        raise SystemExit("rounded design rejected - adjust rounding")

    w = res["winding"]
    rpm_max = res["rpm_max_elec"]
    pp = int(p["pole_pairs"])
    slots = int(p["stator_slots"])

    # Electrical ratings from the analytical model
    vbus_nom = 44.4
    vbus_max = 50.4
    kv = rpm_max / vbus_max if vbus_max else 0
    kt = res["torque_Nm"] / p["I_continuous"]

    performance = {
        "design_point": {
            "continuous": {
                "torque_Nm": round(res["torque_Nm"], 3),
                "speed_rpm": round(rpm_max, 0),
                "phase_current_A_rms": 60.0,
                "vbus_V": 44.4,
            },
            "burst": {"duration_s": 30, "phase_current_A_rms": 90.0},
        },
        "electrical": {
            "R_phase_line_to_line_mOhm": round(2 * w["R_dc_ohm"] * 1000, 2),
            "L_phase_estimate_uH": None,
            "Kt_Nm_per_A_rms": round(kt, 4),
            "Kv_rpm_per_V": round(kv, 1),
            "back_emf_line_to_line_rms_V_per_krpm": round(1000.0 / kv, 2),
            "copper_loss_W_continuous": round(res["copper_loss_W"], 1),
            "iron_loss_model_fraction_of_copper": 0.10
        },
        "thermal": {
            "ambient_C": 45,
            "T_winding_continuous_C": round(res["T_max_winding"], 1),
            "T_magnet_continuous_C": round(res["T_max"], 1),
            "insulation_class": "H (180C), operating ceiling 170C",
            "magnet_grade_limit_C": 150,
            "steady_state_safe": bool(res["overall_safe"]),
            "burst_30s_estimate_C": round(
                res["T_max"]
                + (w["P_cu_burst_W"] - w["P_cu_cont_W"]) / 60.0 * 0.35, 1)
        },
        "mechanical": {
            "max_operating_rpm": round(rpm_max, 0),
            "proof_rpm_125pct": round(res["rpm_proof"], 0),
            "overspeed_safe": bool(res["overspeed_safe"]),
            "mechanical_margin": round(float(res["mech_safety"]["min_margin"]), 3),
            "airgap_mm": round(p["Rint"] - p["Rrotor_ext"], 2)
        },
        "efficiency_at_design_point": round(res["efficiency"], 4),
        "safety_gates_passed": {
            "magnet_temp": bool(res["magnet_safe"]),
            "thermal_expansion": bool(res["thermally_safe"]),
            "mechanical": bool(res["mechanically_safe"]),
            "axial": bool(res["axial_safe"]),
            "overspeed": bool(res["overspeed_safe"]),
            "overall": bool(res["overall_safe"])
        },
        "geometry_final": {k: round(v, 3) for k, v in p.items()
                          if isinstance(v, (int, float))}
    }
    with open(REL / "performance.json", "w", encoding="utf-8") as f:
        json.dump(performance, f, indent=2)
    print("performance.json written; overall_safe =", res["overall_safe"],
          "Tmax =", round(res["T_max"], 1))

    # Persist the FINAL rounded design as the canonical optimizer result
    final = {
        "score": 0.0,
        "params": p,
        "result": {
            "overall_safe": bool(res["overall_safe"]),
            "torque_Nm": float(res["torque_Nm"]),
            "T_max_C": float(res["T_max"]),
            "T_max_winding_C": float(res["T_max_winding"]),
            "efficiency": float(res["efficiency"]),
            "rpm_max_elec": float(res["rpm_max_elec"]),
            "rpm_proof": float(res["rpm_proof"]),
            "overspeed_safe": bool(res["overspeed_safe"]),
            "mechanically_safe": bool(res["mechanically_safe"]),
            "mechanical_margin": float(res["mech_safety"]["min_margin"]),
            "mechanical_limiting_factor": str(res["mech_safety"]["limiting_factor"]),
            "magnet_safe": bool(res["magnet_safe"]),
            "axial_safe": bool(res["axial_safe"]),
            "thermally_safe": bool(res["thermally_safe"]),
            "copper_loss_W": float(res["copper_loss_W"]),
            "electrical_power_W": float(res["electrical_power_W"]),
            "winding": {k: float(v) for k, v in res["winding"].items()},
        },
    }
    with open(REL / "optimizer_result.json", "w", encoding="utf-8") as f:
        json.dump(final, f, indent=2)
    print("optimizer_result.json updated with final rounded design")

    # winding_table.csv
    with open(REL / "winding_table.csv", "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["slot", "phase", "layer", "turns", "direction",
                     "wire", "strands"])
        # Balanced double-layer FSCW distribution: phase = slot index mapped by
        # star of slots; alternate in/out direction every coil side.
        for i in range(slots):
            phase = ["U", "V", "W"][i % 3] if False else ["U", "V", "W"][(i * pp) % 3]
            direction = "in" if ((i // 1) % 2 == 0) else "out"
            turns_per_layer = int(w["turns_per_slot"] // 2)
            extra = int(w["turns_per_slot"] % 2)
            wr.writerow([i + 1, phase, 1, turns_per_layer + extra, direction,
                         f"{p['wire_diameter']:.2f}mm", p["parallel_strands"]])
            wr.writerow([i + 1, phase, 2, turns_per_layer,
                         "out" if direction == "in" else "in",
                         f"{p['wire_diameter']:.2f}mm", p["parallel_strands"]])
    print("winding_table.csv written")

    # thermal_results.csv - transient curve from single-capacity estimate
    C_th = 900.0  # J/K rough thermal capacity of stator copper+iron
    P_net = res["copper_loss_W"] * 1.1  # + iron model fraction
    R_th = (res["T_max_winding"] - p["T_ambient"]) / max(P_net, 1e-9)
    tau = R_th * C_th
    with open(REL / "thermal_results.csv", "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["time_s", "T_winding_C", "case"])
        for t in [0, 30, 60, 120, 300, 600, 900, 1200, 1800]:
            T = p["T_ambient"] + (res["T_max_winding"] - p["T_ambient"]) * (
                1 - math.exp(-t / tau))
            wr.writerow([t, round(T, 1), "continuous"])
        # burst on top of steady state
        T_ss = res["T_max_winding"]
        for t in [10, 20, 30]:
            dT = (w["P_cu_burst_W"] - w["P_cu_cont_W"]) * t / C_th
            wr.writerow([t, round(T_ss + dT, 1), "burst_from_steady_state"])
    print("thermal_results.csv written")

    # efficiency_map.csv over torque-speed grid
    with open(REL / "efficiency_map.csv", "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["speed_rpm", "I_phase_A", "torque_Nm", "P_elec_W",
                     "P_cu_W", "P_fe_W", "eta"])
        I = p["I_continuous"]
        for frac_speed in [0.25, 0.5, 0.75, 1.0]:
            n = rpm_max * frac_speed
            omega = n * 2 * math.pi / 60
            T = kt * I
            P_mech = T * omega
            P_cu = w["P_cu_cont_W"] * (frac_speed ** 0)  # resistive, speed-independent
            P_fe = P_cu * 0.10 * frac_speed  # iron loss grows with speed
            P_elec = P_mech + P_cu + P_fe
            eta = P_mech / P_elec if P_elec > 0 else 0
            wr.writerow([round(n), I, round(T, 3), round(P_elec, 1),
                         round(P_cu, 1), round(P_fe, 1), round(eta, 4)])
    print("efficiency_map.csv written")


if __name__ == "__main__":
    main()
