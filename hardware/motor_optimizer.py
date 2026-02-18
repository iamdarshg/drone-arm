# ============================================================
# MOTOR DESIGN OPTIMIZER — ITERATIVE DEEPENING
# Uses real pyleecan magnetics analysis and progressive refinement
# ============================================================

import numpy as np
import json
import sys
from copy import deepcopy
import matplotlib.pyplot as plt
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve
from scipy.optimize import fsolve

try:
    import cadquery as cq
    from cadquery import Assembly, Color
    HAS_CADQUERY = True
except ImportError:
    HAS_CADQUERY = False
    print("WARNING: CadQuery not available, STL export disabled")

# ============================================================
# OPTIMIZATION PARAMETERS
# ============================================================
MAGNET_TEMP_LIMIT = 120.0
TEMP_SAFETY_MARGIN = 5.0
MAX_REFINEMENT_ITERATIONS = 1500
INITIAL_DIVISION_COUNT = 12  # Coarse-to-fine divisions

# ============================================================
# MATERIAL PROPERTIES & THERMAL/MECHANICAL DATA
# ============================================================
MATERIAL_PROPS = {
    "copper": {
        "density": 8960,              # kg/m³
        "E_modulus": 110e9,           # Pa (Young's modulus)
        "yield_strength": 200e6,      # Pa (annealed copper)
        "CTE": 16.5e-6,               # /°C (coefficient of thermal expansion)
        "max_stress_safety_factor": 2.0,  # Safety factor for yield
    },
    "steel_M330": {  # M330-35A electrical steel
        "density": 7850,              # kg/m³
        "E_modulus": 210e9,           # Pa
        "yield_strength": 400e6,      # Pa
        "CTE": 12.0e-6,               # /°C
        "max_stress_safety_factor": 2.5,
        "saturation_B": 2.0,          # Tesla (saturation flux density)
    },
    "magnet_N45SH": {
        "density": 7450,              # kg/m³
        "CTE": 5.0e-6,                # /°C (lower than steel)
        "max_compression_stress": 150e6,  # Pa (max compressive)
        "demag_T_coefficient": -0.11,  # %/°C (remanence temp coefficient)
    },
}

# ============================================================
# CONSTRAINED & VARIABLE PARAMETERS
# ============================================================
# FIXED (cannot modify):
# - voltage_dc
# - I_continuous, I_burst
# - wire_resistivity
# - steel_grade, magnet grade properties
# - Rext (external radius)
# - T_ambient
# - fill_factor

# VARIABLE (can be optimized):
# NOTE: Rrotor_ext is derived from Rint (always = Rint - 0.5mm)
VARIABLE_PARAMS = {
    "Rint": (8.0, 25.0),              # stator inner radius [mm]
    "Rshaft": (2.0, 4.0),              # shaft radius [mm]
    "magnet_thickness": (1.5, 8.0),    # magnet thickness [mm]
    "motor_length": (30.0, 80.0),      # motor stack length [mm]
    "pole_pairs": (4, 12),             # integer, pole pairs
    "stator_slots": (9, 24),           # integer, slot count
    "magnet_Br": (1.0, 1.35),          # remanence [T]
    "h_conv": (20.0, 100.0),           # convection coefficient [W/m²K]
    "slot_opening": (1.5, 4.0),        # slot opening width [mm]
}

# ============================================================
# CORE SIMULATION FUNCTIONS
# ============================================================

def compute_winding_params(p):
    """Calculate winding parameters and copper losses"""
    Rext = p["Rext"] * 1e-3
    Rint = p["Rint"] * 1e-3
    L = p["motor_length"] * 1e-3
    slots = p["stator_slots"]

    tooth_angle = 2 * np.pi / slots
    slot_area = 0.5 * (Rext**2 - Rint**2) * tooth_angle * p["fill_factor"]
    J_target = 8e6
    wire_area = p["I_continuous"] / J_target
    turns_per_slot = max(1, int(slot_area / wire_area))
    mean_turn_len = np.pi * (Rext + Rint) + 2 * L
    total_turns = turns_per_slot * slots
    R_dc = p["wire_resistivity"] * mean_turn_len * total_turns / (wire_area * slots / 3)
    P_cu_cont = 3 * (p["I_continuous"] / np.sqrt(3))**2 * R_dc
    P_cu_burst = 3 * (p["I_burst"] / np.sqrt(3))**2 * R_dc

    results = {
        "slot_area_mm2": slot_area * 1e6,
        "wire_area_mm2": wire_area * 1e6,
        "turns_per_slot": int(turns_per_slot),
        "R_dc_ohm": R_dc,
        "P_cu_cont_W": P_cu_cont,
        "P_cu_burst_W": P_cu_burst,
        "wire_diameter_mm": 2 * np.sqrt(wire_area / np.pi) * 1e3,
    }
    return results


def run_magnetics_pyleecan(p, winding, air_gap_adjustment=0.0, verbose=False):
    """
    Run real magnetics analysis using pyleecan + FEMM
    Falls back to analytical if pyleecan unavailable

    Args:
        air_gap_adjustment: Radial deflection to adjust air gap (mm), negative = gap narrows
    """
    try:
        from pyleecan.Classes.MachineSIPMSM import MachineSIPMSM
        from pyleecan.Classes.LamSlotWind import LamSlotWind
        from pyleecan.Classes.LamSlotLM import LamSlotLM
        from pyleecan.Classes.SlotW29 import SlotW29
        from pyleecan.Classes.HoleM50 import HoleM50
        from pyleecan.Classes.Winding import Winding
        from pyleecan.Classes.Simulation import Simulation
        from pyleecan.Classes.MagFEMM import MagFEMM
        from pyleecan.Classes.InputCurrent import InputCurrent

        stator_slot = SlotW29(
            Zs=p["stator_slots"],
            H0=0.5e-3,
            H1=1.0e-3,
            H2=(p["Rext"] - p["Rint"] - 2.5) * 1e-3,
            W0=p["slot_opening"] * 1e-3,
            W1=3.5e-3,
            W2=4.5e-3,
        )

        # Apply air gap adjustment from mechanical deflection
        Rint_adj = p["Rint"] * 1e-3
        Rrotor_adj = (p["Rrotor_ext"] + air_gap_adjustment * 1e-3) * 1e-3  # Convert mm to m for adjustment

        stator = LamSlotWind(
            Rint=Rint_adj,
            Rext=p["Rext"] * 1e-3,
            L1=p["motor_length"] * 1e-3,
            slot=stator_slot,
            winding=Winding(
                qs=3,
                p=p["pole_pairs"],
                Ntcoil=winding["turns_per_slot"]
            ),
        )

        rotor_hole = HoleM50(
            Zh=p["pole_pairs"] * 2,
            W0=0.005, W1=0.004, W2=0.010,
            W3=0.002, W4=0.002,
            H0=0.001, H1=0.0015,
            H2=p["magnet_thickness"] * 1e-3,
            H3=0.001, H4=0.0,
        )
        rotor = LamSlotLM(
            Rint=p["Rshaft"] * 1e-3,
            Rext=Rrotor_adj,  # Use deflection-adjusted rotor radius
            L1=p["motor_length"] * 1e-3,
            hole=[rotor_hole],
        )

        machine = MachineSIPMSM(stator=stator, rotor=rotor, name="drone_motor_opt")

        simu = Simulation(
            machine=machine,
            input=InputCurrent(
                Id_ref=0,
                Iq_ref=p["I_continuous"],
                Na_tot=2048,
                Nt_tot=32,
                felec=500.0,
            ),
            mag=MagFEMM(
                is_periodicity_a=True,
                is_periodicity_t=True,
                nb_worker=4,
                is_fast_draw=True,
                is_sliding_band=True,
                is_calc_torque_energy=True,
                T_mag=p["T_ambient"] + 25,
            ),
        )

        out = simu.run()
        Tem_vals = out.mag.Tem.values
        Tem_avg = float(np.mean(Tem_vals))
        Tem_rip = float(out.mag.Tem_rip_norm)
        time_vec = out.mag.Tem.axes[0].values

        if verbose:
            print(f"    [FEMM] Torque: {Tem_avg:.3f}Nm, Ripple: {Tem_rip*100:.1f}%", flush=True)

        return {"Tem_avg": Tem_avg, "Tem_rip": Tem_rip,
                "Tem_vals": Tem_vals, "time_vec": time_vec, "is_pyleecan": True}

    except Exception as e:
        # Fallback to analytical
        if verbose:
            print(f"    [Analytical fallback] {type(e).__name__}", flush=True)

        Rrotor = p["Rrotor_ext"] * 1e-3
        L = p["motor_length"] * 1e-3
        p_ = p["pole_pairs"]
        N = winding["turns_per_slot"] * p["stator_slots"]
        Ke = (4 / np.pi) * N * p["magnet_Br"] * Rrotor * L / p_
        Tem_avg = Ke * p["I_continuous"]

        Nt = 256
        theta = np.linspace(0, 2 * np.pi, Nt)
        ripple = 0.04 * Tem_avg * np.cos(p["stator_slots"] * theta)
        Tem_vals = Tem_avg + ripple

        return {"Tem_avg": Tem_avg, "Tem_rip": 0.04,
                "Tem_vals": Tem_vals, "time_vec": theta, "is_pyleecan": False}


def run_thermal_fem(p, winding):
    """2D Axisymmetric thermal FEM solver"""
    Nr, Nz = 40, 60
    r_min = p["Rshaft"] * 1e-3
    r_max = p["Rext"] * 1e-3
    z_max = p["motor_length"] * 1e-3 / 2
    dr = (r_max - r_min) / (Nr - 1)
    dz = z_max / (Nz - 1)
    r = np.linspace(r_min, r_max, Nr)
    z = np.linspace(0, z_max, Nz)

    N_nodes = Nr * Nz
    A = lil_matrix((N_nodes, N_nodes))
    b = np.zeros(N_nodes)

    def idx(i, j): return i * Nz + j

    slot_vol = (winding["slot_area_mm2"] * 1e-6 *
                p["motor_length"] * 1e-3 * p["stator_slots"])
    Q_cu = winding["P_cu_cont_W"] / max(slot_vol, 1e-8)
    Q_fe = Q_cu * 0.05
    Rint_m = p["Rint"] * 1e-3

    for i in range(Nr):
        for j in range(Nz):
            n = idx(i, j)
            ri = r[i]
            in_winding = ri >= Rint_m

            k = (p["k_copper"] * p["fill_factor"] +
                 p["k_steel"] * (1 - p["fill_factor"])) if in_winding else p["k_steel"]
            Q = (Q_cu + Q_fe) if in_winding else 0.0

            if 0 < i < Nr - 1 and 0 < j < Nz - 1:
                A[n, idx(i + 1, j)] += k / dr**2 + k / (2 * ri * dr)
                A[n, idx(i - 1, j)] += k / dr**2 - k / (2 * ri * dr)
                A[n, idx(i, j + 1)] += k / dz**2
                A[n, idx(i, j - 1)] += k / dz**2
                A[n, n] -= (2 * k / dr**2 + 2 * k / dz**2)
                b[n] = -Q
            elif i == Nr - 1 and 0 < j < Nz - 1:
                h = p["h_conv"]
                A[n, n] = -(k / dr + h)
                A[n, idx(i - 1, j)] = k / dr
                b[n] = -h * p["T_ambient"]
            elif i == 0 and 0 < j < Nz - 1:
                A[n, n] = -k / dr
                A[n, idx(i + 1, j)] = k / dr
                b[n] = 0.0
            elif j == 0:
                A[n, n] = -k / dz
                A[n, idx(i, j + 1)] = k / dz
                b[n] = 0.0
            elif j == Nz - 1:
                A[n, n] = -(k / dz + p["h_conv"])
                A[n, idx(i, j - 1)] = k / dz
                b[n] = -p["h_conv"] * p["T_ambient"]

    T_vec = spsolve(A.tocsr(), b)
    T_2d = T_vec.reshape(Nr, Nz)
    T_max = float(np.max(T_2d))

    i_rint = int((Rint_m - r_min) / dr)
    T_max_winding = float(np.max(T_2d[i_rint:, :]))

    return {"T_max_winding": T_max_winding, "T_max": T_max,
            "T_2d": T_2d, "r": r, "z": z}


def estimate_motor_rpm(torque_Nm, power_W):
    """Estimate motor RPM from power and torque: P = T*ω"""
    if torque_Nm <= 0:
        return 0
    rpm = (power_W / torque_Nm) * 60 / (2 * np.pi)
    return rpm


def calculate_thermal_expansion(p, thermal_result):
    """
    Calculate dimensional changes due to thermal expansion.

    Returns:
    - expansion_clearance: how much clearance is consumed by expansion (mm)
    - rotor_radial_growth: radial growth of rotor at hot spot (mm)
    - stator_bore_shrinkage: effective shrinkage of stator bore due to CTE difference (mm)
    """
    T_max = thermal_result["T_max"]
    T_ambient = p["T_ambient"]
    delta_T = T_max - T_ambient

    # Rotor (mostly steel with magnets) expands from center
    Rrotor_ext = p["Rrotor_ext"]
    CTE_rotor = MATERIAL_PROPS["steel_M330"]["CTE"]  # Dominated by steel
    rotor_radial_growth = Rrotor_ext * CTE_rotor * delta_T

    # Stator bore (steel) contracts relative to rotor
    Rint = p["Rint"]
    CTE_stator = MATERIAL_PROPS["steel_M330"]["CTE"]
    stator_bore_change = Rint * CTE_stator * delta_T  # Bore expands outward

    # Air gap change (rotor grows outward, stator grows inward, net gap reduction)
    aire_gap_reduction = rotor_radial_growth - (stator_bore_change * -1)  # Simplification: mostly rotor growth
    # More accurate: rotor grows outward, stator bore grows inward (both reduce gap)
    air_gap_reduction_effective = rotor_radial_growth

    # Safety check: ensure gap doesn't close completely
    original_gap = 0.5  # mm
    final_gap = original_gap - air_gap_reduction_effective

    return {
        "delta_T": delta_T,
        "rotor_radial_growth_mm": rotor_radial_growth,
        "stator_bore_expansion_mm": stator_bore_change,
        "air_gap_reduction_mm": air_gap_reduction_effective,
        "final_air_gap_mm": final_gap,
        "gap_safety_ok": final_gap > 0.1,  # Require at least 0.1mm clearance
    }


def calculate_mechanical_loads(p, mag_results, thermal_result):
    """
    Calculate mechanical forces and loads on motor components.

    Forces modeled:
    1. Centrifugal force on rotor (from rotation)
    2. Radial magnetic force (attraction between stator and rotor)
    3. Forces on magnets from magnetic field
    4. Stator tooth forces

    Returns stress and deflection estimates.
    """
    # Estimate operating RPM
    power_W = p["voltage_dc"] * p["I_continuous"]
    torque_Nm = mag_results["Tem_avg"]
    rpm = estimate_motor_rpm(torque_Nm, power_W)
    omega = rpm * 2 * np.pi / 60  # rad/s

    # Rotor parameters
    Rrotor_ext = p["Rrotor_ext"] * 1e-3  # meter
    Rshaft = p["Rshaft"] * 1e-3
    L_motor = p["motor_length"] * 1e-3
    pole_pairs = p["pole_pairs"]
    poles = pole_pairs * 2

    # Mass of rotor (simplified as hollow cylinder)
    rotor_volume = np.pi * (Rrotor_ext**2 - Rshaft**2) * L_motor
    rotor_mass = rotor_volume * MATERIAL_PROPS["steel_M330"]["density"]

    # Centrifugal stress at rotor rim (hoop stress): σ = ρ * ω² * r²
    rho_rotor = MATERIAL_PROPS["steel_M330"]["density"]
    centrifugal_stress = rho_rotor * omega**2 * Rrotor_ext**2
    centrifugal_stress_pa = centrifugal_stress  # Already in Pa

    # Rotor radial deflection under centrifugal load (hollow cylinder)
    # δ = ρ * ω² * r³ / (3 * E) for thick walI
    E_steel = MATERIAL_PROPS["steel_M330"]["E_modulus"]
    wall_thickness = Rrotor_ext - Rshaft
    centrifugal_deflection = (rho_rotor * omega**2 * Rrotor_ext**3) / (3 * E_steel)

    # Radial magnetic force (Maxwell stress on rotor surface)
    # F_radial ≈ (B²/2μ₀) * Area, B from magnets at air gap
    Br = p["magnet_Br"]
    B_airgap = Br * 0.7  # Approximate field in air gap (70% of remanence)
    mu_0 = 4 * np.pi * 1e-7
    magnetic_pressure = (B_airgap**2) / (2 * mu_0)  # Pa
    rotor_outer_area = 2 * np.pi * Rrotor_ext * L_motor
    radial_magnetic_force = magnetic_pressure * rotor_outer_area * poles  # N

    # Radial stress from magnetic pull on rotor
    rotor_cross_section = 2 * wall_thickness * L_motor
    radial_magnetic_stress = radial_magnetic_force / rotor_cross_section

    # Total hoop stress in rotor rim
    total_hoop_stress = centrifugal_stress_pa + radial_magnetic_stress

    # Shaft bending: magnetic force acts like distributed load on shaft
    # Simplified as cantilever: δ = F*L³/(3*E*I) for solid circular shaft
    I_shaft = np.pi * Rshaft**4 / 4  # Second moment for solid cylinder
    F_magnetic_total = radial_magnetic_force
    shaft_deflection = (F_magnetic_total * L_motor**3) / (3 * E_steel * I_shaft + 1e-20)

    # Stator tooth stress (winding in slots creates radial force)
    # Simplified: F ≈ 0.5 * B² * A / μ₀
    Rint = p["Rint"] * 1e-3
    stator_bore_area = (2 * np.pi * Rint * L_motor) / p["stator_slots"]
    tooth_stress = (B_airgap**2 * stator_bore_area) / (2 * mu_0 * wall_thickness)

    # Magnet compression stress (magnets held in by centrifugal + magnetic forces)
    magnet_thickness = p["magnet_thickness"] * 1e-3
    magnet_radial_mass = magnet_thickness * 2 * np.pi * Rrotor_ext * L_motor * MATERIAL_PROPS["magnet_N45SH"]["density"]
    magnet_centrifugal_stress = (magnet_radial_mass / poles) * omega**2 * Rrotor_ext / (magnet_thickness * L_motor)

    return {
        "rpm": rpm,
        "omega_rad_s": omega,
        "rotor_mass_kg": rotor_mass,
        "centrifugal_stress_pa": centrifugal_stress_pa,
        "centrifugal_deflection_mm": centrifugal_deflection * 1e3,
        "radial_magnetic_stress_pa": radial_magnetic_stress,
        "radial_magnetic_force_N": radial_magnetic_force,
        "total_hoop_stress_pa": total_hoop_stress,
        "shaft_deflection_mm": shaft_deflection * 1e3,
        "tooth_stress_pa": tooth_stress,
        "magnet_compression_stress_pa": magnet_centrifugal_stress,
    }


def check_mechanical_safety(p, mech_loads):
    """
    Check if mechanical loads are within safe limits.
    Returns: (is_safe, limiting_factor, margin)
    """
    # Rotor hoop stress (steel limit)
    rotor_yield = MATERIAL_PROPS["steel_M330"]["yield_strength"]
    rotor_sf = MATERIAL_PROPS["steel_M330"]["max_stress_safety_factor"]
    rotor_max_stress = rotor_yield / rotor_sf
    rotor_stress_margin = (rotor_max_stress - mech_loads["total_hoop_stress_pa"]) / rotor_max_stress

    # Magnet compression (limited)
    magnet_max = MATERIAL_PROPS["magnet_N45SH"]["max_compression_stress"]
    magnet_margin = (magnet_max - mech_loads["magnet_compression_stress_pa"]) / magnet_max

    # Geometric clearance (rotor deflection shouldn't cause contact)
    max_deflection_allowed = 0.1  # mm
    deflection_margin = (max_deflection_allowed - mech_loads["shaft_deflection_mm"]) / max_deflection_allowed

    margins = {
        "rotor_hoop": rotor_stress_margin,
        "magnet_compression": magnet_margin,
        "deflection": deflection_margin,
    }

    min_margin = min(margins.values())
    is_safe = min_margin > 0

    limiting_factor = [k for k, v in margins.items() if v == min_margin][0]

    return {
        "is_safe": is_safe,
        "limiting_factor": limiting_factor,
        "min_margin": min_margin,
        "margins": margins,
        "rotor_hoop_stress_pa": mech_loads["total_hoop_stress_pa"],
        "rotor_hoop_limit_pa": rotor_max_stress,
        "magnet_compression_stress_pa": mech_loads["magnet_compression_stress_pa"],
        "magnet_compression_limit_pa": magnet_max,
        "shaft_deflection_mm": mech_loads["shaft_deflection_mm"],
        "deflection_limit_mm": max_deflection_allowed,
    }



def calculate_axial_load(p, mag_results, thrust_force_N=5.0):
    """
    Model axial load from propeller thrust pulling the shaft upward.

    Forces modeled:
    1. Direct shaft bending/compression from axial thrust
    2. Clamping force on rotor/magnets
    3. Bearing stress

    Args:
        thrust_force_N: Propeller thrust pulling shaft (positive = upward, typical 3-10N for small drones)
    """
    Rshaft = p["Rshaft"] * 1e-3  # meter
    L_motor = p["motor_length"] * 1e-3
    E_steel = MATERIAL_PROPS["steel_M330"]["E_modulus"]

    # Shaft cross-section area
    A_shaft = np.pi * Rshaft**2

    # Direct tensile stress on shaft
    tensile_stress = thrust_force_N / A_shaft

    # Bending stress if thrust is off-center (assume mounted at one end)
    # M = F * L_moment_arm, with small offset
    moment_arm = Rshaft * 0.5  # Small bending moment from off-center load
    bending_moment = thrust_force_N * moment_arm
    I_shaft = np.pi * Rshaft**4 / 4
    bending_stress = (bending_moment * Rshaft) / (I_shaft + 1e-20)

    # Combined stress (tensile + bending)
    combined_axial_stress = tensile_stress + bending_stress

    # Shaft deflection under axial load (compression)
    axial_deflection = (thrust_force_N * L_motor) / (E_steel * A_shaft + 1e-20)

    # Clamping force on rotor - distributes thrust to magnets
    # Assume thrust clamping increases magnet compression
    Rrotor_ext = p["Rrotor_ext"] * 1e-3
    magnet_thickness = p["magnet_thickness"] * 1e-3
    poles = p["pole_pairs"] * 2

    # Clamping stress on magnets (per pole)
    clamping_stress_per_pole = thrust_force_N / (magnet_thickness * L_motor)
    # Distributed to all poles
    magnet_clamping_stress = clamping_stress_per_pole / poles

    return {
        "thrust_force_N": thrust_force_N,
        "tensile_stress_pa": tensile_stress,
        "bending_stress_pa": bending_stress,
        "combined_axial_stress_pa": combined_axial_stress,
        "axial_deflection_mm": axial_deflection * 1e3,
        "magnet_clamping_stress_pa": magnet_clamping_stress,
    }


def run_coupled_magnetomechanical(p, winding, thrust_force_N=5.0, max_iterations=3):
    """
    Iteratively couple magnetics and mechanical analysis.

    1. Run magnetics with baseline air gap
    2. Calculate mechanical loads and deflection
    3. Update air gap based on radial deflection
    4. Re-run magnetics with adjusted air gap
    5. Iterate until convergence

    Returns: (mag_results, mech_loads, mech_safety, axial_loads)
    """
    air_gap_adjustment = 0.0

    for iteration in range(max_iterations):
        # Step 1: Magnetics with current air gap
        mag_results = run_magnetics_pyleecan(p, winding, air_gap_adjustment=air_gap_adjustment, verbose=False)

        # Step 2: Mechanical loads
        mech_loads = calculate_mechanical_loads(p, mag_results, {"T_max": 0})  # Dummy thermal for now

        # Step 3: Axial load analysis
        axial_loads = calculate_axial_load(p, mag_results, thrust_force_N=thrust_force_N)

        # Step 4: Update air gap based on radial deflection
        new_air_gap_adjustment = mech_loads["centrifugal_deflection_mm"]

        # Check convergence
        if abs(new_air_gap_adjustment - air_gap_adjustment) < 0.001:  # 1 micron convergence
            break

        air_gap_adjustment = new_air_gap_adjustment * 0.5  # Dampen iteration for stability

    # Final mechanical safety check
    mech_safety = check_mechanical_safety(p, mech_loads)

    return mag_results, mech_loads, mech_safety, axial_loads


def evaluate_design(p, verbose=False):
    """Evaluate a complete motor design"""
    try:
        # Apply constraints
        # Force stator_slots to be integer
        p["stator_slots"] = int(round(p["stator_slots"]))

        # Enforce: Rrotor_ext = Rint - 0.5 mm (0.5mm air gap)
        p["Rrotor_ext"] = p["Rint"] - 0.5

        # Validation checks
        if p["stator_slots"] < 9 or p["stator_slots"] > 24:
            return None
        if p["pole_pairs"] < 4 or p["pole_pairs"] > 12:
            return None
        if p["Rrotor_ext"] <= p["Rshaft"]:
            return None
        if p["Rrotor_ext"] >= p["Rint"]:
            return None

        winding = compute_winding_params(p)

        # Skip impossible designs
        if winding["turns_per_slot"] < 1:
            return None

        # NEW: Run coupled magnetomechanical analysis (with propeller thrust force)
        thrust_force_N = 5.0  # Typical propeller thrust for small drone (N)
        mag_results, mech_loads, mech_safety, axial_loads = run_coupled_magnetomechanical(
            p, winding, thrust_force_N=thrust_force_N, max_iterations=3
        )

        thermal = run_thermal_fem(p, winding)

        # NEW: Thermal expansion analysis
        thermal_expansion = calculate_thermal_expansion(p, thermal)

        electrical_power = p["voltage_dc"] * p["I_continuous"]
        copper_loss = winding["P_cu_cont_W"]
        mechanical_power = electrical_power - copper_loss
        torque = mag_results["Tem_avg"]

        magnet_safe = thermal["T_max"] < MAGNET_TEMP_LIMIT
        temp_margin = MAGNET_TEMP_LIMIT - thermal["T_max"]

        # NEW: Safety checks for mechanical and thermal expansion
        thermally_safe = thermal_expansion["gap_safety_ok"]
        mechanically_safe = mech_safety["is_safe"]

        # NEW: Check axial load safety
        axial_stress_limit = MATERIAL_PROPS["steel_M330"]["yield_strength"] / 3.0  # Conservative limit
        axial_safe = axial_loads["combined_axial_stress_pa"] < axial_stress_limit

        overall_safe = magnet_safe and thermally_safe and mechanically_safe and axial_safe

        results = {
            "winding": winding,
            "mag_results": mag_results,
            "thermal": thermal,
            "thermal_expansion": thermal_expansion,
            "mech_loads": mech_loads,
            "mech_safety": mech_safety,
            "axial_loads": axial_loads,
            "electrical_power_W": electrical_power,
            "copper_loss_W": copper_loss,
            "mechanical_power_W": mechanical_power,
            "torque_Nm": torque,
            "T_max_winding": thermal["T_max_winding"],
            "T_max": thermal["T_max"],
            "magnet_safe": magnet_safe,
            "thermally_safe": thermally_safe,
            "mechanically_safe": mechanically_safe,
            "axial_safe": axial_safe,
            "overall_safe": overall_safe,
            "temp_margin": temp_margin,
            "mechanical_margin": mech_safety["min_margin"],
            "efficiency": mechanical_power / electrical_power if electrical_power > 0 else 0,
            "params": deepcopy(p),
        }

        if verbose:
            status_mag = "✓" if magnet_safe else "✗"
            status_therm = "✓" if thermally_safe else "✗"
            status_mech = "✓" if mechanically_safe else "✗"
            status_axial = "✓" if axial_safe else "✗"
            print(f"      {status_mag}{status_therm}{status_mech}{status_axial} T:{torque:.2f}Nm "
                  f"T_max:{thermal['T_max']:.0f}°C Gap:{thermal_expansion['final_air_gap_mm']:.3f}mm "
                  f"Hoop:{mech_loads['total_hoop_stress_pa']/1e6:.0f}MPa "
                  f"Axial:{axial_loads['combined_axial_stress_pa']/1e6:.0f}MPa "
                  f"eff:{results['efficiency']*100:.0f}%", flush=True)

        return results

    except Exception as e:
        if verbose:
            print(f"      ✗ Error: {type(e).__name__}", flush=True)
        return None


def objective_score(eval_result, target_power, require_safe=True):
    """
    Calculate optimization score. Lower is better.

    Hard constraints:
    - Magnet temperature < 120°C
    - Thermal expansion doesn't close air gap
    - Mechanical stresses within limits
    - Axial loads within limits

    Soft objectives (in order of priority):
    1. Safety margins (HEAVILY REWARDED) - robust designs preferred
    2. Power output matching target
    3. Efficiency
    """
    if eval_result is None:
        return 1e10

    # Hard constraint: magnet safe
    if require_safe and not eval_result["magnet_safe"]:
        excess_temp = eval_result["T_max"] - MAGNET_TEMP_LIMIT
        return 1e8 + excess_temp * 10000

    # Hard constraint: thermal expansion safe
    if require_safe and not eval_result["thermally_safe"]:
        gap = eval_result["thermal_expansion"]["final_air_gap_mm"]
        return 1e8 + (0.1 - gap) * 1e6 if gap < 0 else 1e8 + 1000

    # Hard constraint: mechanical safe
    if require_safe and not eval_result["mechanically_safe"]:
        margin = eval_result["mech_safety"]["min_margin"]
        return 1e8 + (abs(margin) * 1e6)

    # Hard constraint: axial safe
    if require_safe and not eval_result["axial_safe"]:
        axial_stress_limit = MATERIAL_PROPS["steel_M330"]["yield_strength"] / 3.0
        excess_stress = eval_result["axial_loads"]["combined_axial_stress_pa"] - axial_stress_limit
        return 1e8 + excess_stress * 100

    # ===== SOFT OBJECTIVES FOR SAFE DESIGNS =====

    # 1. SAFETY MARGIN REWARDS (Primary optimization goal) - HEAVILY WEIGHTED
    # Incentivize designs with large safety margins
    temp_margin_pct = eval_result["temp_margin"] / MAGNET_TEMP_LIMIT * 100
    mech_margin_pct = max(0, eval_result["mech_safety"]["min_margin"] * 100)

    # Strong bonus for good margins (designs that are well-away from limits)
    # Quadratic penalty for small margins to heavily discourage marginal designs
    temp_margin_reward = -(10 * temp_margin_pct**1.5)  # Negative = good, exponential reward
    mech_margin_reward = -(50 * mech_margin_pct**2.0)  # Quadratic reward for mechanical margin

    # Air gap margin bonus
    air_gap_current = eval_result["thermal_expansion"]["final_air_gap_mm"]
    gap_margin = air_gap_current - 0.1  # Buffer above minimum
    air_gap_reward = -(100 * max(0, gap_margin)**2)  # Reward for large gap margin

    # 2. Power matching (Secondary objective)
    power_diff = abs(eval_result["mechanical_power_W"] - target_power)

    # 3. Efficiency (Tertiary objective)
    efficiency_bonus = eval_result["efficiency"] * 100

    # Combined score
    safety_score = temp_margin_reward + mech_margin_reward + air_gap_reward
    power_score = power_diff * 1.0  # Linear power matching (lower weight than safety)
    efficiency_score = -efficiency_bonus * 5.0  # Small bonus for efficiency

    score = safety_score + power_score + efficiency_score

    return score


def iterative_deepening_optimize(base_params, target_power):
    """
    Iterative deepening optimization:
    1. Start with coarse grid search
    2. Progressively refine around best solution
    3. Each refinement: zoom in and increase precision
    """
    print("\n" + "="*80)
    print("ITERATIVE DEEPENING MOTOR DESIGN OPTIMIZER")
    print("="*80)
    print(f"\nTarget mechanical power: {target_power:.0f} W")
    print(f"Magnet temperature limit: {MAGNET_TEMP_LIMIT}°C")
    print(f"Safety margin: {TEMP_SAFETY_MARGIN}°C\n")

    best_result = None
    best_score = 1e10
    best_params = None
    all_evaluations = []

    # Initialize refinement regions (expand from base parameters)
    refinement_regions = {
        key: {"bounds": VARIABLE_PARAMS[key], "divisions": INITIAL_DIVISION_COUNT}
        for key in VARIABLE_PARAMS
    }

    for refinement_level in range(MAX_REFINEMENT_ITERATIONS):
        print(f"\n{'='*80}")
        print(f"REFINEMENT LEVEL {refinement_level + 1}/{MAX_REFINEMENT_ITERATIONS}")
        print(f"{'='*80}")

        # Build grid of parameter combinations
        param_lists = {}
        for key in VARIABLE_PARAMS:
            bounds = refinement_regions[key]["bounds"]
            divisions = refinement_regions[key]["divisions"]
            param_lists[key] = np.linspace(bounds[0], bounds[1], divisions)

        # For integer parameters, round to valid values
        if "pole_pairs" in param_lists:
            param_lists["pole_pairs"] = np.unique(np.round(param_lists["pole_pairs"])).astype(int)
            param_lists["pole_pairs"] = param_lists["pole_pairs"][
                (param_lists["pole_pairs"] >= 4) & (param_lists["pole_pairs"] <= 12)
            ]
        if "stator_slots" in param_lists:
            param_lists["stator_slots"] = np.unique(np.round(param_lists["stator_slots"])).astype(int)
            param_lists["stator_slots"] = param_lists["stator_slots"][
                (param_lists["stator_slots"] >= 9) & (param_lists["stator_slots"] <= 24)
            ]

        # Estimate total evaluations
        total_evals = 1
        for v in param_lists.values():
            total_evals *= len(v)
        print(f"Evaluating ~{min(total_evals, 1000)} designs...")

        # Limit evaluations if too many
        if total_evals > 1000:
            # Random sampling of parameter space
            keys = list(param_lists.keys())
            samples_per_param = int(1000 ** (1 / len(keys))) + 1
            param_lists = {
                key: np.random.choice(np.linspace(
                    VARIABLE_PARAMS[key][0], VARIABLE_PARAMS[key][1], 50
                ), min(samples_per_param, len(np.linspace(
                    VARIABLE_PARAMS[key][0], VARIABLE_PARAMS[key][1], 50
                ))))
                for key in keys
            }

        # Generate combinations
        import itertools
        combinations = list(itertools.product(*[
            enumerate(param_lists[key]) for key in param_lists.keys()
        ]))

        progress_interval = max(1, len(combinations) // 20)

        for eval_idx, combo in enumerate(combinations):
            if eval_idx % progress_interval == 0:
                pct = 100 * eval_idx / len(combinations)
                print(f"  [{pct:5.1f}%] ", end="", flush=True)

            p = deepcopy(base_params)
            for (_, key_values) in zip(range(len(VARIABLE_PARAMS)), combo):
                idx, val = key_values
                key = list(param_lists.keys())[eval_idx % len(param_lists)]
                for k, (i, v) in zip(param_lists.keys(), combo):
                    p[k] = v

            eval_result = evaluate_design(p, verbose=False)

            if eval_result is not None:
                score = objective_score(eval_result, target_power, require_safe=True)
                all_evaluations.append((p, eval_result, score))

                # Update best
                if score < best_score:
                    best_score = score
                    best_result = eval_result
                    best_params = deepcopy(p)
                    print(f"\n    NEW BEST: Score={score:.1f}, T_max={eval_result['T_max']:.0f}°C, "
                          f"P_mech={eval_result['mechanical_power_W']:.0f}W", flush=True)

            if eval_idx % progress_interval == 0:
                if best_result:
                    status = "✓" if best_result["magnet_safe"] else "✗"
                    print(f"{status} Best: {best_score:.1f}", flush=True)

        print(f"\n  Refinement {refinement_level + 1} complete.")

        if best_params is not None:
            # Refine around best solution
            refinement_factor = 0.4  # Zoom in to 40% of previous region
            new_divisions = refinement_regions[list(VARIABLE_PARAMS.keys())[0]]["divisions"] + 2

            for key in VARIABLE_PARAMS:
                old_bounds = refinement_regions[key]["bounds"]
                center = best_params[key]
                range_size = (old_bounds[1] - old_bounds[0]) * refinement_factor
                new_bounds = (
                    max(VARIABLE_PARAMS[key][0], center - range_size/2),
                    min(VARIABLE_PARAMS[key][1], center + range_size/2)
                )
                refinement_regions[key]["bounds"] = new_bounds
                refinement_regions[key]["divisions"] = new_divisions

            print(f"  Refined search region. Next divisions: {new_divisions}")

    print("\n" + "="*80)
    print("OPTIMIZATION COMPLETE")
    print("="*80)

    if best_result is None:
        print("\nERROR: No valid designs found!")
        return None

    print(f"\n✓ BEST DESIGN FOUND:")
    print(f"  Thermal Safety:")
    print(f"    Max temperature: {best_result['T_max']:.1f}°C (limit: {MAGNET_TEMP_LIMIT}°C)")
    print(f"    Temperature margin: {best_result['temp_margin']:.1f}°C")
    print(f"    Winding temperature: {best_result['T_max_winding']:.1f}°C")
    print(f"    Magnet safe: {'✓ YES' if best_result['magnet_safe'] else '✗ NO'}")

    print(f"\n  Thermal Expansion:")
    print(f"    Temperature rise: {best_result['thermal_expansion']['delta_T']:.1f}°C")
    print(f"    Rotor radial growth: {best_result['thermal_expansion']['rotor_radial_growth_mm']:.4f} mm")
    print(f"    Air gap reduction: {best_result['thermal_expansion']['air_gap_reduction_mm']:.4f} mm")
    print(f"    Final air gap: {best_result['thermal_expansion']['final_air_gap_mm']:.4f} mm (min: 0.1mm)")
    print(f"    Expansion safe: {'✓ YES' if best_result['thermally_safe'] else '✗ NO'}")

    print(f"\n  Mechanical Analysis:")
    print(f"    Operating RPM: {best_result['mech_loads']['rpm']:.0f}")
    print(f"    Rotor mass: {best_result['mech_loads']['rotor_mass_kg']:.3f} kg")
    print(f"    Centrifugal stress: {best_result['mech_loads']['centrifugal_stress_pa']/1e6:.1f} MPa")
    print(f"    Magnetic radial force: {best_result['mech_loads']['radial_magnetic_force_N']:.1f} N")
    print(f"    Total rotor hoop stress: {best_result['mech_loads']['total_hoop_stress_pa']/1e6:.1f} MPa")
    print(f"      (limit: {best_result['mech_safety']['rotor_hoop_limit_pa']/1e6:.1f} MPa)")
    print(f"    Magnet compression stress: {best_result['mech_loads']['magnet_compression_stress_pa']/1e6:.1f} MPa")
    print(f"      (limit: {best_result['mech_safety']['magnet_compression_limit_pa']/1e6:.1f} MPa)")
    print(f"    Shaft deflection: {best_result['mech_loads']['shaft_deflection_mm']:.4f} mm")
    print(f"      (limit: {best_result['mech_safety']['deflection_limit_mm']:.2f} mm)")
    print(f"    Limiting factor: {best_result['mech_safety']['limiting_factor']}")
    print(f"    Mechanical margin: {best_result['mech_safety']['min_margin']*100:.1f}%")
    print(f"    Mechanical safe: {'✓ YES' if best_result['mechanically_safe'] else '✗ NO'}")

    print(f"\n  Axial Load Analysis (Propeller Thrust):")
    print(f"    Thrust force: {best_result['axial_loads']['thrust_force_N']:.1f} N")
    print(f"    Shaft tensile stress: {best_result['axial_loads']['tensile_stress_pa']/1e6:.1f} MPa")
    print(f"    Shaft bending stress: {best_result['axial_loads']['bending_stress_pa']/1e6:.1f} MPa")
    print(f"    Combined axial stress: {best_result['axial_loads']['combined_axial_stress_pa']/1e6:.1f} MPa")
    print(f"      (limit: {MATERIAL_PROPS['steel_M330']['yield_strength']/(3.0*1e6):.1f} MPa)")
    print(f"    Axial deflection: {best_result['axial_loads']['axial_deflection_mm']:.4f} mm")
    print(f"    Magnet clamping stress: {best_result['axial_loads']['magnet_clamping_stress_pa']/1e6:.2f} MPa")
    print(f"    Axial safe: {'✓ YES' if best_result['axial_safe'] else '✗ NO'}")

    print(f"\n  Power:")
    print(f"    Electrical input: {best_result['electrical_power_W']:.1f} W")
    print(f"    Copper loss: {best_result['copper_loss_W']:.1f} W")
    print(f"    Mechanical output: {best_result['mechanical_power_W']:.1f} W")
    print(f"    Efficiency: {best_result['efficiency']*100:.1f}%")
    print(f"    Target power: {target_power:.1f} W")
    print(f"    Difference: {abs(best_result['mechanical_power_W'] - target_power):.1f} W")
    print(f"\n  Performance:")
    print(f"    Torque: {best_result['torque_Nm']:.3f} Nm")
    print(f"    Torque ripple: {best_result['mag_results']['Tem_rip']*100:.1f}%")
    print(f"\n  Design Parameters:")
    print(f"    Stator inner radius: {best_params['Rint']:.2f} mm")
    print(f"    Rotor outer radius: {best_params['Rrotor_ext']:.2f} mm (Rint - 0.5mm)")
    print(f"    Air gap: 0.5 mm")
    print(f"    Shaft radius: {best_params['Rshaft']:.2f} mm")
    print(f"    Magnet thickness: {best_params['magnet_thickness']:.2f} mm")
    print(f"    Motor length: {best_params['motor_length']:.2f} mm")
    print(f"    Pole pairs: {int(best_params['pole_pairs'])}")
    print(f"    Stator slots: {int(best_params['stator_slots'])}")
    print(f"    Magnet Br: {best_params['magnet_Br']:.3f} T")
    print(f"    Convection coeff: {best_params['h_conv']:.1f} W/m²K")
    print(f"    Slot opening: {best_params['slot_opening']:.2f} mm")

    return best_params, best_result


def build_and_export_stl(params, winding, output_suffix="optimized"):
    """Export STL files for optimized design"""
    if not HAS_CADQUERY:
        print("CadQuery not available, skipping STL export")
        return

    slots = params["stator_slots"]
    L = params["motor_length"]
    Rext = params["Rext"]
    Rint = params["Rint"]
    Rrot = params["Rrotor_ext"]
    Rsh = params["Rshaft"]
    t_mag = params["magnet_thickness"]
    poles = params["pole_pairs"] * 2
    slot_depth = (Rext - Rint) * 0.85
    slot_width = 2 * np.pi * Rint / slots * 0.45

    TOL = 0.05
    ANG_TOL = 0.1

    print("\nExporting STL files...")
    import math

    # Ensure integer values for slots and poles
    slots = int(slots)
    poles = int(poles)

    # Stator core
    stator_core = cq.Workplane("XY").circle(Rext).circle(Rint).extrude(L)
    for i in range(slots):
        ang = 360.0 * i / slots
        stator_core = (
            stator_core.faces(">Z").workplane()
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(Rint + slot_depth / 2, 0)
            .rect(slot_depth, slot_width)
            .cutThruAll()
        )
    stator_core.val().exportStl(f"stator_core_{output_suffix}.stl",
                                tolerance=TOL, angularTolerance=ANG_TOL)
    print(f"  ✓ stator_core_{output_suffix}.stl")

    # Windings
    winding_assy = Assembly()
    slot_depth_w = (Rext - Rint) * 0.75
    slot_width_w = 2 * np.pi * Rint / slots * 0.38
    for i in range(slots):
        ang = 360.0 * i / slots
        coil = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(Rint + slot_depth_w / 2, 0)
            .rect(slot_depth_w, slot_width_w)
            .extrude(L)
        )
        winding_assy.add(coil, name=f"coil_{i}", color=Color("orange"))
    winding_assy.toCompound().exportStl(
        f"windings_{output_suffix}.stl", tolerance=TOL, angularTolerance=ANG_TOL)
    print(f"  ✓ windings_{output_suffix}.stl")

    # Rotor core
    rotor_core = cq.Workplane("XY").circle(Rrot).circle(Rsh).extrude(L)
    rotor_core.val().exportStl(f"rotor_core_{output_suffix}.stl",
                               tolerance=TOL, angularTolerance=ANG_TOL)
    print(f"  ✓ rotor_core_{output_suffix}.stl")

    # Permanent magnets
    magnet_assy = Assembly()
    mag_w = 2 * np.pi * Rrot / poles * 0.85
    for i in range(poles):
        ang = 360.0 * i / poles
        mag_r = Rrot + t_mag / 2
        magnet = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(mag_r, 0)
            .rect(t_mag, mag_w)
            .extrude(L)
        )
        magnet_assy.add(magnet, name=f"mag_{i}",
                        color=Color("lightgray") if i % 2 == 0 else Color("black"))
    magnet_assy.toCompound().exportStl(
        f"magnets_{output_suffix}.stl", tolerance=TOL, angularTolerance=ANG_TOL)
    print(f"  ✓ magnets_{output_suffix}.stl")

    # Full assembly
    full_assy = Assembly(name="drone_motor_optimized")
    full_assy.add(stator_core, name="stator", color=Color("lightgray"))
    full_assy.add(rotor_core, name="rotor", color=Color("black"))

    for i in range(slots):
        ang = 360.0 * i / slots
        coil = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(Rint + slot_depth_w / 2, 0)
            .rect(slot_depth_w, slot_width_w)
            .extrude(L)
        )
        full_assy.add(coil, name=f"coil_{i}", color=Color("orange"))

    for i in range(poles):
        ang = 360.0 * i / poles
        mag_r = Rrot + t_mag / 2
        magnet = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(mag_r, 0)
            .rect(t_mag, mag_w)
            .extrude(L)
        )
        full_assy.add(magnet, name=f"mag_{i}",
                      color=Color("lightgray") if i % 2 == 0 else Color("black"))

    full_assy.toCompound().exportStl(
        f"motor_full_assembly_{output_suffix}.stl", tolerance=TOL, angularTolerance=ANG_TOL)
    print(f"  ✓ motor_full_assembly_{output_suffix}.stl")


def save_results(best_params, best_eval, output_suffix="optimized"):
    """Save results to JSON and PNG"""
    # JSON summary
    summary = {
        "optimization_metadata": {
            "refinement_iterations": MAX_REFINEMENT_ITERATIONS,
            "initial_divisions": INITIAL_DIVISION_COUNT,
            "magnet_temp_limit_C": MAGNET_TEMP_LIMIT,
            "safety_margin_C": TEMP_SAFETY_MARGIN,
        },
        "winding": {k: float(v) if isinstance(v, (int, float, np.number)) else v
                    for k, v in best_eval["winding"].items()},
        "magnetics": {
            "Tem_avg_Nm": float(best_eval["mag_results"]["Tem_avg"]),
            "Tem_rip_percent": float(best_eval["mag_results"]["Tem_rip"] * 100),
            "is_pyleecan_analysis": bool(best_eval["mag_results"].get("is_pyleecan", False)),
        },
        "thermal": {
            "T_max_winding_C": float(best_eval["T_max_winding"]),
            "T_max_overall_C": float(best_eval["T_max"]),
            "T_ambient_C": float(best_params["T_ambient"]),
            "magnet_safe": bool(best_eval["magnet_safe"]),
            "temp_margin_C": float(best_eval["temp_margin"]),
        },
        "power_analysis": {
            "voltage_dc_V": float(best_params["voltage_dc"]),
            "current_A": float(best_params["I_continuous"]),
            "electrical_power_W": float(best_eval["electrical_power_W"]),
            "copper_loss_W": float(best_eval["copper_loss_W"]),
            "mechanical_power_W": float(best_eval["mechanical_power_W"]),
            "efficiency_percent": float(best_eval["efficiency"] * 100),
        },
        "performance": {
            "torque_Nm": float(best_eval["torque_Nm"]),
            "torque_ripple_percent": float(best_eval["mag_results"]["Tem_rip"] * 100),
        },
        "thermal_expansion_analysis": {
            "delta_T_C": float(best_eval["thermal_expansion"]["delta_T"]),
            "rotor_radial_growth_mm": float(best_eval["thermal_expansion"]["rotor_radial_growth_mm"]),
            "stator_bore_expansion_mm": float(best_eval["thermal_expansion"]["stator_bore_expansion_mm"]),
            "air_gap_reduction_mm": float(best_eval["thermal_expansion"]["air_gap_reduction_mm"]),
            "final_air_gap_mm": float(best_eval["thermal_expansion"]["final_air_gap_mm"]),
            "gap_safety_ok": bool(best_eval["thermal_expansion"]["gap_safety_ok"]),
        },
        "mechanical_analysis": {
            "operating_rpm": float(best_eval["mech_loads"]["rpm"]),
            "rotor_mass_kg": float(best_eval["mech_loads"]["rotor_mass_kg"]),
            "centrifugal_stress_MPa": float(best_eval["mech_loads"]["centrifugal_stress_pa"] / 1e6),
            "centrifugal_deflection_mm": float(best_eval["mech_loads"]["centrifugal_deflection_mm"]),
            "radial_magnetic_force_N": float(best_eval["mech_loads"]["radial_magnetic_force_N"]),
            "radial_magnetic_stress_MPa": float(best_eval["mech_loads"]["radial_magnetic_stress_pa"] / 1e6),
            "total_hoop_stress_MPa": float(best_eval["mech_loads"]["total_hoop_stress_pa"] / 1e6),
            "shaft_deflection_mm": float(best_eval["mech_loads"]["shaft_deflection_mm"]),
            "magnet_compression_stress_MPa": float(best_eval["mech_loads"]["magnet_compression_stress_pa"] / 1e6),
        },
        "mechanical_safety": {
            "is_safe": bool(best_eval["mechanically_safe"]),
            "limiting_factor": str(best_eval["mech_safety"]["limiting_factor"]),
            "min_margin_percent": float(best_eval["mech_safety"]["min_margin"] * 100),
            "rotor_hoop_stress_vs_limit_MPa": {
                "stress": float(best_eval["mech_safety"]["rotor_hoop_stress_pa"] / 1e6),
                "limit": float(best_eval["mech_safety"]["rotor_hoop_limit_pa"] / 1e6),
            },
            "magnet_compression_vs_limit_MPa": {
                "stress": float(best_eval["mech_safety"]["magnet_compression_stress_pa"] / 1e6),
                "limit": float(best_eval["mech_safety"]["magnet_compression_limit_pa"] / 1e6),
            },
            "shaft_deflection_vs_limit_mm": {
                "deflection": float(best_eval["mech_safety"]["shaft_deflection_mm"]),
                "limit": float(best_eval["mech_safety"]["deflection_limit_mm"]),
            },
        },
        "axial_load_analysis": {
            "thrust_force_N": float(best_eval["axial_loads"]["thrust_force_N"]),
            "tensile_stress_MPa": float(best_eval["axial_loads"]["tensile_stress_pa"] / 1e6),
            "bending_stress_MPa": float(best_eval["axial_loads"]["bending_stress_pa"] / 1e6),
            "combined_axial_stress_MPa": float(best_eval["axial_loads"]["combined_axial_stress_pa"] / 1e6),
            "axial_stress_limit_MPa": float(MATERIAL_PROPS["steel_M330"]["yield_strength"] / (3.0 * 1e6)),
            "axial_deflection_mm": float(best_eval["axial_loads"]["axial_deflection_mm"]),
            "magnet_clamping_stress_MPa": float(best_eval["axial_loads"]["magnet_clamping_stress_pa"] / 1e6),
            "is_safe": bool(best_eval["axial_safe"]),
        },
        "design_parameters": {
            "Rext_mm": float(best_params["Rext"]),
            "Rint_mm": float(best_params["Rint"]),
            "Rrotor_ext_mm": float(best_params["Rrotor_ext"]),
            "Rshaft_mm": float(best_params["Rshaft"]),
            "magnet_thickness_mm": float(best_params["magnet_thickness"]),
            "motor_length_mm": float(best_params["motor_length"]),
            "pole_pairs": int(best_params["pole_pairs"]),
            "stator_slots": int(best_params["stator_slots"]),
            "magnet_Br_T": float(best_params["magnet_Br"]),
            "h_conv_W_m2K": float(best_params["h_conv"]),
            "slot_opening_mm": float(best_params["slot_opening"]),
            "fill_factor": float(best_params["fill_factor"]),
        }
    }

    with open(f"motor_design_results_{output_suffix}.json", "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\n  ✓ motor_design_results_{output_suffix}.json")

    # Thermal polar plot
    thermal = best_eval["thermal"]
    r = thermal["r"]
    T_2d = thermal["T_2d"]
    T_mid = T_2d[:, T_2d.shape[1] // 2]

    N_ang = 360
    angles = np.linspace(0, 2 * np.pi, N_ang)
    R_grid, A_grid = np.meshgrid(
        (r - r.min()) / (r.max() - r.min()),
        angles
    )
    T_polar = np.tile(T_mid, (N_ang, 1))

    fig1, ax1 = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(8, 8))
    pcm = ax1.pcolormesh(A_grid, R_grid, T_polar,
                         cmap="hot", shading="auto",
                         vmin=best_params["T_ambient"], vmax=thermal["T_max"])
    cbar = plt.colorbar(pcm, ax=ax1, label="Temperature [°C]", pad=0.1)
    ax1.set_title(f"Thermal Map (Optimized)\nT_max={thermal['T_max']:.1f}°C, "
                  f"Safe={'✓' if best_eval['magnet_safe'] else '✗'}",
                  va="bottom", pad=20, fontsize=12)
    ax1.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax1.set_yticklabels(["shaft", "", "bore", "", "Rext"], fontsize=9)
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f"thermal_polar_{output_suffix}.png", dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  ✓ thermal_polar_{output_suffix}.png")

    # Torque polar plot
    mag_results = best_eval["mag_results"]
    Tem_vals = np.array(mag_results["Tem_vals"])
    time_vec = np.array(mag_results["time_vec"])

    theta_norm = (time_vec - time_vec.min()) / (time_vec.max() - time_vec.min()) * 2 * np.pi
    theta_plot = np.append(theta_norm, theta_norm[0])
    torque_plot = np.append(Tem_vals, Tem_vals[0])

    fig2, ax2 = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(8, 8))
    ax2.plot(theta_plot, torque_plot, color="royalblue", linewidth=2)
    ax2.fill(theta_plot, torque_plot, alpha=0.2, color="royalblue")
    ax2.plot(theta_plot, np.full_like(torque_plot, mag_results["Tem_avg"]),
             "r--", linewidth=1.5, label=f"Mean: {mag_results['Tem_avg']:.3f} Nm")
    ax2.set_title(f"Torque Waveform (Optimized)\nRipple={mag_results['Tem_rip']*100:.1f}%",
                  va="bottom", pad=20, fontsize=12)
    ax2.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1), fontsize=10)
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f"torque_polar_{output_suffix}.png", dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  ✓ torque_polar_{output_suffix}.png")


# ============================================================
# MAIN ENTRY
# ============================================================

if __name__ == "__main__":
    BASE_PARAMS = {
        # FIXED PARAMETERS (cannot change)
        "voltage_dc": 60.2,
        "I_continuous": 160.0,
        "I_burst": 200.0,
        "wire_resistivity": 2.72e-8,
        "Rext": 25.5,
        "T_ambient": 45.0,
        "fill_factor": 0.5,

        # Material properties (FIXED)
        "steel_grade": "M330-35A",
        "k_copper": 380.0,
        "k_steel": 25.0,

        # VARIABLE PARAMETERS (optimized)
        "Rint": 40.5,
        "Rrotor_ext": 40.0,  # Will be overridden: Rrotor_ext = Rint - 0.5
        "Rshaft": 2.0,
        "magnet_thickness": 4.0,
        "motor_length": 55.0,
        "pole_pairs": 7,
        "stator_slots": 12,  # Will be forced to integer
        "magnet_Br": 1.2,
        "h_conv": 40.0,
        "slot_opening": 2.5,
    }

    # Calculate target power (electrical input - aim for high efficiency)
    electrical_power = BASE_PARAMS["voltage_dc"] * BASE_PARAMS["I_continuous"]
    target_power = electrical_power * 0.85  # Target 85% efficiency

    print(f"\nInput electrical power: {electrical_power:.0f}W")
    print(f"Target mechanical power: {target_power:.0f}W (85% efficiency)")

    # Run optimization
    result = iterative_deepening_optimize(BASE_PARAMS, target_power)

    if result:
        best_params, best_eval = result

        print("\n" + "="*80)
        print("SAVING RESULTS")
        print("="*80)

        # Compute winding for geometry
        winding = compute_winding_params(best_params)

        # Export STLs
        build_and_export_stl(best_params, winding, output_suffix="optimized")

        # Save JSON and PNGs
        save_results(best_params, best_eval, output_suffix="optimized")

        print("\n" + "="*80)
        print("✓ OPTIMIZATION COMPLETE!")
        print("="*80)
        print("\nAll output files saved with '_optimized' suffix:")
        print("  - motor_design_results_optimized.json")
        print("  - motor_full_assembly_optimized.stl")
        print("  - stator_core_optimized.stl / windings_optimized.stl")
        print("  - rotor_core_optimized.stl / magnets_optimized.stl")
        print("  - thermal_polar_optimized.png")
        print("  - torque_polar_optimized.png")
