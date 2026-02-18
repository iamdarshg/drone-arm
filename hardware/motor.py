# ============================================================
# HIGH-POWER BLDC MOTOR DESIGN & SIMULATION WORKFLOW
# Stages: 1) Motor definition  2) Magnetics (pyleecan+FEMM)
#         3) Thermal (scipy FEA)  4) STEP export (CadQuery)
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import spsolve
import cadquery as cq
from cadquery import Assembly, Color
import json

# ============================================================
# MOTOR PARAMETERS — Edit these for your design
# ============================================================
PARAMS = {
    # Electrical
    "voltage_dc"      : 60.2,    # 12S nominal [V]
    "I_continuous"    : 120.0,    # per-motor continuous [A]
    "I_burst"         : 180.0,   # per-motor burst [A]
    "pole_pairs"      : 7,       # p  → 14 poles (12N14P config)
    "stator_slots"    : 12,

    # Geometry [mm]
    "Rext"            : 60.5,    # stator outer radius
    "Rint"            : 40.5,    # stator inner radius (bore)
    "Rrotor_ext"      : 40.0,    # rotor outer radius (airgap = Rint - Rrotor_ext)
    "Rshaft"          : 2.0,     # shaft radius
    "motor_length"    : 55.0,    # active stack length
    "magnet_thickness": 4.0,     # PM thickness
    "slot_opening"    : 2.5,     # slot opening width

    # Materials
    "fill_factor"     : 0.5,    # copper fill in slot
    "wire_resistivity": 2.72e-8, # copper [Ω·m]
    "magnet_Br"       : 1.2,     # N45SH remanence [T]
    "steel_grade"     : "M330-35A",

    # Thermal
    "T_ambient"       : 45.0,    # [°C] — Bengaluru hot day
    "k_copper"        : 380.0,   # W/m·K
    "k_steel"         : 25.0,    # W/m·K
    "h_conv"          : 40.0,    # W/m²·K — forced air from props
}

# ============================================================
# STAGE 1 — WINDING PARAMETERS
# ============================================================
def compute_winding_params(p):
    Rext       = p["Rext"] * 1e-3
    Rint       = p["Rint"] * 1e-3
    L          = p["motor_length"] * 1e-3
    slots      = p["stator_slots"]

    tooth_angle  = 2 * np.pi / slots
    slot_area    = 0.5 * (Rext**2 - Rint**2) * tooth_angle * p["fill_factor"]
    J_target     = 8e6
    wire_area    = p["I_continuous"] / J_target
    turns_per_slot = max(1, int(slot_area / wire_area))
    mean_turn_len  = np.pi * (Rext + Rint) + 2 * L
    total_turns    = turns_per_slot * slots
    R_dc           = p["wire_resistivity"] * mean_turn_len * total_turns / (wire_area * slots / 3)
    P_cu_cont      = 3 * (p["I_continuous"]  / np.sqrt(3))**2 * R_dc
    P_cu_burst     = 3 * (p["I_burst"]       / np.sqrt(3))**2 * R_dc

    results = {
        "slot_area_mm2"    : slot_area * 1e6,
        "wire_area_mm2"    : wire_area * 1e6,
        "turns_per_slot"   : turns_per_slot,
        "R_dc_ohm"         : R_dc,
        "P_cu_cont_W"      : P_cu_cont,
        "P_cu_burst_W"     : P_cu_burst,
        "wire_diameter_mm" : 2 * np.sqrt(wire_area / np.pi) * 1e3,
    }
    print("\n=== Winding Parameters ===")
    for k, v in results.items():
        print(f"  {k:25s}: {v:.4f}")
    return results

winding = compute_winding_params(PARAMS)

# ============================================================
# STAGE 2 — MAGNETICS (pyleecan + FEMM, fixed imports)
# ============================================================
def run_magnetics(p, winding):
    try:
        from pyleecan.Classes.MachineSIPMSM  import MachineSIPMSM
        from pyleecan.Classes.LamSlotWind    import LamSlotWind
        from pyleecan.Classes.LamSlotLM      import LamSlotLM
        from pyleecan.Classes.SlotW29        import SlotW29
        from pyleecan.Classes.HoleM50        import HoleM50
        from pyleecan.Classes.Winding        import Winding   # ← fixed: no WindingDPNV
        from pyleecan.Classes.Simulation     import Simulation
        from pyleecan.Classes.MagFEMM        import MagFEMM
        from pyleecan.Classes.InputCurrent   import InputCurrent

        stator_slot = SlotW29(
            Zs = p["stator_slots"],
            H0 = 0.5e-3,
            H1 = 1.0e-3,
            H2 = (p["Rext"] - p["Rint"] - 2.5) * 1e-3,
            W0 = p["slot_opening"] * 1e-3,
            W1 = 3.5e-3,
            W2 = 4.5e-3,
        )

        # Winding defined directly on LamSlotWind — no WindingDPNV needed
        stator = LamSlotWind(
            Rint    = p["Rint"]         * 1e-3,
            Rext    = p["Rext"]         * 1e-3,
            L1      = p["motor_length"] * 1e-3,
            slot    = stator_slot,
            winding = Winding(
                qs      = 3,
                p       = p["pole_pairs"],
                Ntcoil  = winding["turns_per_slot"]
            ),
        )

        rotor_hole = HoleM50(
            Zh = p["pole_pairs"] * 2,
            W0 = 0.005, W1 = 0.004, W2 = 0.010,
            W3 = 0.002, W4 = 0.002,
            H0 = 0.001, H1 = 0.0015,
            H2 = p["magnet_thickness"] * 1e-3,
            H3 = 0.001, H4 = 0.0,
        )
        rotor = LamSlotLM(
            Rint = p["Rshaft"]      * 1e-3,
            Rext = p["Rrotor_ext"]  * 1e-3,
            L1   = p["motor_length"]* 1e-3,
            hole = [rotor_hole],
        )

        machine = MachineSIPMSM(stator=stator, rotor=rotor, name="custom_drone_motor")

        simu = Simulation(
            machine = machine,
            input   = InputCurrent(
                Id_ref = 0,
                Iq_ref = p["I_continuous"],
                Na_tot = 2048,
                Nt_tot = 32,
                felec  = 500.0,
            ),
            mag = MagFEMM(
                is_periodicity_a      = True,
                is_periodicity_t      = True,
                nb_worker             = 4,
                is_fast_draw          = True,
                is_sliding_band       = True,
                is_calc_torque_energy = True,
                T_mag                 = p["T_ambient"] + 25,
            ),
        )

        out      = simu.run()
        Tem_vals = out.mag.Tem.values
        Tem_avg  = float(np.mean(Tem_vals))
        Tem_rip  = float(out.mag.Tem_rip_norm)
        time_vec = out.mag.Tem.axes[0].values   # time axis

        print(f"\n=== Magnetics Results ===")
        print(f"  Average Torque  : {Tem_avg:.3f} Nm")
        print(f"  Torque Ripple   : {Tem_rip*100:.2f} %")

        return {"Tem_avg": Tem_avg, "Tem_rip": Tem_rip,
                "Tem_vals": Tem_vals, "time_vec": time_vec}

    except ImportError as e:
        print(f"[Magnetics] pyleecan/FEMM not available: {e}")
        print("[Magnetics] Using analytical fallback...")

        Rrotor = p["Rrotor_ext"] * 1e-3
        L      = p["motor_length"] * 1e-3
        p_     = p["pole_pairs"]
        N      = winding["turns_per_slot"] * p["stator_slots"]
        Ke     = (4/np.pi) * N * p["magnet_Br"] * Rrotor * L / p_
        Tem_avg = Ke * p["I_continuous"]

        # Synthetic torque waveform: mean + ripple harmonics
        Nt       = 256
        theta    = np.linspace(0, 2*np.pi, Nt)
        ripple   = 0.04 * Tem_avg * np.cos(p["stator_slots"] * theta)  # slot harmonic
        Tem_vals = Tem_avg + ripple
        print(f"  Estimated Torque: {Tem_avg:.3f} Nm")

        return {"Tem_avg": Tem_avg, "Tem_rip": 0.04,
                "Tem_vals": Tem_vals, "time_vec": theta}

mag_results = run_magnetics(PARAMS, winding)

# ============================================================
# STAGE 3 — THERMAL (2D axisymmetric FEM, scipy sparse)
# ============================================================
def run_thermal_fem(p, winding):
    Nr, Nz  = 40, 60
    r_min   = p["Rshaft"]      * 1e-3
    r_max   = p["Rext"]        * 1e-3
    z_max   = p["motor_length"]* 1e-3 / 2
    dr      = (r_max - r_min) / (Nr - 1)
    dz      =  z_max           / (Nz - 1)
    r       = np.linspace(r_min, r_max, Nr)
    z       = np.linspace(0,     z_max, Nz)

    N_nodes = Nr * Nz
    A       = lil_matrix((N_nodes, N_nodes))
    b       = np.zeros(N_nodes)

    def idx(i, j): return i * Nz + j

    slot_vol = (winding["slot_area_mm2"] * 1e-6 *
                p["motor_length"] * 1e-3 * p["stator_slots"])
    Q_cu     = winding["P_cu_cont_W"] / max(slot_vol, 1e-8)
    Q_fe     = Q_cu * 0.05
    Rint_m   = p["Rint"] * 1e-3

    for i in range(Nr):
        for j in range(Nz):
            n  = idx(i, j)
            ri = r[i]
            in_winding = ri >= Rint_m

            k = (p["k_copper"] * p["fill_factor"] +
                 p["k_steel"]  * (1 - p["fill_factor"])) if in_winding else p["k_steel"]
            Q = (Q_cu + Q_fe) if in_winding else 0.0

            if 0 < i < Nr-1 and 0 < j < Nz-1:
                A[n, idx(i+1, j)] +=  k/dr**2 + k/(2*ri*dr)
                A[n, idx(i-1, j)] +=  k/dr**2 - k/(2*ri*dr)
                A[n, idx(i,  j+1)] +=  k/dz**2
                A[n, idx(i,  j-1)] +=  k/dz**2
                A[n, n]            -= (2*k/dr**2 + 2*k/dz**2)
                b[n]                = -Q
            elif i == Nr-1 and 0 < j < Nz-1:
                h = p["h_conv"]
                A[n, n]            = -(k/dr + h)
                A[n, idx(i-1, j)]  =   k/dr
                b[n]               = -h * p["T_ambient"]
            elif i == 0 and 0 < j < Nz-1:
                A[n, n]            = -k/dr
                A[n, idx(i+1, j)]  =  k/dr
                b[n]               = 0.0
            elif j == 0:
                A[n, n]            = -k/dz
                A[n, idx(i, j+1)]  =  k/dz
                b[n]               = 0.0
            elif j == Nz-1:
                A[n, n]            = -(k/dz + p["h_conv"])
                A[n, idx(i, j-1)]  =  k/dz
                b[n]               = -p["h_conv"] * p["T_ambient"]

    T_vec = spsolve(A.tocsr(), b)
    T_2d  = T_vec.reshape(Nr, Nz)
    T_max = float(np.max(T_2d))

    i_rint = int((Rint_m - r_min) / dr)
    T_max_winding = float(np.max(T_2d[i_rint:, :]))

    print(f"\n=== Thermal Results ===")
    print(f"  Max winding temp : {T_max_winding:.1f} °C")
    print(f"  Max overall temp : {T_max:.1f} °C")
    print(f"  Magnet safe?     : {'YES' if T_max < 120 else 'WARNING — check magnet grade'}")

    return {"T_max_winding": T_max_winding, "T_max": T_max,
            "T_2d": T_2d, "r": r, "z": z}

thermal = run_thermal_fem(PARAMS, winding)

# ============================================================
# STAGE 4 — POLAR PLOT WRAPPER
# Maps both the thermal field and torque waveform to polar axes
# ============================================================
def polar_wrapper(thermal, mag_results, p):
    """
    Re-maps simulation outputs to polar coordinate plots.
    - Thermal: radial slices of T(r) swept around 2π → filled contour ring
    - Torque:  waveform plotted on a polar axis (angle vs. torque radius)
    """
    r       = thermal["r"]
    T_2d    = thermal["T_2d"]          # shape (Nr, Nz)
    T_mid   = T_2d[:, T_2d.shape[1]//2]  # mid-axial slice

    # ---- 1. Polar thermal map (rotational symmetry sweep) ----
    N_ang   = 360
    angles  = np.linspace(0, 2*np.pi, N_ang)
    R_grid, A_grid = np.meshgrid(
        (r - r.min()) / (r.max() - r.min()),   # normalised 0→1
        angles
    )
    T_polar = np.tile(T_mid, (N_ang, 1))       # same profile every angle

    fig1, ax1 = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(7, 7))
    pcm = ax1.pcolormesh(A_grid, R_grid, T_polar,
                         cmap="hot", shading="auto",
                         vmin=p["T_ambient"], vmax=thermal["T_max"])
    plt.colorbar(pcm, ax=ax1, label="Temperature [°C]", pad=0.1)
    ax1.set_title("Motor Thermal Map — Polar View\n(mid-axial slice, rotational symmetry)",
                  va="bottom", pad=20)
    ax1.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
    ax1.set_yticklabels(["shaft", "", "bore", "", "Rext"], fontsize=7)
    ax1.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("thermal_polar.png", dpi=150)
    plt.show()
    print("  Saved: thermal_polar.png")

    # ---- 2. Polar torque waveform ----
    Tem_vals = np.array(mag_results["Tem_vals"])
    time_vec = np.array(mag_results["time_vec"])

    # Normalise time/angle axis to 0→2π
    theta_norm = (time_vec - time_vec.min()) / (time_vec.max() - time_vec.min()) * 2 * np.pi
    # Close the loop
    theta_plot = np.append(theta_norm, theta_norm[0])
    torque_plot = np.append(Tem_vals,  Tem_vals[0])

    fig2, ax2 = plt.subplots(subplot_kw={"projection": "polar"}, figsize=(7, 7))
    ax2.plot(theta_plot, torque_plot, color="royalblue", linewidth=1.8)
    ax2.fill(theta_plot, torque_plot, alpha=0.15, color="royalblue")

    # Mark mean torque as a circle
    ax2.plot(theta_plot, np.full_like(torque_plot, mag_results["Tem_avg"]),
             "r--", linewidth=1.2, label=f"Mean = {mag_results['Tem_avg']:.3f} Nm")

    ax2.set_title("Torque Waveform — Polar View\n(one electrical cycle)",
                  va="bottom", pad=20)
    ax2.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1))
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("torque_polar.png", dpi=150)
    plt.show()
    print("  Saved: torque_polar.png")

print("\n=== Polar Plot Wrapper ===")
polar_wrapper(thermal, mag_results, PARAMS)

# ============================================================
# STAGE 5 — 3D GEOMETRY & STL EXPORT via CadQuery
# ============================================================
def build_and_export_stl(p, winding):
    slots       = p["stator_slots"]
    L           = p["motor_length"]
    Rext        = p["Rext"]
    Rint        = p["Rint"]
    Rrot        = p["Rrotor_ext"]
    Rsh         = p["Rshaft"]
    t_mag       = p["magnet_thickness"]
    poles       = p["pole_pairs"] * 2
    slot_depth  = (Rext - Rint) * 0.85
    slot_width  = 2 * np.pi * Rint / slots * 0.45

    # STL tolerance — tighter = finer mesh, larger file
    TOL     = 0.05    # mm-scale linear tolerance
    ANG_TOL = 0.1     # radians

    # ---- Stator core ----
    stator_core = cq.Workplane("XY").circle(Rext).circle(Rint).extrude(L)
    for i in range(slots):
        ang = 360.0 * i / slots
        stator_core = (
            stator_core.faces(">Z").workplane()
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(Rint + slot_depth/2, 0)
            .rect(slot_depth, slot_width)
            .cutThruAll()
        )
    stator_core.val().exportStl("stator_core.stl", tolerance=TOL, angularTolerance=ANG_TOL)
    print("  Exported: stator_core.stl")

    # ---- Windings — each coil as a separate solid, merged into one STL ----
    winding_assy = Assembly()
    slot_depth_w = (Rext - Rint) * 0.75
    slot_width_w = 2 * np.pi * Rint / slots * 0.38
    for i in range(slots):
        ang = 360.0 * i / slots
        coil = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(Rint + slot_depth_w/2, 0)
            .rect(slot_depth_w, slot_width_w)
            .extrude(L)
        )
        winding_assy.add(coil, name=f"coil_{i}", color=Color("orange"))
    winding_assy.toCompound().exportStl(
        "windings.stl", tolerance=TOL, angularTolerance=ANG_TOL)
    print("  Exported: windings.stl")

    # ---- Rotor core ----
    rotor_core = cq.Workplane("XY").circle(Rrot).circle(Rsh).extrude(L)
    rotor_core.val().exportStl("rotor_core.stl", tolerance=TOL, angularTolerance=ANG_TOL)
    print("  Exported: rotor_core.stl")

    # ---- Permanent magnets ----
    magnet_assy = Assembly()
    mag_w = 2 * np.pi * Rrot / poles * 0.85
    for i in range(poles):
        ang   = 360.0 * i / poles
        mag_r = Rrot + t_mag / 2
        magnet = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(mag_r, 0)
            .rect(t_mag, mag_w)
            .extrude(L)
        )
        magnet_assy.add(magnet, name=f"mag_{i}",
                        color=Color("lightgray") if i%2==0 else Color("black"))
    magnet_assy.toCompound().exportStl(
        "magnets.stl", tolerance=TOL, angularTolerance=ANG_TOL)
    print("  Exported: magnets.stl")

    # ---- Full assembly STL ----
    full_assy = Assembly(name="drone_motor_12S")
    full_assy.add(stator_core, name="stator", color=Color("lightgray"))
    full_assy.add(rotor_core,  name="rotor",  color=Color("black"))

    slot_depth_w = (Rext - Rint) * 0.75
    slot_width_w = 2 * np.pi * Rint / slots * 0.38
    mag_w        = 2 * np.pi * Rrot / poles * 0.85

    for i in range(slots):
        ang  = 360.0 * i / slots
        coil = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(Rint + slot_depth_w/2, 0)
            .rect(slot_depth_w, slot_width_w)
            .extrude(L)
        )
        full_assy.add(coil, name=f"coil_{i}", color=Color("orange"))

    for i in range(poles):
        ang    = 360.0 * i / poles
        mag_r  = Rrot + t_mag / 2
        magnet = (
            cq.Workplane("XY")
            .transformed(rotate=cq.Vector(0, 0, ang))
            .center(mag_r, 0)
            .rect(t_mag, mag_w)
            .extrude(L)
        )
        full_assy.add(magnet, name=f"mag_{i}",
                      color=Color("lightgray") if i%2==0 else Color("black"))

    # toCompound() flattens the assembly into a single brep for STL
    full_assy.toCompound().exportStl(
        "motor_full_assembly.stl", tolerance=TOL, angularTolerance=ANG_TOL)
    print("  Exported: motor_full_assembly.stl")

print("\n=== STL Export ===")
build_and_export_stl(PARAMS, winding)

# ============================================================
# STAGE 6 — RESULTS SUMMARY
# ============================================================
summary = {
    "winding"  : {k: float(v) for k, v in winding.items()},
    "magnetics": {"Tem_avg": float(mag_results["Tem_avg"]),
                  "Tem_rip": float(mag_results["Tem_rip"])},
    "thermal"  : {"T_max_winding_C": float(thermal["T_max_winding"]),
                  "T_max_C"        : float(thermal["T_max"]),
                  "magnet_safe"    : bool(thermal["T_max"] < 120)},
}
with open("motor_design_results.json", "w") as f:
    json.dump(summary, f, indent=2)
print("\nAll results saved to motor_design_results.json")