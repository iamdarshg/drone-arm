#!/usr/bin/env python3
"""
Drone Pitch Autopilot — PID Controller Design with MDO-Based Gain Optimization

This tool demonstrates industry-standard techniques for PID controller design
and optimization, as used in aerospace and automotive applications.

CONTROL SYSTEM BACKGROUND:
A control system makes a system's output follow a desired reference.
For a drone, we want the pitch angle to track user commands.

Architecture:
    Reference ──►[ C(s) ]──►┬──►[ G(s) ]──► Output
                          │ │
                          ▼ │
                          -[ Feedback ]
                          │ │
                          └─────────┘

Where: C(s) = PID Controller (designed), G(s) = Plant (drone dynamics)

PID CONTROLLER MATHEMATICS:
A PID controller produces: u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de/dt
In Laplace domain: C(s) = (Kd·s² + Kp·s + Ki) / s

- Proportional (Kp): Response proportional to current error
- Integral (Ki): Eliminates steady-state offset
- Derivative (Kd): Adds damping, reduces overshoot

DRONE PITCH DYNAMICS (The Plant G(s)):
Ixx·θ̈ + b_damp·θ̇ = τ (Euler's equation)
G(s) = 1 / (Ixx·s² + b_damp·s)

STABILITY ANALYSIS:
- Gain Margin (GM): How much gain can increase before instability
- Phase Margin (PM): How much phase lag can be added before instability
- Standard aerospace requirements: GM ≥ 6 dB, PM ≥ 45°

OPTIMIZATION PROBLEM:
Minimize: J = settling_time + 0.05 × overshoot_pct
Subject to: GM ≥ 6 dB, PM ≥ 45°
"""

import warnings
from dataclasses import dataclass
from typing import Any, Dict, Tuple

import control as ct
import matplotlib.pyplot as plt
import numpy as np
import openmdao.api as om

# =============================================================================
# DRONE PITCH DYNAMICS MODEL
# =============================================================================


@dataclass
class DronePitchModel:
    """
    Linearized pitch dynamics model for a quadrotor in hover.

    G(s) = 1 / (Ixx·s² + b_damp·s)

    Attributes:
        Ixx: Moment of inertia about pitch axis (kg·m²)
        b_damp: Aerodynamic damping coefficient (N·m·s/rad)
    """

    Ixx: float = 0.01  # kg·m²
    b_damp: float = 0.1  # N·m·s/rad

    def get_plant_tf(self) -> ct.TransferFunction:
        """Get pitch dynamics transfer function G(s) = 1 / (Ixx·s² + b_damp·s)"""
        return ct.TransferFunction([1], [self.Ixx, self.b_damp, 0])

    def get_pid_tf(self, Kp: float, Ki: float, Kd: float) -> ct.TransferFunction:
        """Get PID controller C(s) = (Kd·s² + Kp·s + Ki) / s"""
        return ct.TransferFunction([Kd, Kp, Ki], [1, 0])

    def get_open_loop_tf(self, Kp: float, Ki: float, Kd: float) -> ct.TransferFunction:
        """Get open-loop L(s) = C(s)·G(s)"""
        plant = self.get_plant_tf()
        pid = self.get_pid_tf(Kp, Ki, Kd)
        return ct.series(pid, plant)

    def get_closed_loop_tf(
        self, Kp: float, Ki: float, Kd: float
    ) -> ct.TransferFunction:
        """Get closed-loop T(s) = L(s) / (1 + L(s))"""
        open_loop = self.get_open_loop_tf(Kp, Ki, Kd)
        return ct.feedback(open_loop, 1)


# =============================================================================
# OPENMDAO COMPONENT FOR OPTIMIZATION
# =============================================================================


class PIDPerformanceComp(om.ExplicitComponent):
    """
    OpenMDAO component evaluating PID controller performance.

    Wraps the control library analysis in an OpenMDAO interface,
    enabling multidisciplinary optimization (MDO).
    """

    def initialize(self):
        self.options.declare("Ixx", default=0.01, desc="Moment of inertia (kg·m²)")
        self.options.declare("b_damp", default=0.1, desc="Damping coefficient")

    def setup(self):
        # Inputs: PID gains
        self.add_input("Kp", val=5.0, desc="Proportional gain")
        self.add_input("Ki", val=1.0, desc="Integral gain")
        self.add_input("Kd", val=0.5, desc="Derivative gain")

        # Outputs: Performance metrics
        self.add_output("settling_time", val=0.0, desc="2% settling time (s)")
        self.add_output("overshoot_pct", val=0.0, desc="Overshoot (%)")
        self.add_output("gain_margin_dB", val=20.0, desc="Gain margin (dB)")
        self.add_output("phase_margin_deg", val=60.0, desc="Phase margin (deg)")
        self.add_output("objective", val=0.0, desc="Objective value")

        # Finite difference for derivatives
        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        """Evaluate controller performance at given gains."""
        Kp, Ki, Kd = inputs["Kp"], inputs["Ki"], inputs["Kd"]
        Ixx, b_damp = self.options["Ixx"], self.options["b_damp"]

        try:
            # Build transfer functions
            plant_tf = ct.TransferFunction([1], [Ixx, b_damp, 0])
            pid_tf = ct.TransferFunction([Kd, Kp, Ki], [1, 0])
            open_loop_tf = ct.series(pid_tf, plant_tf)
            closed_loop_tf = ct.feedback(open_loop_tf, 1)

            # Stability margins
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                result = ct.stability_margins(open_loop_tf)

            if len(result) == 4:
                gm, pm = result[0], result[1]
            else:
                gm, pm = result[0], result[1]

            # Handle infinity
            gain_margin_dB = (
                40.0 if np.isinf(gm) or np.isnan(gm) else max(0.1, 20 * np.log10(gm))
            )
            phase_margin_deg = (
                90.0 if np.isinf(pm) or np.isnan(pm) else max(1.0, float(pm))
            )

            # Clamp for numerical stability
            gain_margin_dB = np.clip(gain_margin_dB, 0.1, 100.0)
            phase_margin_deg = np.clip(phase_margin_deg, 1.0, 90.0)

            # Step response analysis
            t, y = ct.step_response(closed_loop_tf)
            ss_value = y[-1] if len(y) > 0 else 1.0
            y_norm = y / ss_value if ss_value != 0 else y

            # Overshoot
            peak_idx = np.argmax(y_norm)
            peak_value = y_norm[peak_idx]
            overshoot_pct = max(0.0, (peak_value - 1.0) * 100)

            # Settling time (2% criterion)
            tolerance = 0.02
            within_band = np.abs(y_norm - 1.0) <= tolerance
            settling_time = 0.0
            if np.any(within_band):
                for i in range(len(t) - 1, -1, -1):
                    if np.abs(y_norm[i] - 1.0) > tolerance:
                        settling_time = t[min(i + 1, len(t) - 1)]
                        break
                else:
                    settling_time = t[-1]
            else:
                settling_time = t[-1]

            # Objective: balance speed and overshoot
            objective = settling_time + 0.05 * overshoot_pct

            outputs["settling_time"] = settling_time
            outputs["overshoot_pct"] = overshoot_pct
            outputs["gain_margin_dB"] = gain_margin_dB
            outputs["phase_margin_deg"] = phase_margin_deg
            outputs["objective"] = objective

        except Exception:
            outputs["settling_time"] = 10.0
            outputs["overshoot_pct"] = 100.0
            outputs["gain_margin_dB"] = 0.1
            outputs["phase_margin_deg"] = 1.0
            outputs["objective"] = 100.0


# =============================================================================
# ANALYSIS FUNCTION
# =============================================================================


def analyze_with_gains(
    drone: DronePitchModel, Kp: float, Ki: float, Kd: float
) -> Dict[str, Any]:
    """Analyze control system with given PID gains."""
    open_loop_tf = drone.get_open_loop_tf(Kp, Ki, Kd)
    closed_loop_tf = drone.get_closed_loop_tf(Kp, Ki, Kd)

    # Stability margins
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        result = ct.stability_margins(open_loop_tf)

    if len(result) == 4:
        gm, pm = result[0], result[1]
    else:
        gm, pm = result[0], result[1]

    gain_margin_dB = 20 * np.log10(gm) if gm != np.inf else 60.0
    phase_margin_deg = pm if pm != np.inf else 90.0

    # Step response
    t, y = ct.step_response(closed_loop_tf)
    ss_value = y[-1] if len(y) > 0 else 1.0
    y_norm = y / ss_value if ss_value != 0 else y

    # Metrics
    peak_idx = np.argmax(y_norm)
    overshoot = max(0, (y_norm[peak_idx] - 1.0) * 100)

    tolerance = 0.02
    within_band = np.abs(y_norm - 1.0) <= tolerance
    settling_time = 0.0
    if np.any(within_band):
        for i in range(len(t) - 1, -1, -1):
            if np.abs(y_norm[i] - 1.0) > tolerance:
                settling_time = t[min(i + 1, len(t) - 1)]
                break
        else:
            settling_time = t[-1]
    else:
        settling_time = t[-1]

    return {
        "Kp": Kp,
        "Ki": Ki,
        "Kd": Kd,
        "open_loop_tf": open_loop_tf,
        "closed_loop_tf": closed_loop_tf,
        "gain_margin": gain_margin_dB,
        "phase_margin": phase_margin_deg,
        "settling_time": settling_time,
        "overshoot": overshoot,
        "time_response": (t, y),
    }


# =============================================================================
# PLOTTING FUNCTIONS
# =============================================================================


def plot_bode_comparison(
    ax, open_loop_before, open_loop_after, gains_before, gains_after
):
    """Bode plot comparison."""
    # Handle both old and new API
    fresp1 = ct.frequency_response(open_loop_before)
    fresp2 = ct.frequency_response(open_loop_after)
    omega1 = fresp1[0]
    omega2 = fresp2[0]
    mag1 = np.abs(fresp1[1])
    phase1 = np.angle(fresp1[1])
    mag2 = np.abs(fresp2[1])
    phase2 = np.angle(fresp2[1])

    ax_bode_mag = ax
    ax_bode_phase = ax.twinx()

    ax_bode_mag.semilogx(
        omega1, 20 * np.log10(np.maximum(mag1, 1e-9)), "b-", linewidth=2, label="Before"
    )
    ax_bode_mag.semilogx(
        omega2, 20 * np.log10(np.maximum(mag2, 1e-9)), "r-", linewidth=2, label="After"
    )
    ax_bode_mag.axhline(y=0, color="k", linestyle="--", alpha=0.3)
    ax_bode_mag.set_ylabel("Magnitude (dB)", color="b")
    ax_bode_mag.tick_params(axis="y", labelcolor="b")
    ax_bode_phase.semilogx(omega1, np.degrees(phase1), "b--", linewidth=1.5, alpha=0.7)
    ax_bode_phase.semilogx(omega2, np.degrees(phase2), "r--", linewidth=1.5, alpha=0.7)
    ax_bode_phase.set_ylabel("Phase (deg)", color="gray")
    ax_bode_phase.tick_params(axis="y", labelcolor="gray")
    ax_bode_mag.set_xlabel("Frequency (rad/s)")
    ax_bode_mag.set_title(
        f"Bode: Kp={gains_before['Kp']:.2f}→{gains_after['Kp']:.2f}, "
        f"Ki={gains_before['Ki']:.2f}→{gains_after['Ki']:.2f}, "
        f"Kd={gains_before['Kd']:.2f}→{gains_after['Kd']:.2f}"
    )
    ax_bode_mag.legend(loc="upper left")
    ax_bode_mag.grid(True, alpha=0.3)


def plot_step_comparison(ax, time_before, time_after, metrics_before, metrics_after):
    """Step response comparison."""
    t1, y1 = time_before
    t2, y2 = time_after
    y1 = y1 / y1[-1] if y1[-1] != 0 else y1
    y2 = y2 / y2[-1] if y2[-1] != 0 else y2

    ax.plot(t1, y1, "b-", linewidth=2, label="Before")
    ax.plot(t2, y2, "r-", linewidth=2, label="After")
    ax.axhline(y=0.95, color="g", linestyle=":", alpha=0.5)
    ax.axhline(y=1.05, color="g", linestyle=":", alpha=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Normalized Response")
    ax.set_title(
        f"Step: Ts={metrics_before['settling_time']:.3f}s→{metrics_after['settling_time']:.3f}s, "
        f"OS={metrics_before['overshoot']:.1f}%→{metrics_after['overshoot']:.1f}%"
    )
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, max(t1[-1], t2[-1]) * 1.1])


def plot_nyquist_comparison(ax, open_loop_before, open_loop_after):
    """Nyquist diagram comparison."""
    fresp1 = ct.frequency_response(open_loop_before)
    fresp2 = ct.frequency_response(open_loop_after)
    omega1 = fresp1[0]
    omega2 = fresp2[0]
    mag1 = np.abs(fresp1[1])
    phase1 = np.angle(fresp1[1])
    mag2 = np.abs(fresp2[1])
    phase2 = np.angle(fresp2[1])

    z1 = mag1 * np.exp(1j * np.radians(phase1))
    z2 = mag2 * np.exp(1j * np.radians(phase2))

    ax.plot(z1.real, z1.imag, "b-", linewidth=2, label="Before")
    ax.plot(z2.real, z2.imag, "r-", linewidth=2, label="After")
    ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
    ax.axvline(x=-1, color="k", linestyle="--", alpha=0.3)
    ax.plot(-1, 0, "ko", markersize=10, fillstyle="none", label="-1 point")
    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.set_title("Nyquist (Encirclements of -1 indicate stability)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")


def plot_pole_zero_comparison(ax, closed_loop_before, closed_loop_after):
    """Pole-zero map comparison."""
    poles1 = ct.poles(closed_loop_before)
    poles2 = ct.poles(closed_loop_after)
    zeros1 = ct.zeros(closed_loop_before)
    zeros2 = ct.zeros(closed_loop_after)

    ax.plot(
        np.real(poles1), np.imag(poles1), "bx", markersize=10, label="Poles (Before)"
    )
    ax.plot(
        np.real(poles2), np.imag(poles2), "rx", markersize=10, label="Poles (After)"
    )
    ax.plot(
        np.real(zeros1), np.imag(zeros1), "bo", markersize=8, label="Zeros (Before)"
    )
    ax.plot(np.real(zeros2), np.imag(zeros2), "ro", markersize=8, label="Zeros (After)")

    theta = np.linspace(0, 2 * np.pi, 100)
    ax.plot(np.cos(theta), np.sin(theta), "k--", alpha=0.4, label="Unit circle")
    ax.axhline(y=0, color="k", linestyle="-", alpha=0.2)
    ax.axvline(x=0, color="k", linestyle="-", alpha=0.2)
    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.set_title("Pole-Zero Map (LHP = stable)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")


def plot_optimization_history(axes, history):
    """Plot optimization convergence."""
    iterations = range(len(history["objective"]))

    axes[0, 0].plot(iterations, history["Kp"], "b-", linewidth=2)
    axes[0, 0].set_xlabel("Iteration")
    axes[0, 0].set_ylabel("Kp")
    axes[0, 0].set_title("Kp Evolution")
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(iterations, history["Ki"], "g-", linewidth=2)
    axes[0, 1].set_xlabel("Iteration")
    axes[0, 1].set_ylabel("Ki")
    axes[0, 1].set_title("Ki Evolution")
    axes[0, 1].grid(True, alpha=0.3)

    axes[0, 2].plot(iterations, history["Kd"], "r-", linewidth=2)
    axes[0, 2].set_xlabel("Iteration")
    axes[0, 2].set_ylabel("Kd")
    axes[0, 2].set_title("Kd Evolution")
    axes[0, 2].grid(True, alpha=0.3)

    axes[1, 0].plot(iterations, history["objective"], "purple", linewidth=2)
    axes[1, 0].set_xlabel("Iteration")
    axes[1, 0].set_ylabel("Objective")
    axes[1, 0].set_title("Objective: Ts + 0.05×OS")
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(iterations, history["gain_margin"], "b-", linewidth=2, label="GM")
    axes[1, 1].axhline(y=6, color="r", linestyle="--", alpha=0.7, label="GM ≥ 6 dB")
    axes[1, 1].set_xlabel("Iteration")
    axes[1, 1].set_ylabel("Gain Margin (dB)")
    axes[1, 1].set_title("Gain Margin Constraint")
    axes[1, 1].legend(loc="best")
    axes[1, 1].grid(True, alpha=0.3)

    axes[1, 2].plot(
        iterations, history["phase_margin"], "orange", linewidth=2, label="PM"
    )
    axes[1, 2].axhline(y=45, color="r", linestyle="--", alpha=0.7, label="PM ≥ 45°")
    axes[1, 2].set_xlabel("Iteration")
    axes[1, 2].set_ylabel("Phase Margin (°)")
    axes[1, 2].set_title("Phase Margin Constraint")
    axes[1, 2].legend(loc="best")
    axes[1, 2].grid(True, alpha=0.3)


def print_comparison_table(
    initial_gains, optimized_gains, initial_metrics, optimized_metrics
):
    """Print formatted comparison table."""
    print("\n" + "=" * 80)
    print(" COMPARISON: Hand-Tuned vs Optimized PID Gains")
    print("=" * 80)
    print(f"{'Parameter':<25} {'Before':<15} {'After':<15} {'Change':<15}")
    print("-" * 80)

    print(
        f"{'Kp (Proportional)':<25} {initial_gains['Kp']:<15.4f} {optimized_gains['Kp']:<15.4f} {'—':<15}"
    )
    print(
        f"{'Ki (Integral)':<25} {initial_gains['Ki']:<15.4f} {optimized_gains['Ki']:<15.4f} {'—':<15}"
    )
    print(
        f"{'Kd (Derivative)':<25} {initial_gains['Kd']:<15.4f} {optimized_gains['Kd']:<15.4f} {'—':<15}"
    )

    st_b, st_a = initial_metrics["settling_time"], optimized_metrics["settling_time"]
    st_imp = (st_b - st_a) / st_b * 100 if st_b > 0 else 0
    print(
        f"{'Settling Time (s)':<25} {st_b:<15.4f} {st_a:<15.4f} {f'{st_imp:.1f}% faster':<15}"
    )

    os_b, os_a = initial_metrics["overshoot"], optimized_metrics["overshoot"]
    print(
        f"{'Overshoot (%)':<25} {os_b:<15.2f} {os_a:<15.2f} {f'{os_b - os_a:.1f}% less':<15}"
    )

    gm_b, gm_a = initial_metrics["gain_margin"], optimized_metrics["gain_margin"]
    pm_b, pm_a = initial_metrics["phase_margin"], optimized_metrics["phase_margin"]
    print(f"{'Gain Margin (dB)':<25} {gm_b:<15.2f} {gm_a:<15.2f} {'—':<15}")
    print(f"{'Phase Margin (°)':<25} {pm_b:<15.2f} {pm_a:<15.2f} {'—':<15}")

    print("=" * 80)
    print("\nConstraint Verification:")
    gm_check = "PASS" if optimized_metrics["gain_margin"] >= 6 else "FAIL"
    pm_check = "PASS" if optimized_metrics["phase_margin"] >= 45 else "FAIL"
    print(f" GM ≥ 6 dB: {gm_check} ({optimized_metrics['gain_margin']:.2f} dB)")
    print(f" PM ≥ 45°: {pm_check} ({optimized_metrics['phase_margin']:.2f}°)")
    print("=" * 80)


# =============================================================================
# OPTIMIZATION SETUP
# =============================================================================


def create_optimization_problem(drone: DronePitchModel) -> om.Problem:
    """Create and configure OpenMDAO optimization problem."""
    prob = om.Problem()
    prob.model = om.Group()
    prob.model.add_subsystem(
        "pid_perf",
        PIDPerformanceComp(Ixx=drone.Ixx, b_damp=drone.b_damp),
        promotes_inputs=["*"],
        promotes_outputs=["*"],
    )

    prob.driver = om.ScipyOptimizeDriver()
    prob.driver.options["optimizer"] = "SLSQP"
    prob.driver.options["maxiter"] = 300
    prob.driver.options["tol"] = 1e-8

    prob.model.add_design_var("Kp", lower=0.1, upper=50.0)
    prob.model.add_design_var("Ki", lower=0.0, upper=20.0)
    prob.model.add_design_var("Kd", lower=0.0, upper=10.0)

    prob.model.add_objective("objective")
    prob.model.add_constraint("gain_margin_dB", lower=6.0)
    prob.model.add_constraint("phase_margin_deg", lower=45.0)

    prob.setup()
    return prob


# =============================================================================
# MAIN ORCHESTRATOR
# =============================================================================


def main():
    """Main orchestrator for PID design and optimization."""
    print("=" * 80)
    print(" DRONE PITCH AUTOPILOT — PID Design + MDO-Based Gain Optimization")
    print("=" * 80)

    drone = DronePitchModel(Ixx=0.01, b_damp=0.1)

    print(f"\nPlant Parameters:")
    print(f" Ixx = {drone.Ixx} kg·m²")
    print(f" b_damp = {drone.b_damp} N·m·s/rad")
    print(f"\nTransfer Function: G(s) = 1 / ({drone.Ixx}s² + {drone.b_damp}s)")

    # Hand-tuned initial gains
    initial_gains = {"Kp": 5.0, "Ki": 1.0, "Kd": 0.5}

    # ========================================================================
    # PHASE 1: Hand-tuned analysis
    # ========================================================================
    print("\n" + "-" * 80)
    print(" PHASE 1: Hand-Tuned PID Analysis")
    print("-" * 80)
    print(
        f"Initial Gains: Kp={initial_gains['Kp']}, Ki={initial_gains['Ki']}, Kd={initial_gains['Kd']}"
    )

    initial_metrics = analyze_with_gains(
        drone, initial_gains["Kp"], initial_gains["Ki"], initial_gains["Kd"]
    )

    print(f" Settling Time: {initial_metrics['settling_time']:.4f} s")
    print(f" Overshoot: {initial_metrics['overshoot']:.2f}%")
    print(f" Gain Margin: {initial_metrics['gain_margin']:.2f} dB")
    print(f" Phase Margin: {initial_metrics['phase_margin']:.2f}°")

    # ========================================================================
    # PHASE 2: OpenMDAO optimization
    # ========================================================================
    print("\n" + "-" * 80)
    print(" PHASE 2: OpenMDAO Optimization")
    print("-" * 80)
    print("Objective: Minimize (settling_time + 0.05 × overshoot)")
    print("Constraints: GM ≥ 6 dB, PM ≥ 45°")
    print("Optimizer: SLSQP (max 300 iterations)")

    prob = create_optimization_problem(drone)

    # Set initial design point
    prob.set_val("Kp", initial_gains["Kp"])
    prob.set_val("Ki", initial_gains["Ki"])
    prob.set_val("Kd", initial_gains["Kd"])

    print(f"\nInitial Point:")
    print(
        f" Kp={initial_gains['Kp']}, Ki={initial_gains['Ki']}, Kd={initial_gains['Kd']}"
    )

    prob.run_model()
    print(f"Objective: {prob.get_val('objective')[0]:.4f}")
    print(f" Settling Time: {prob.get_val('settling_time')[0]:.4f} s")
    print(f" Overshoot: {prob.get_val('overshoot_pct')[0]:.2f}%")
    print(f" Gain Margin: {prob.get_val('gain_margin_dB')[0]:.2f} dB")
    print(f" Phase Margin: {prob.get_val('phase_margin_deg')[0]:.2f}°")

    # Track history
    history = {
        "Kp": [],
        "Ki": [],
        "Kd": [],
        "objective": [],
        "gain_margin": [],
        "phase_margin": [],
    }

    def record_history():
        history["Kp"].append(prob.get_val("Kp")[0])
        history["Ki"].append(prob.get_val("Ki")[0])
        history["Kd"].append(prob.get_val("Kd")[0])
        history["objective"].append(prob.get_val("objective")[0])
        history["gain_margin"].append(prob.get_val("gain_margin_dB")[0])
        history["phase_margin"].append(prob.get_val("phase_margin_deg")[0])

    record_history()

    print("\nRunning optimization...")
    try:
        prob.run_driver()
        opt_success = True
    except Exception as e:
        print(f"Optimization encountered issue: {e}")
        opt_success = False

    record_history()

    Kp_opt = float(prob.get_val("Kp")[0])
    Ki_opt = float(prob.get_val("Ki")[0])
    Kd_opt = float(prob.get_val("Kd")[0])
    optimized_gains = {"Kp": Kp_opt, "Ki": Ki_opt, "Kd": Kd_opt}

    print(
        f"\nOptimization Result: {'Success' if opt_success else 'Issues encountered'}"
    )
    print(f"Optimized Gains: Kp={Kp_opt:.4f}, Ki={Ki_opt:.4f}, Kd={Kd_opt:.4f}")

    # ========================================================================
    # PHASE 3: Optimized analysis
    # ========================================================================
    print("\n" + "-" * 80)
    print(" PHASE 3: Optimized PID Analysis")
    print("-" * 80)
    print(f"Optimized Gains: Kp={Kp_opt:.4f}, Ki={Ki_opt:.4f}, Kd={Kd_opt:.4f}")

    optimized_metrics = analyze_with_gains(drone, Kp_opt, Ki_opt, Kd_opt)
    print(f" Settling Time: {optimized_metrics['settling_time']:.4f} s")
    print(f" Overshoot: {optimized_metrics['overshoot']:.2f}%")
    print(f" Gain Margin: {optimized_metrics['gain_margin']:.2f} dB")
    print(f" Phase Margin: {optimized_metrics['phase_margin']:.2f}°")

    # ========================================================================
    # PHASE 4: Generate plots
    # ========================================================================
    print("\n" + "-" * 80)
    print(" PHASE 4: Generating Plots")
    print("-" * 80)

    # Comparison plot
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(
        "Drone Pitch Autopilot: Hand-Tuned vs Optimized PID",
        fontsize=16,
        fontweight="bold",
    )

    plot_bode_comparison(
        axes[0, 0],
        initial_metrics["open_loop_tf"],
        optimized_metrics["open_loop_tf"],
        initial_gains,
        optimized_gains,
    )
    plot_step_comparison(
        axes[0, 1],
        initial_metrics["time_response"],
        optimized_metrics["time_response"],
        initial_metrics,
        optimized_metrics,
    )
    plot_nyquist_comparison(
        axes[1, 0], initial_metrics["open_loop_tf"], optimized_metrics["open_loop_tf"]
    )
    plot_pole_zero_comparison(
        axes[1, 1],
        initial_metrics["closed_loop_tf"],
        optimized_metrics["closed_loop_tf"],
    )

    plt.tight_layout()
    plt.savefig("tools/PID/comparison_plots.png", dpi=150, bbox_inches="tight")
    plt.close()
    print(" Saved: tools/PID/comparison_plots.png")

    # History plot
    fig2, axes2 = plt.subplots(2, 3, figsize=(16, 10))
    fig2.suptitle("OpenMDAO Optimization Convergence", fontsize=16, fontweight="bold")
    plot_optimization_history(axes2, history)
    plt.tight_layout()
    plt.savefig("tools/PID/optimization_history.png", dpi=150, bbox_inches="tight")
    plt.close()
    print(" Saved: tools/PID/optimization_history.png")

    # ========================================================================
    # SUMMARY
    # ========================================================================
    print("\n" + "=" * 80)
    print(" SUMMARY")
    print("=" * 80)
    print_comparison_table(
        initial_gains, optimized_gains, initial_metrics, optimized_metrics
    )
    print("\n" + "=" * 80)
    print(" OPTIMIZATION COMPLETE")
    print("=" * 80)

    return {
        "initial_gains": initial_gains,
        "optimized_gains": optimized_gains,
        "initial_metrics": initial_metrics,
        "optimized_metrics": optimized_metrics,
        "history": history,
    }


if __name__ == "__main__":
    results = main()
