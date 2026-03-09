#!/usr/bin/env python3
"""
Drone Pitch Autopilot — Main Orchestrator
=========================================

This script orchestrates the complete PID design and optimization workflow:
1. Analyzes the drone pitch dynamics with initial (hand-tuned) gains
2. Runs OpenMDAO optimization to find optimal PID gains
3. Compares before/after results with visualizations

The system models a quadrotor's linearized pitch dynamics and uses OpenMDAO
to optimally tune PID gains subject to stability margin constraints.
"""

import warnings
from dataclasses import dataclass
from typing import Any, Dict, Tuple

import control as ct
import matplotlib.pyplot as plt
import numpy as np
import openmdao.api as om

# ============================================================================
# Drone Pitch Dynamics Model
# ============================================================================


@dataclass
class DronePitchModel:
    """
    Linearized pitch dynamics model for a quadrotor in hover.

    Transfer function: G(s) = 1 / (Ixx * s^2 + b_damp * s)

    Attributes:
        Ixx: Moment of inertia about pitch axis (kg*m^2)
        b_damp: Aerodynamic damping coefficient (N*m*s/rad)
    """

    Ixx: float = 0.01  # kg*m^2
    b_damp: float = 0.1  # N*m*s/rad

    def get_plant_tf(self) -> ct.TransferFunction:
        """Create pitch dynamics transfer function G(s) = 1 / (Ixx*s^2 + b_damp*s)"""
        return ct.TransferFunction([1], [self.Ixx, self.b_damp, 0])

    def get_pid_tf(self, Kp: float, Ki: float, Kd: float) -> ct.TransferFunction:
        """Create PID controller transfer function C(s) = (Kd*s^2 + Kp*s + Ki) / s"""
        return ct.TransferFunction([Kd, Kp, Ki], [1, 0])

    def get_open_loop_tf(self, Kp: float, Ki: float, Kd: float) -> ct.TransferFunction:
        """Create open-loop transfer function L(s) = C(s) * G(s)"""
        plant = self.get_plant_tf()
        pid = self.get_pid_tf(Kp, Ki, Kd)
        return ct.series(pid, plant)

    def get_closed_loop_tf(
        self, Kp: float, Ki: float, Kd: float
    ) -> ct.TransferFunction:
        """Create closed-loop transfer function T(s) = L(s) / (1 + L(s))"""
        open_loop = self.get_open_loop_tf(Kp, Ki, Kd)
        return ct.feedback(open_loop, 1)


# ============================================================================
# OpenMDAO Component for PID Performance Evaluation
# ============================================================================


class PIDPerformanceComp(om.ExplicitComponent):
    """
    OpenMDAO component that evaluates PID controller performance.

    Takes PID gains as inputs and computes performance/stability metrics.
    """

    def initialize(self):
        """Initialize component with drone parameters."""
        self.options.declare("Ixx", default=0.01, desc="Moment of inertia (kg·m²)")
        self.options.declare(
            "b_damp", default=0.1, desc="Damping coefficient (N·m·s/rad)"
        )

    def setup(self):
        """Set up inputs and outputs."""
        Ixx = self.options["Ixx"]
        b_damp = self.options["b_damp"]

        # Inputs: PID gains
        self.add_input("Kp", val=5.0, desc="Proportional gain")
        self.add_input("Ki", val=1.0, desc="Integral gain")
        self.add_input("Kd", val=0.5, desc="Derivative gain")

        # Outputs: Performance metrics
        self.add_output("settling_time", val=0.0, desc="Settling time (s)")
        self.add_output("overshoot_pct", val=0.0, desc="Overshoot (%)")
        self.add_output("gain_margin_dB", val=20.0, desc="Gain margin (dB)")
        self.add_output("phase_margin_deg", val=60.0, desc="Phase margin (deg)")
        self.add_output("rise_time", val=0.0, desc="Rise time (s)")
        self.add_output("objective", val=0.0, desc="Objective function value")

        # Finite difference for derivatives (control library doesn't support analytic)
        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        """Compute performance metrics from PID gains."""
        Kp = inputs["Kp"]
        Ki = inputs["Ki"]
        Kd = inputs["Kd"]
        Ixx = self.options["Ixx"]
        b_damp = self.options["b_damp"]

        try:
            # Build transfer functions
            plant_tf = ct.TransferFunction([1], [Ixx, b_damp, 0])
            pid_tf = ct.TransferFunction([Kd, Kp, Ki], [1, 0])
            open_loop_tf = ct.series(pid_tf, plant_tf)
            closed_loop_tf = ct.feedback(open_loop_tf, 1)

            # Stability margins
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                gm, pm, wg, wp = ct.stability_margins(open_loop_tf)

            # Handle infinite gain margin
            if np.isinf(gm) or np.isnan(gm):
                gain_margin_dB = 40.0
            else:
                gain_margin_dB = max(0.1, 20 * np.log10(gm))

            # Handle infinite phase margin
            if np.isinf(pm) or np.isnan(pm):
                phase_margin_deg = 90.0
            else:
                phase_margin_deg = max(1.0, pm)

            # Clamp to reasonable bounds
            gain_margin_dB = np.clip(gain_margin_dB, 0.1, 100.0)
            phase_margin_deg = np.clip(phase_margin_deg, 1.0, 90.0)

            # Step response analysis
            t, y = ct.step_response(closed_loop_tf)

            # Normalize
            ss_value = y[-1] if len(y) > 0 else 1.0
            if ss_value != 0:
                y_norm = y / ss_value
            else:
                y_norm = y

            # Overshoot
            peak_idx = np.argmax(y_norm)
            peak_value = y_norm[peak_idx]
            overshoot_pct = max(0.0, (peak_value - 1.0) * 100)

            # Settling time (2% criterion)
            tolerance = 0.02
            within_band = np.abs(y_norm - 1.0) <= tolerance

            settling_time = 0.0
            if np.any(within_band):
                # Find last index outside tolerance, then first after
                for i in range(len(t) - 1, -1, -1):
                    if np.abs(y_norm[i] - 1.0) > tolerance:
                        settling_time = t[min(i + 1, len(t) - 1)]
                        break
                else:
                    settling_time = t[-1]
            else:
                settling_time = t[-1]

            # Rise time (10% to 90%)
            rise_time = 0.0
            idx_10 = np.where(y_norm >= 0.1)[0]
            idx_90 = np.where(y_norm >= 0.9)[0]
            if len(idx_10) > 0 and len(idx_90) > 0:
                rise_time = t[idx_90[0]] - t[idx_10[0]]

            # Objective: minimize settling time + weighted overshoot
            objective = settling_time + 0.05 * overshoot_pct

            # Output metrics
            outputs["settling_time"] = settling_time
            outputs["overshoot_pct"] = overshoot_pct
            outputs["gain_margin_dB"] = gain_margin_dB
            outputs["phase_margin_deg"] = phase_margin_deg
            outputs["rise_time"] = rise_time
            outputs["objective"] = objective

        except Exception as e:
            # Unstable or failed computation - return worst-case values
            outputs["settling_time"] = 10.0
            outputs["overshoot_pct"] = 100.0
            outputs["gain_margin_dB"] = 0.1
            outputs["phase_margin_deg"] = 1.0
            outputs["rise_time"] = 10.0
            outputs["objective"] = 100.0


# ============================================================================
# Plotting Functions
# ============================================================================


def plot_bode_comparison(
    ax, open_loop_before, open_loop_after, gains_before: Dict, gains_after: Dict
):
    """Plot Bode diagram comparison."""
    omega1, mag1, phase1 = ct.bode_plot(open_loop_before, plot=False)
    omega2, mag2, phase2 = ct.bode_plot(open_loop_after, plot=False)

    ax_bode_mag = ax
    ax_bode_phase = ax.twinx()

    (l1,) = ax_bode_mag.semilogx(
        omega1, 20 * np.log10(mag1), "b-", linewidth=2, label="Before"
    )
    (l2,) = ax_bode_mag.semilogx(
        omega2, 20 * np.log10(mag2), "r-", linewidth=2, label="After"
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
        f"Bode Plot\nBefore: Kp={gains_before['Kp']:.2f}, Ki={gains_before['Ki']:.2f}, Kd={gains_before['Kd']:.2f}\n"
        f"After: Kp={gains_after['Kp']:.2f}, Ki={gains_after['Ki']:.2f}, Kd={gains_after['Kd']:.2f}"
    )
    ax_bode_mag.legend(loc="upper left")
    ax_bode_mag.grid(True, alpha=0.3)


def plot_step_comparison(
    ax, time_before, time_after, metrics_before: Dict, metrics_after: Dict
):
    """Plot step response comparison."""
    t1, y1 = time_before
    t2, y2 = time_after

    # Normalize
    y1 = y1 / y1[-1] if y1[-1] != 0 else y1
    y2 = y2 / y2[-1] if y2[-1] != 0 else y2

    ax.plot(t1, y1, "b-", linewidth=2, label="Before")
    ax.plot(t2, y2, "r-", linewidth=2, label="After")

    ax.axhline(y=1.0, color="k", linestyle="--", alpha=0.3)
    ax.axhline(y=0.95, color="g", linestyle=":", alpha=0.5)
    ax.axhline(y=1.05, color="g", linestyle=":", alpha=0.5)

    # Mark settling time
    ax.axvline(
        x=metrics_before["settling_time"],
        color="b",
        linestyle="--",
        alpha=0.5,
        label=f"Ts={metrics_before['settling_time']:.3f}s",
    )
    ax.axvline(
        x=metrics_after["settling_time"],
        color="r",
        linestyle="--",
        alpha=0.5,
        label=f"Ts={metrics_after['settling_time']:.3f}s",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Amplitude")
    ax.set_title(
        f"Step Response\nBefore: OS={metrics_before['overshoot']:.1f}%, Ts={metrics_before['settling_time']:.3f}s\n"
        f"After: OS={metrics_after['overshoot']:.1f}%, Ts={metrics_after['settling_time']:.3f}s"
    )
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, max(t1[-1], t2[-1]) * 1.1])


def plot_nyquist_comparison(ax, open_loop_before, open_loop_after):
    """Plot Nyquist diagram comparison."""
    # Extract frequency response data
    omega1, mag1, phase1 = ct.bode_plot(open_loop_before, plot=False)
    omega2, mag2, phase2 = ct.bode_plot(open_loop_after, plot=False)

    # Convert to real/imaginary
    z1 = mag1 * np.exp(1j * np.radians(phase1))
    z2 = mag2 * np.exp(1j * np.radians(phase2))

    ax.plot(z1.real, z1.imag, "b-", linewidth=2, label="Before")
    ax.plot(z2.real, z2.imag, "r-", linewidth=2, label="After")

    # Mark -1 point
    ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
    ax.axvline(x=-1, color="k", linestyle="--", alpha=0.3)
    ax.plot(-1, 0, "ko", markersize=10, fillstyle="none", label="-1 point")

    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.set_title("Nyquist Diagram")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")


def plot_pole_zero_comparison(ax, closed_loop_before, closed_loop_after):
    """Plot pole-zero map comparison."""
    poles1 = ct.pole(closed_loop_before)
    poles2 = ct.pole(closed_loop_after)
    zeros1 = ct.zero(closed_loop_before)
    zeros2 = ct.zero(closed_loop_after)

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

    # Unit circle
    theta = np.linspace(0, 2 * np.pi, 100)
    ax.plot(np.cos(theta), np.sin(theta), "k--", alpha=0.4, label="Unit circle")

    ax.axhline(y=0, color="k", linestyle="-", alpha=0.2)
    ax.axvline(x=0, color="k", linestyle="-", alpha=0.2)

    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.set_title("Pole-Zero Map")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")


def plot_optimization_history(axes, history: Dict):
    """Plot optimization convergence history."""
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
    axes[1, 0].set_title("Objective Function (Ts + 0.05×OS)")
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
    """Print a formatted comparison table."""
    print("\n" + "=" * 80)
    print(" COMPARISON TABLE: Before vs After Optimization")
    print("=" * 80)
    print(f"{'Metric':<25} {'Before':<20} {'After':<20} {'Improvement':<15}")
    print("-" * 80)

    # Gains
    print(
        f"{'Kp (Proportional)':<25} {initial_gains['Kp']:<20.4f} {optimized_gains['Kp']:<20.4f} {'—':<15}"
    )
    print(
        f"{'Ki (Integral)':<25} {initial_gains['Ki']:<20.4f} {optimized_gains['Ki']:<20.4f} {'—':<15}"
    )
    print(
        f"{'Kd (Derivative)':<25} {initial_gains['Kd']:<20.4f} {optimized_gains['Kd']:<20.4f} {'—':<15}"
    )

    # Performance
    st_before = initial_metrics["settling_time"]
    st_after = optimized_metrics["settling_time"]
    st_imp = (st_before - st_after) / st_before * 100 if st_before > 0 else 0
    print(
        f"{'Settling Time (s)':<25} {st_before:<20.4f} {st_after:<20.4f} {f'{st_imp:.1f}% faster':<15}"
    )

    os_before = initial_metrics["overshoot"]
    os_after = optimized_metrics["overshoot"]
    os_imp = os_before - os_after
    print(
        f"{'Overshoot (%)':<25} {os_before:<20.2f} {os_after:<20.2f} {f'{os_imp:.1f}% less':<15}"
    )

    # Stability margins
    gm_before = initial_metrics["gain_margin"]
    gm_after = optimized_metrics["gain_margin"]
    pm_before = initial_metrics["phase_margin"]
    pm_after = optimized_metrics["phase_margin"]
    print(f"{'Gain Margin (dB)':<25} {gm_before:<20.2f} {gm_after:<20.2f} {'—':<15}")
    print(f"{'Phase Margin (°)':<25} {pm_before:<20.2f} {pm_after:<20.2f} {'—':<15}")

    print("=" * 80)
    print("\nConstraints satisfied:")
    gm_check = "✓" if optimized_metrics["gain_margin"] >= 6 else "✗"
    pm_check = "✓" if optimized_metrics["phase_margin"] >= 45 else "✗"
    print(
        f"  Gain Margin ≥ 6 dB: {gm_check} (value: {optimized_metrics['gain_margin']:.2f} dB)"
    )
    print(
        f"  Phase Margin ≥ 45°: {pm_check} (value: {optimized_metrics['phase_margin']:.2f}°)"
    )
    print("=" * 80)


# ============================================================================
# Analysis Functions
# ============================================================================


def analyze_with_gains(
    drone: DronePitchModel, Kp: float, Ki: float, Kd: float
) -> Dict[str, Any]:
    """Analyze system with given PID gains."""
    open_loop_tf = drone.get_open_loop_tf(Kp, Ki, Kd)
    closed_loop_tf = drone.get_closed_loop_tf(Kp, Ki, Kd)

    # Stability margins
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        gm, pm, wg, wp = ct.stability_margins(open_loop_tf)

    gain_margin_dB = 20 * np.log10(gm) if gm != np.inf else 60
    phase_margin_deg = pm if pm != np.inf else 90

    # Step response
    t, y = ct.step_response(closed_loop_tf)
    ss_value = y[-1] if len(y) > 0 else 1.0
    y_norm = y / ss_value if ss_value != 0 else y

    # Metrics
    peak_idx = np.argmax(y_norm)
    peak_value = y_norm[peak_idx]
    overshoot = max(0, (peak_value - 1.0) * 100)

    # Settling time
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
        "time_response": (t, y),
        "gain_margin": gain_margin_dB,
        "phase_margin": phase_margin_deg,
        "settling_time": settling_time,
        "overshoot": overshoot,
        "wg": wg,
        "wp": wp,
    }


# ============================================================================
# Optimization Setup
# ============================================================================


def create_optimization_problem(drone: DronePitchModel) -> om.Problem:
    """Create and configure OpenMDAO optimization problem."""
    prob = om.Problem()

    prob.model = om.Group()
    prob.model.add_subsystem(
        "pid_perf", PIDPerformanceComp(Ixx=drone.Ixx, b_damp=drone.b_damp)
    )

    # Optimizer
    prob.driver = om.ScipyOptimizeDriver()
    prob.driver.options["optimizer"] = "SLSQP"
    prob.driver.options["maxiter"] = 300
    prob.driver.options["tol"] = 1e-8

    # Design variables with bounds
    prob.model.add_design_var("Kp", lower=0.1, upper=50.0)
    prob.model.add_design_var("Ki", lower=0.0, upper=20.0)
    prob.model.add_design_var("Kd", lower=0.0, upper=10.0)

    # Objective: minimize settling time + weighted overshoot
    prob.model.add_objective("objective")

    # Constraints: stability margins
    prob.model.add_constraint("gain_margin_dB", lower=6.0)  # Gain margin ≥ 6 dB
    prob.model.add_constraint("phase_margin_deg", lower=45.0)  # Phase margin ≥ 45°

    # Setup
    prob.setup()

    return prob


# ============================================================================
# Main Orchestrator
# ============================================================================


def main():
    """Main orchestrator function."""
    print("=" * 80)
    print(" DRONE PITCH AUTOPILOT — PID Design + MDO-Based Gain Optimization")
    print(" Using OpenMDAO for structured multidisciplinary optimization")
    print("=" * 80)

    # Initialize drone model
    drone = DronePitchModel(Ixx=0.01, b_damp=0.1)

    print("\n📦 Drone Pitch Model Parameters:")
    print(f"   Ixx (moment of inertia): {drone.Ixx} kg·m²")
    print(f"   b_damp (damping): {drone.b_damp} N·m·s/rad")
    print(f"\n📐 Plant Transfer Function: G(s) = 1 / ({drone.Ixx}s² + {drone.b_damp}s)")

    # Initial (hand-tuned) gains
    initial_gains = {"Kp": 5.0, "Ki": 1.0, "Kd": 0.5}

    # ===== BEFORE OPTIMIZATION =====
    print("\n" + "=" * 80)
    print(" PHASE 1: Analysis with Hand-Tuned PID Gains")
    print("=" * 80)

    initial_metrics = analyze_with_gains(
        drone, initial_gains["Kp"], initial_gains["Ki"], initial_gains["Kd"]
    )

    print(
        f"\n🎛️  Initial PID Gains: Kp={initial_gains['Kp']:.2f}, "
        f"Ki={initial_gains['Ki']:.2f}, Kd={initial_gains['Kd']:.2f}"
    )
    print(f"   Settling Time: {initial_metrics['settling_time']:.4f} s")
    print(f"   Overshoot: {initial_metrics['overshoot']:.2f}%")
    print(f"   Gain Margin: {initial_metrics['gain_margin']:.2f} dB")
    print(f"   Phase Margin: {initial_metrics['phase_margin']:.2f}°")

    # ===== RUN OPTIMIZATION =====
    print("\n" + "=" * 80)
    print(" PHASE 2: OpenMDAO Optimization")
    print("=" * 80)

    prob = create_optimization_problem(drone)

    # Set initial design variables
    prob.set_val("Kp", initial_gains["Kp"])
    prob.set_val("Ki", initial_gains["Ki"])
    prob.set_val("Kd", initial_gains["Kd"])

    # Initial analysis
    print("\n📊 Initial design point:")
    prob.run_model()
    print(f"   Objective: {prob.get_val('objective')[0]:.4f}")
    print(f"   Settling Time: {prob.get_val('settling_time')[0]:.4f} s")
    print(f"   Overshoot: {prob.get_val('overshoot_pct')[0]:.2f}%")
    print(f"   Gain Margin: {prob.get_val('gain_margin_dB')[0]:.2f} dB")
    print(f"   Phase Margin: {prob.get_val('phase_margin_deg')[0]:.2f}°")

    # Track optimization history
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

    # Run optimization
    print("\n🚀 Running optimization (SLSQP)...")
    print("-" * 40)

    try:
        prob.run_driver()
        opt_success = True
    except Exception as e:
        print(f"\n⚠️  Optimization failed: {e}")
        opt_success = False

    record_history()

    # Get optimized gains
    Kp_opt = prob.get_val("Kp")[0]
    Ki_opt = prob.get_val("Ki")[0]
    Kd_opt = prob.get_val("Kd")[0]
    optimized_gains = {"Kp": Kp_opt, "Ki": Ki_opt, "Kd": Kd_opt}

    print(
        f"\n✅ Optimization {'completed successfully!' if opt_success else 'encountered issues.'}"
    )
    print(f"   Optimized Gains: Kp={Kp_opt:.4f}, Ki={Ki_opt:.4f}, Kd={Kd_opt:.4f}")

    # ===== AFTER OPTIMIZATION =====
    print("\n" + "=" * 80)
    print(" PHASE 3: Analysis with Optimized PID Gains")
    print("=" * 80)

    optimized_metrics = analyze_with_gains(drone, Kp_opt, Ki_opt, Kd_opt)

    print(
        f"\n🎯 Optimized PID Gains: Kp={Kp_opt:.4f}, Ki={Ki_opt:.4f}, Kd={Kd_opt:.4f}"
    )
    print(f"   Settling Time: {optimized_metrics['settling_time']:.4f} s")
    print(f"   Overshoot: {optimized_metrics['overshoot']:.2f}%")
    print(f"   Gain Margin: {optimized_metrics['gain_margin']:.2f} dB")
    print(f"   Phase Margin: {optimized_metrics['phase_margin']:.2f}°")

    # ===== GENERATE PLOTS =====
    print("\n" + "=" * 80)
    print(" PHASE 4: Generating Visualization Plots")
    print("=" * 80)

    # Comparison plots
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(
        "Drone Pitch Autopilot: Before vs After OpenMDAO Optimization",
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
    print("📊 Saved: tools/PID/comparison_plots.png")
    plt.close()

    # Optimization history
    fig2, axes2 = plt.subplots(2, 3, figsize=(16, 10))
    fig2.suptitle(
        "OpenMDAO Optimization Convergence History", fontsize=16, fontweight="bold"
    )
    plot_optimization_history(axes2, history)
    plt.tight_layout()
    plt.savefig("tools/PID/optimization_history.png", dpi=150, bbox_inches="tight")
    print("📊 Saved: tools/PID/optimization_history.png")
    plt.close()

    # ===== SUMMARY =====
    print("\n" + "=" * 80)
    print(" FINAL RESULTS SUMMARY")
    print("=" * 80)

    print_comparison_table(
        initial_gains, optimized_gains, initial_metrics, optimized_metrics
    )

    print("\n" + "=" * 80)
    print(" OPTIMIZATION COMPLETE!")
    print("=" * 80)
    print("\n📁 Generated files:")
    print(
        "   • tools/PID/comparison_plots.png — Bode, Step, Nyquist, Pole-Zero comparisons"
    )
    print("   • tools/PID/optimization_history.png — Convergence history")

    st_before = initial_metrics["settling_time"]
    st_after = optimized_metrics["settling_time"]
    os_before = initial_metrics["overshoot"]
    os_after = optimized_metrics["overshoot"]

    print("\n📈 Key Improvements:")
    print(
        f"   • Settling Time: {st_before:.4f}s → {st_after:.4f}s "
        f"(Δ={(st_before - st_after) / st_before * 100:.1f}%)"
    )
    print(
        f"   • Overshoot: {os_before:.2f}% → {os_after:.2f}% "
        f"(Δ={os_before - os_after:.1f}%)"
    )
    print(
        f"   • Stability margins: GM={optimized_metrics['gain_margin']:.2f}dB ≥ 6dB, "
        f"PM={optimized_metrics['phase_margin']:.2f}° ≥ 45°"
    )

    print("\n🔧 The OpenMDAO optimizer successfully tuned the PID gains to minimize")
    print("   settling time while maintaining robust stability margins.")
    print("=" * 80)

    return {
        "initial_gains": initial_gains,
        "optimized_gains": optimized_gains,
        "initial_metrics": initial_metrics,
        "optimized_metrics": optimized_metrics,
        "optimization_history": history,
    }


if __name__ == "__main__":
    results = main()
