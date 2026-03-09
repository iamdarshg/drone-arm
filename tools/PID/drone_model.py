# drone_model.py
"""
Drone Pitch Dynamics Model and Control Analysis
================================================

This module provides the transfer function model for a quadrotor's
linearized pitch dynamics and utilities for stability analysis.

Linearized pitch dynamics of a drone in hover:
    G(s) = 1 / (Ixx * s^2 + b_damp * s)

Where:
    Ixx = moment of inertia about pitch axis
    b_damp = aerodynamic damping coefficient
"""

from dataclasses import dataclass
from typing import Any, Tuple

import control as ct
import numpy as np


@dataclass
class DronePitchModel:
    """
    Drone Pitch Dynamics Model

    Represents the linearized pitch dynamics of a quadrotor in hover.
    The dynamics are modeled as a second-order transfer function:
        G(s) = 1 / (Ixx * s^2 + b_damp * s)
    """

    Ixx: float = 0.01  # kg*m^2, moment of inertia about pitch axis
    b_damp: float = 0.1  # N*m*s/rad, aerodynamic damping

    def get_plant_tf(self) -> ct.TransferFunction:
        """
        Get the pitch dynamics transfer function.

        Returns:
            Transfer function representing pitch dynamics
        """
        return ct.TransferFunction([1], [self.Ixx, self.b_damp, 0])

    @staticmethod
    def get_pid_tf(Kp: float, Ki: float, Kd: float) -> ct.TransferFunction:
        """
        Create a PID controller transfer function.

        C(s) = Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki) / s

        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain

        Returns:
            Transfer function representing PID controller
        """
        # PID numerator: Kd*s^2 + Kp*s + Ki
        numerator = [Kd, Kp, Ki]
        # PID denominator: s
        denominator = [1, 0]

        return ct.TransferFunction(numerator, denominator)

    @staticmethod
    def get_open_loop_tf(Kp: float, Ki: float, Kd: float) -> ct.TransferFunction:
        """
        Create the open-loop transfer function L(s) = C(s) * G(s).

        Args:
            Kp, Ki, Kd: PID gains

        Returns:
            Open-loop transfer function
        """
        G = ct.TransferFunction([1], [1, 0, 0])  # Placeholder, will use actual plant
        C = DronePitchModel.get_pid_tf(Kp, Ki, Kd)

        return ct.series(C, G)

    @staticmethod
    def get_closed_loop_tf(open_loop_tf: ct.TransferFunction) -> ct.TransferFunction:
        """
        Create the closed-loop transfer function T(s) = L(s) / (1 + L(s)).

        Uses unity negative feedback.

        Args:
            open_loop_tf: Open-loop transfer function

        Returns:
            Closed-loop transfer function with unity feedback
        """
        return ct.feedback(open_loop_tf, 1)

    @staticmethod
    def analyze_stability_margins(
        open_loop_tf: ct.TransferFunction,
    ) -> Tuple[float, float, float, float]:
        """
        Analyze stability margins for a transfer function.

        Args:
            open_loop_tf: Open-loop transfer function

        Returns:
            Tuple of (gain_margin_dB, phase_margin_deg, wg, wp)
        """
        gm, pm, wg, wp = ct.stability_margins(open_loop_tf)

        # Handle infinite margins
        if np.isinf(gm):
            gain_margin_dB = 60.0  # Cap at reasonable value
        else:
            gain_margin_dB = 20 * np.log10(gm)

        if np.isinf(pm):
            phase_margin_deg = 90.0  # Cap at 90 degrees
        else:
            phase_margin_deg = pm

        return gain_margin_dB, phase_margin_deg, float(wg), float(wp)

    @staticmethod
    def analyze_step_response(
        closed_loop_tf: ct.TransferFunction,
    ) -> Tuple[float, float]:
        """
        Analyze step response characteristics.

        Args:
            closed_loop_tf: Closed-loop transfer function

        Returns:
            Tuple of (settling_time, overshoot_pct)
        """
        t, y = ct.step_response(closed_loop_tf)

        # Ensure arrays
        t = np.asarray(t)
        y = np.asarray(y)

        # Normalize to start at 0 and end at 1
        y = y - y[0]
        final_value = y[-1]

        if final_value != 0:
            y_normalized = y / final_value
        else:
            y_normalized = y

        # Calculate percent overshoot
        peak_value = np.max(y_normalized)
        overshoot_pct = max(0.0, (peak_value - 1.0) * 100)

        # Calculate settling time (2% criterion)
        tolerance = 0.02
        within_band = np.abs(y_normalized - 1.0) <= tolerance

        settling_time = 0.0
        if np.any(within_band):
            # Find the last time we exited the tolerance band
            for i in range(len(within_band) - 1, -1, -1):
                if not within_band[i]:
                    if i + 1 < len(t):
                        settling_time = t[i + 1]
                    else:
                        settling_time = t[-1]
                    break
            else:
                settling_time = 0.0
        else:
            settling_time = t[-1]

        return settling_time, overshoot_pct

    def analyze(self, Kp: float, Ki: float, Kd: float) -> dict:
        """
        Perform complete system analysis.

        Convenience function that runs all analyses and returns results.

        Args:
            Kp, Ki, Kd: PID gains

        Returns:
            Dictionary containing all analysis results
        """
        # Build transfer functions
        plant_tf = self.get_plant_tf()
        pid_tf = self.get_pid_tf(Kp, Ki, Kd)
        open_loop_tf = ct.series(pid_tf, plant_tf)
        closed_loop_tf = ct.feedback(open_loop_tf, 1)

        # Analyze
        gm_dB, pm_deg, wg, wp = self.analyze_stability_margins(open_loop_tf)
        settling_time, overshoot_pct = self.analyze_step_response(closed_loop_tf)

        # Get poles and zeros
        poles = ct.pole(closed_loop_tf)
        zeros = ct.zero(closed_loop_tf)

        return {
            "plant_tf": plant_tf,
            "pid_tf": pid_tf,
            "open_loop_tf": open_loop_tf,
            "closed_loop_tf": closed_loop_tf,
            "gain_margin_dB": gm_dB,
            "phase_margin_deg": pm_deg,
            "gain_margin_freq": wg,
            "phase_margin_freq": wp,
            "settling_time": settling_time,
            "overshoot_pct": overshoot_pct,
            "poles": poles,
            "zeros": zeros,
        }


def analyze_system(
    Kp: float,
    Ki: float,
    Kd: float,
    Ixx: float = 0.01,
    b_damp: float = 0.1,
) -> dict:
    """
    Perform complete system analysis (backward compatibility wrapper).

    Args:
        Kp, Ki, Kd: PID gains
        Ixx: Moment of inertia (default: 0.01 kg*m^2)
        b_damp: Damping coefficient (default: 0.1 N*m*s/rad)

    Returns:
        Dictionary containing all analysis results
    """
    drone = DronePitchModel(Ixx=Ixx, b_damp=b_damp)
    return drone.analyze(Kp, Ki, Kd)


if __name__ == "__main__":
    # Quick demonstration
    print("=== Drone Pitch Dynamics Demo ===\n")

    # Create drone model with default parameters
    drone = DronePitchModel(Ixx=0.01, b_damp=0.1)

    # Get plant transfer function
    G = drone.get_plant_tf()
    print(f"Plant transfer function G(s):")
    print(f"  G(s) = 1 / ({drone.Ixx}*s² + {drone.b_damp}*s)")
    print(f"  Numerator: {G.num[0][0]}")
    print(f"  Denominator: {G.den[0][0]}")
    print()

    # Create PID controller with example gains
    Kp, Ki, Kd = 5.0, 1.0, 0.5
    C = drone.get_pid_tf(Kp, Ki, Kd)
    print(f"PID controller: C(s) = ({Kd}*s² + {Kp}*s + {Ki}) / s")
    print(f"  Numerator: {C.num[0][0]}")
    print(f"  Denominator: {C.den[0][0]}")
    print()

    # Analyze closed-loop system
    results = drone.analyze(Kp, Ki, Kd)

    print("=== Stability Analysis ===")
    print(
        f"Gain Margin: {results['gain_margin_dB']:.2f} dB @ {results['gain_margin_freq']:.2f} rad/s"
    )
    print(
        f"Phase Margin: {results['phase_margin_deg']:.2f}° @ {results['phase_margin_freq']:.2f} rad/s"
    )
    print()

    print("=== Step Response ===")
    print(f"Settling Time: {results['settling_time']:.4f} s")
    print(f"Overshoot: {results['overshoot_pct']:.2f}%")
    print()

    print("=== Poles/Zeros ===")
    print(f"Poles: {results['poles']}")
    print(f"Zeros: {results['zeros']}")
