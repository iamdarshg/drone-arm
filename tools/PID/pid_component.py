"""
OpenMDAO ExplicitComponent wrapper for PID controller performance evaluation.

This component wraps the control library analysis, taking PID gains as inputs
and computing stability/performance metrics as outputs for OpenMDAO optimization.
"""

import numpy as np
import openmdao.api as om
import control as ct


class PIDPerformanceComp(om.ExplicitComponent):
    """
    OpenMDAO component that evaluates PID controller performance.

    Takes PID gains as inputs and computes:
    - settling_time: Step response settling time
    - overshoot_pct: Percent overshoot
    - gain_margin_dB: Gain margin in decibels
    - phase_margin_deg: Phase margin in degrees
    """

    def initialize(self):
        """Initialize component with drone parameters."""
        self.options.declare('Ixx', default=0.1, desc='Moment of inertia about pitch axis (kg·m²)')
        self.options.declare('b_damp', default=0.5, desc='Aerodynamic damping coefficient (N·m·s/rad)')

    def setup(self):
        """Set up inputs and outputs."""
        Ixx = self.options['Ixx']
        b_damp = self.options['b_damp']

        # Inputs: PID gains
        self.add_input('Kp', val=5.0, desc='Proportional gain')
        self.add_input('Ki', val=1.0, desc='Integral gain')
        self.add_input('Kd', val=0.5, desc='Derivative gain')

        # Outputs: Performance metrics
        self.add_output('settling_time', val=0.0, desc='Step response settling time (s)')
        self.add_output('overshoot_pct', val=0.0, desc='Percent overshoot (%)')
        self.add_output('gain_margin_dB', val=20.0, desc='Gain margin (dB)')
        self.add_output('phase_margin_deg', val=60.0, desc='Phase margin (degrees)')
        self.add_output('closed_loop_poles', shape=(3,), desc='Closed-loop poles')
        self.add_output('bandwidth_rad_s', val=0.0, desc='Control bandwidth (rad/s)')

        # Declare finite difference partials (no analytic derivatives through control lib)
        self.declare_partials('*', '*', method='fd')

    def compute(self, inputs, outputs):
        """
        Compute performance metrics from PID gains.

        Uses the control library to build transfer functions and analyze stability.
        """
        Kp = inputs['Kp']
        Ki = inputs['Ki']
        Kd = inputs['Kd']
        Ixx = self.options['Ixx']
        b_damp = self.options['b_damp']

        try:
            # Build plant transfer function: G(s) = 1 / (Ixx*s^2 + b_damp*s)
            # Denominator is s*(Ixx*s + b_damp) = Ixx*s^2 + b_damp*s
            plant_tf = ct.TransferFunction([1], [Ixx, b_damp, 0])

            # Build PID controller transfer function: C(s) = (Kd*s^2 + Kp*s + Ki) / s
            pid_tf = ct.TransferFunction([Kd, Kp, Ki], [1, 0])

            # Open-loop transfer function: L(s) = C(s) * G(s)
            open_loop_tf = ct.series(pid_tf, plant_tf)

            # Closed-loop transfer function: T(s) = L(s) / (1 + L(s))
            closed_loop_tf = ct.feedback(open_loop_tf, 1)

            # Compute stability margins
            gm, pm, wg, wp = ct.stability_margins(open_loop_tf)

            # Handle infinite gain margin (phase never crosses -180°)
            if np.isinf(gm):
                gain_margin_dB = 40.0  # Cap at a reasonable high value
            else:
                gain_margin_dB = 20 * np.log10(gm)

            # Handle infinite phase margin
            if np.isinf(pm):
                phase_margin_deg = 90.0  # Cap at 90 degrees
            else:
                phase_margin_deg = pm

            # Clamp margins to reasonable bounds for optimizer
            gain_margin_dB = np.clip(gain_margin_dB, 0.1, 100.0)
            phase_margin_deg = np.clip(phase_margin_deg, 1.0, 90.0)

            # Compute step response characteristics
            t, y = ct.step_response(closed_loop_tf)

            # Normalize step response to start at 0 and end at 1
            y = y - y[0]
            final_value = y[-1]
            if final_value != 0:
                y = y / final_value

            # Calculate percent overshoot
            peak_value = np.max(y)
            overshoot_pct = max(0.0, (peak_value - 1.0) * 100.0)

            # Calculate settling time (time to reach and stay within 2% of final value)
            tolerance = 0.02
            settled_indices = np.where(np.abs(y - 1.0) <= tolerance)[0]
            if len(settled_indices) > 0:
                # Find the last time we exited the tolerance band
                # Settling time is when we first enter and stay within tolerance
                for i in range(len(t) - 1, -1, -1):
                    if np.abs(y[i] - 1.0) > tolerance:
                        if i + 1 < len(t):
                            settling_time = t[i + 1]
                        else:
                            settling_time = t[-1]
                        break
                else:
                    settling_time = t[-1]  # Never exited tolerance
            else:
                settling_time = t[-1]  # Never settled within tolerance

            # Get closed-loop poles
            poles = ct.pole(closed_loop_tf)
            # Sort by real part for consistent ordering
            sorted_poles = sorted(poles, key=lambda p: (np.real(p), np.imag(p)))
            closed_loop_poles = np.array([complex(p) for p in sorted_poles])

            # Estimate bandwidth from Bode plot (frequency where magnitude drops to -3dB)
            try:
                omega, mag, phase = ct.bode_plot(open_loop_tf, plot=False)
                # Find bandwidth: frequency where open-loop gain is 1 (0 dB)
                # For a unity feedback system, closed-loop bandwidth is approximately
                # where open-loop gain is 1
                idx = np.argmin(np.abs(mag - 1.0))
                if np.abs(mag[idx] - 1.0) < 5.0:  # Only if we cross 0 dB
                    bandwidth_rad_s = omega[idx]
                else:
                    bandwidth_rad_s = omega[-1]  # Use max frequency if no crossing
            except:
                bandwidth_rad_s = 0.0

            # Populate outputs
            outputs['settling_time'] = settling_time
            outputs['overshoot_pct'] = overshoot_pct
            outputs['gain_margin_dB'] = gain_margin_dB
            outputs['phase_margin_deg'] = phase_margin_deg
            outputs['closed_loop_poles'] = closed_loop_poles
            outputs['bandwidth_rad_s'] = bandwidth_rad_s

        except Exception as e:
            # If computation fails (e.g., unstable system), return worst-case values
            print(f"Warning: Computation failed for Kp={Kp}, Ki={Ki}, Kd={Kd}: {e}")
            outputs['settling_time'] = 10.0  # Very slow
            outputs['overshoot_pct'] = 100.0  # Maximum overshoot
            outputs['gain_margin_dB'] = 0.1  # Minimum margin
            outputs['phase_margin_deg'] = 1.0  # Minimum margin
            outputs['closed_loop_poles'] = np.array([0.0+0.0j, 0.0+0.0j, 0.0+0.0j])
            outputs['bandwidth_rad_s'] = 0.0


if __name__ == "__main__":
    # Simple test of the component
    prob = om.Problem()
    prob.model.add_subsystem('pid_perf', PIDPerformanceComp(Ixx=0.1, b_damp=0.5))

    prob.setup()
    prob.run_model()

    print("PID Performance Evaluation:")
    print(f"  Kp={prob['pid_perf.Kp']}, Ki={prob['pid_perf.Ki']}, Kd={prob['pid_perf.Kd']}")
    print(f"  Settling Time: {prob['pid_perf.settling_time']:.4f} s")
    print(f"  Overshoot: {prob['pid_perf.overshoot_pct']:.2f}%")
    print(f"  Gain Margin: {prob['pid_perf.gain_margin_dB']:.2f} dB")
    print(f"  Phase Margin: {prob['pid_perf.phase_margin_deg']:.2f}°")
    print(f"  Bandwidth: {prob['pid_perf.bandwidth_rad_s']:.4f} rad/s")
```
