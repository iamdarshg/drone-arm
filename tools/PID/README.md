# PID Controller Design Tool — Documentation

## Overview

This tool implements industry-standard techniques for designing and tuning PID controllers for drone pitch control, using OpenMDAO for multidisciplinary optimization. The methodology mirrors practices at aerospace companies like NASA, Boeing, and Lockheed Martin.

**Location**: `tools/PID/`
**Main Script**: `main.py`

---

## Quick Start

```bash
# Install dependencies
pip install control openmdao matplotlib numpy

# Run PID design and optimization
python tools/PID/main.py
```

**Output Files**:
- `tools/PID/comparison_plots.png` — Bode, step response, Nyquist, pole-zero comparisons
- `tools/PID/optimization_history.png` — Convergence history plots

---

## Control Theory Background

### What is a Control System?

A control system makes a system's output follow a desired reference. For a drone, we want the pitch angle to track pilot commands:

```
Reference ──►[ PID Controller ]──►┬──►[ Drone Plant ]──► Output
                                   │
                                   ▼
                                -[ Feedback ]
                                   │
                                   └────────────►
```

Where:
- **Controller C(s)**: The PID controller (what we design)
- **Plant G(s)**: The drone pitch dynamics (physics we must work with)
- **Feedback**: The measured output compared to reference

### The PID Controller Math

A PID controller produces an output based on three terms:

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de/dt
```

In the Laplace domain (frequency domain), this becomes:

```
C(s) = Kp + Ki/s + Kd·s = (Kd·s² + Kp·s + Ki) / s
```

**Breaking down each term (intuitively)**:

| Term | Formula | Effect | Trade-off |
|------|---------|--------|-----------|
| **Proportional** | Kp·e(t) | Response proportional to current error | Fast but leaves steady-state offset |
| **Integral** | Ki·∫e(τ)dτ | Eliminates accumulated error | Removes offset but can cause overshoot |
| **Derivative** | Kd·de/dt | Responds to error rate of change | Adds damping, reduces overshoot, noise-sensitive |

### Drone Pitch Dynamics (The Plant)

Linearizing around hover, a quadrotor's pitch dynamics follow Euler's rotation equation:

```
Ixx·θ̈ + b_damp·θ̇ = τ
```

Where:
- `Ixx` = Moment of inertia about pitch axis (kg·m²)
- `b_damp` = Aerodynamic damping (N·m·s/rad)
- `τ` = Motor torque input

Taking the Laplace transform (zero initial conditions):

```
Ixx·s²·Θ(s) + b_damp·s·Θ(s) = T(s)
G(s) = Θ(s)/T(s) = 1 / (Ixx·s² + b_damp·s)
```

This is a **second-order system** with a pole at `s = 0` (integrator) and a pole at `s = -b_damp/Ixx` (damping).

### Closed-Loop Transfer Function

With unity feedback H(s) = 1:

```
T(s) = L(s) / (1 + L(s))   where   L(s) = C(s)·G(s)
```

The closed-loop transfer function describes how output responds to reference changes. The **poles** of T(s) (roots of the denominator) determine system behavior:
- Poles in left half-plane (LHP) = stable
- Distance from origin = speed (rise time)
- Angle from negative real axis = overshoot tendency

---

## Stability Analysis

### What Makes a System Stable?

A linear system is stable if all poles have negative real parts (left half of the complex plane). Unstable systems oscillate with growing amplitude.

### Gain Margin (GM)

**Definition**: How much the loop gain can increase before the system becomes unstable.

**How to find it**:
1. Find the **phase crossover frequency** ω_pc where phase of L(jω) = -180°
2. Measure gain at that frequency
3. GM = 1 / |L(jω_pc)|, expressed in dB

**Industry standard**: GM ≥ 6 dB for aerospace applications

### Phase Margin (PM)

**Definition**: How much phase lag can be added before instability.

**How to find it**:
1. Find the **gain crossover frequency** ω_gc where |L(jω)| = 1 (0 dB)
2. Measure phase at that frequency
3. PM = 180° + ∠L(jω_gc)

**Industry standard**: PM ≥ 45° for aerospace applications

### Intuition

Think of a swing:
- **Gain margin**: How much harder you can push before losing control
- **Phase margin**: How much delay you can tolerate before the swing goes chaotic

Both margins provide **robustness** against modeling errors and parameter variations.

---

## The Optimization Problem

Tuning PID gains manually is difficult. We formulate it as a **constrained optimization problem**:

### Design Variables
- Kp ∈ [0.1, 50]
- Ki ∈ [0, 20]
- Kd ∈ [0, 10]

### Objective Function
```
Minimize: J = settling_time + 0.05 × overshoot_pct
```

The weight 0.05 means "1% overshoot is worth 0.05s of settling time" — balancing speed and overshoot.

### Constraints
- **Gain Margin**: GM ≥ 6 dB (robustness requirement)
- **Phase Margin**: PM ≥ 45° (robustness requirement)

### Why OpenMDAO?

OpenMDAO is NASA's open-source MDO framework used throughout the aerospace industry:
- Handles constrained optimization with gradients
- Provides convergence analysis
- Separates analysis (control theory) from optimization (numerical methods)
- Industry-proven reliability

---

## Step Response Metrics

When we apply a step input (instantaneous reference change), the response tells us about controller performance:

| Metric | Definition | Typical Desired Value |
|--------|------------|----------------------|
| **Rise Time** | Time from 10% to 90% of final value | Fast |
| **Settling Time** | Time to stay within 2% of final value | < 0.5s for drone pitch |
| **Overshoot** | How much response exceeds final value | < 10% typically |
| **Steady-State Error** | Final value minus reference | 0 (ideally) |

---

## File Structure

```
tools/PID/
├── README.md          # This file
├── main.py            # Main orchestrator, OpenMDAO component, plots
├── drone_model.py     # Drone pitch dynamics model
└── TODO.md            # Development notes
```

### `main.py` Components

1. **DronePitchModel** class: Pitch dynamics G(s) and PID controller C(s)
2. **PIDPerformanceComp**: OpenMDAO component computing metrics
3. **analyze_with_gains()**: Complete system analysis function
4. **plot_*** functions: Visualization of Bode, step, Nyquist, poles
5. **create_optimization_problem()**: OpenMDAO problem setup
6. **main()**: Orchestrator running the full workflow

### `drone_model.py` Components

1. **DronePitchModel** dataclass: Physical parameters (Ixx, b_damp)
2. Transfer function creation methods
3. Stability margin analysis
4. Step response metrics

---

## Troubleshooting

### Optimization Fails to Converge
- Initial gains may be in a poor region
- Try different initial values
- Check if constraints are too tight

### Gain/Phase Margins Show "inf"
- Phase never reaches -180° or gain never reaches 0 dB
- System is "very stable" in that sense
- Clamped to 60 dB / 90° in code

### Plots Show No Data
- Check matplotlib backend: `matplotlib.use('TkAgg')` if needed
- Ensure all dependencies installed

---

## References

1. **OpenMDAO Documentation**: https://openmdao.org/
2. **Python Control Library**: https://python-control.readthedocs.io/
3. **NASA MDO Practices**: https://ntrs.nasa.gov/api/citations/20180004467/downloads/20180004467.pdf

---

## Next Steps

To integrate with actual drone firmware:
1. Export optimized Kp, Ki, Kd values
2. Implement discrete-time PID in C (use bilinear/Tustin transform)
3. Add anti-windup logic for integral term
4. Test on hardware with safety limits