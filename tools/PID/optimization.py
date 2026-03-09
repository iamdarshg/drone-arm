"""
OpenMDAO Optimization Setup for PID Tuning
============================================
This module sets up the optimization problem using OpenMDAO to optimally
tune PID gains subject to stability margin constraints.
"""

import openmdao.api as om
from pid_component import PIDPerformanceComp


def create_optimization_problem():
    """
    Create and configure the OpenMDAO optimization problem.

    Returns:
        om.Problem: Configured optimization problem ready for execution
    """
    # Create the problem instance
    prob = om.Problem()

    # Initialize the model
    prob.model = om.Group()

    # Add the PID performance component
    prob.model.add_subsystem("pid_perf", PIDPerformanceComp(), promotes_inputs=["*"])

    # Define the optimizer
    prob.driver = om.ScipyOptimizeDriver()
    prob.driver.options["optimizer"] = "SLSQP"
    prob.driver.options["maxiter"] = 500
    prob.driver.options["tol"] = 1e-8

    # Add design variables with bounds
    prob.model.add_design_var("Kp", lower=0.1, upper=50.0)
    prob.model.add_design_var("Ki", lower=0.0, upper=20.0)
    prob.model.add_design_var("Kd", lower=0.0, upper=10.0)

    # Objective: minimize settling time + weighted overshoot
    prob.model.add_objective("settling_time", scaler=1.0)

    # Constraints: stability margins (robustness requirements)
    prob.model.add_constraint("gain_margin_dB", lower=6.0)  # Gain margin >= 6 dB
    prob.model.add_constraint("phase_margin_deg", lower=45.0)  # Phase margin >= 45°

    # Add recorder for optimization history
    recorder = om.SqliteRecorder("pid_optimization_history.sql")
    prob.driver.add_recorder(recorder)
    prob.driver.recording_options["record_desvars"] = True
    prob.driver.recording_options["record_objectives"] = True
    prob.driver.recording_options["record_constraints"] = True

    # Setup the problem
    prob.setup()

    return prob


def run_optimization(initial_gains=None):
    """
    Run the optimization process with optional initial gains.

    Args:
        initial_gains (dict, optional): Dictionary with initial Kp, Ki, Kd values.
                                        Defaults to hand-tuned values.

    Returns:
        tuple: (Problem instance, optimization success status)
    """
    if initial_gains is None:
        initial_gains = {"Kp": 5.0, "Ki": 1.0, "Kd": 0.5}

    # Create and setup the problem
    prob = create_optimization_problem()

    # Set initial design variables
    prob.set_val("Kp", initial_gains["Kp"])
    prob.set_val("Ki", initial_gains["Ki"])
    prob.set_val("Kd", initial_gains["Kd"])

    # Run initial analysis (before optimization)
    print("\n" + "=" * 60)
    print("INITIAL ANALYSIS (Before Optimization)")
    print("=" * 60)
    prob.run_model()

    # Print initial values
    print(
        f"\nInitial PID Gains: Kp={prob.get_val('Kp')[0]:.3f}, "
        f"Ki={prob.get_val('Ki')[0]:.3f}, Kd={prob.get_val('Kd')[0]:.3f}"
    )
    print(f"  Settling Time:   {prob.get_val('settling_time')[0]:.4f} s")
    print(f"  Overshoot:       {prob.get_val('overshoot_pct')[0]:.2f} %")
    print(f"  Gain Margin:     {prob.get_val('gain_margin_dB')[0]:.2f} dB")
    print(f"  Phase Margin:    {prob.get_val('phase_margin_deg')[0]:.2f} deg")

    # Run optimization
    print("\n" + "=" * 60)
    print("STARTING OPTIMIZATION")
    print("=" * 60)

    try:
        prob.run_driver()
        optimization_success = True
    except Exception as e:
        print(f"\nOptimization failed with error: {e}")
        optimization_success = False

    # Print optimized results
    print("\n" + "=" * 60)
    print("OPTIMIZED RESULTS (After Optimization)")
    print("=" * 60)
    print(
        f"\nOptimized PID Gains: Kp={prob.get_val('Kp')[0]:.3f}, "
        f"Ki={prob.get_val('Ki')[0]:.3f}, Kd={prob.get_val('Kd')[0]:.3f}"
    )
    print(f"  Settling Time:   {prob.get_val('settling_time')[0]:.4f} s")
    print(f"  Overshoot:       {prob.get_val('overshoot_pct')[0]:.2f} %")
    print(f"  Gain Margin:     {prob.get_val('gain_margin_dB')[0]:.2f} dB")
    print(f"  Phase Margin:    {prob.get_val('phase_margin_deg')[0]:.2f} deg")

    return prob, optimization_success
