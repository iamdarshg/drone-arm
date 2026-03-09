Here is the project: **Drone Pitch Autopilot — PID Design + MDO-Based Gain Optimization**.

The two libraries don't just coexist here — they *integrate*. OpenMDAO drives the optimizer, and the `control` library lives *inside* an OpenMDAO component as the physics evaluator. This mirrors exactly how RTX/Boeing/NASA use OpenMDAO: a discipline analysis tool (here, control theory) is wrapped as a component, and a trade study optimizer sits on top.

***

## The Concept

Model a quadrotor's linearized pitch dynamics as a second-order transfer function, design a PID controller on top of it, then use OpenMDAO to *optimally tune the PID gains* subject to stability margin constraints.

**Why this is realistic:** Linearized pitch dynamics of a drone in hover can be written as:

$$
G(s) = \frac{1}{I_{xx} s^2 + b_{damp} \cdot s}
$$

where $I_{xx}$ is the moment of inertia about the pitch axis and $b_{damp}$ is aerodynamic damping. A PID controller in the Laplace domain is:

$$
C(s) = K_p + \frac{K_i}{s} + K_d s = \frac{K_d s^2 + K_p s + K_i}{s}
$$

The open-loop plant is $L(s) = C(s) \cdot G(s)$, and the closed-loop response is $T(s) = \frac{L(s)}{1 + L(s)}$.

***

## File Structure

```
PID/
├── drone_model.py       # TF definition + stability analysis using control lib
├── pid_component.py     # OpenMDAO ExplicitComponent wrapping control library
├── optimization.py      # OpenMDAO Problem setup, optimizer, constraints
├── plots.py             # Bode, Nyquist, pole-zero, step response figures
├── main.py              # Orchestrator: analyze → optimize → compare
└── TODO.md              # This file you're reading now
```


***

## Part 1 — `control` Library (drone_model.py + plots.py)

This is the pure control-systems showcase.[^1][^2]

- **Transfer function:** `control.tf([^1], [Ixx, b, 0])` — the denominator `s(Ixx·s + b)` captures the double-integrator-like pitch physics
- **PID as a TF:** `control.tf([Kd, Kp, Ki], [1, 0])` — numerator is the PID polynomial, denominator is `s`
- **Closed-loop:** `control.feedback(C * G, 1)` — unity feedback
- **Bode plot:** `control.bode_plot(open_loop_tf)` — magnitude + phase vs. frequency; you read off gain/phase margins visually[^2]
- **Nyquist diagram:** `control.nyquist_plot(open_loop_tf)` — encirclements of -1 tell you Nyquist stability criterion[^3]
- **Pole-zero map:** `control.pzmap(closed_loop_tf)` — visualize where roots land in the complex plane
- **Step response:** `control.step_response(closed_loop_tf)` — read off settling time and overshoot
- **Margins:** `control.stability_margins(open_loop_tf)` — returns `(gm, pm, wg, wp)` numerically

This alone covers every bullet in your requirement list.

***

## Part 2 — OpenMDAO (pid_component.py + optimization.py)

Wrap the control-library analysis as an OpenMDAO `ExplicitComponent`.[^4][^5]

```
class PIDPerformanceComp(om.ExplicitComponent):
    setup():
        add_input:  Kp, Ki, Kd
        add_output: settling_time, overshoot_pct, gain_margin_dB, phase_margin_deg
        declare_partials('*', '*', method='fd')   # finite-diff, no need for analytic grad

    compute(inputs, outputs):
        # Build TFs using control library
        # Run step_response → extract settling_time, overshoot
        # Run stability_margins → extract gm, pm
        # Populate outputs dict
```

Then the optimization problem:


| MDO Element | Value |
| :-- | :-- |
| **Design variables** | `Kp ∈ [0.1, 50]`, `Ki ∈ [0, 20]`, `Kd ∈ [0, 10]` |
| **Objective** | minimize `settling_time + 0.05 × overshoot_pct` |
| **Constraint 1** | `gain_margin_dB ≥ 6 dB` (robustness floor) |
| **Constraint 2** | `phase_margin_deg ≥ 45°` (standard aerospace requirement) |
| **Optimizer** | `ScipyOptimizeDriver` with `SLSQP` |


***

## What the Final Output Shows

**Before optimization** (hand-tuned initial gains, e.g. Kp=5, Ki=1, Kd=0.5):

- Bode plot with gain/phase margins marked
- Nyquist with -1 point reference
- Step response showing sluggish or oscillatory behavior

**After OpenMDAO optimization:**

- Same four plots, but now settling time is minimized and margins are satisfied
- A printed table comparing before/after: gains, settling time, overshoot, GM, PM

This gives you a **side-by-side story** — the `control` library provides the domain physics, and OpenMDAO provides the structured optimizer that no manual Ziegler-Nichols tuning can match.

***

## Time Estimate

| Task | Time |
| :-- | :-- |
| `drone_model.py` + `plots.py` | ~30 min |
| `pid_component.py` | ~25 min |
| `optimization.py` + `main.py` | ~20 min |
| Debugging + plot formatting | ~15 min |
| **Total** | **~90 min** |

The biggest gotcha to watch for: `control.stability_margins()` can return `inf` for gain margin if the phase never crosses −180°, so add a clamp or catch on the output before passing into OpenMDAO. Also use `method='fd'` (finite difference) for partials — trying to compute analytic derivatives through the `control` library is not worth the effort for a 1-2 hour project.
<span style="display:none">[^10][^11][^12][^13][^14][^15][^6][^7][^8][^9]</span>

<div align="center">⁂</div>

[^1]: https://python-control.readthedocs.io/en/latest/generated/control.TransferFunction.html

[^2]: https://python-control.readthedocs.io/en/latest/generated/control.bode_plot.html

[^3]: https://www.youtube.com/watch?v=ak3_-wRDIMQ

[^4]: https://openmdao.org/newdocs/versions/latest/features/core_features/working_with_components/explicit_component.html

[^5]: https://modopt.readthedocs.io/en/latest/src/_temp/examples/ex_5quartic_opt_openmdao.html

[^7]: https://openmdao.org/newdocs/versions/latest/_srcdocs/packages/core/explicitcomponent.html

[^8]: https://www.youtube.com/watch?v=SzHOShB6d3E

[^9]: https://snyk.io/advisor/python/openmdao/functions/openmdao.api.ExplicitComponent

[^10]: https://www.youtube.com/watch?v=J-UbVnhsyJ0

[^11]: https://ntrs.nasa.gov/api/citations/20180004467/downloads/20180004467.pdf

[^13]: https://www.youtube.com/watch?v=yHlC1pG_Dmw

[^14]: https://github.com/python-control/python-control/blob/main/examples/bode-and-nyquist-plots.ipynb

[^15]: https://openmdao.github.io/PracticalMDO/Notebooks/Optimization/debugging_your_optimizations.html
