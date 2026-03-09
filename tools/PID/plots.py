"""
Plots module for Drone Pitch Autopilot PID analysis.
Provides visualization functions for control system analysis.
"""

import matplotlib.pyplot as plt
import numpy as np
from control import TransferFunction, bode_plot, nyquist_plot, pzmap, step_response


def plot_bode(open_loop_tf, closed_loop_tf=None, margins=None, title_suffix=""):
    """
    Generate Bode plot for open-loop transfer function.

    Args:
        open_loop_tf: Open-loop transfer function
        closed_loop_tf: Optional closed-loop TF for additional reference
        margins: Tuple of (gm, pm, wg, wp) for marking margins
        title_suffix: String to append to plot title
    """
    fig, (mag_ax, phase_ax) = plt.subplots(2, 1, figsize=(12, 8))

    # Plot open-loop Bode
    omega, mag, phase = bode_plot(open_loop_tf, plot=False)

    mag_ax.semilogx(omega, 20 * np.log10(mag), "b-", linewidth=2)
    phase_ax.semilogx(omega, np.degrees(phase), "b-", linewidth=2)

    # Mark gain margin
    if margins is not None:
        gm, pm, wg, wp = margins

        # Gain crossover frequency (0 dB crossing)
        gain_crossover_idx = np.argmin(np.abs(20 * np.log10(mag)))
        gain_crossover_freq = omega[gain_crossover_idx]
        gain_crossover_phase = np.degrees(phase[gain_crossover_idx])

        # Phase crossover frequency (-180° crossing)
        phase_crossover_idx = np.argmin(np.abs(np.degrees(phase) + 180))
        phase_crossover_freq = omega[phase_crossover_idx]
        phase_crossover_mag = 20 * np.log10(mag[phase_crossover_idx])

        # Mark gain margin
        if np.isfinite(gm):
            mag_ax.axhline(
                y=-gm, color="r", linestyle="--", alpha=0.7, label=f"GM = {gm:.1f} dB"
            )
            mag_ax.axvline(x=phase_crossover_freq, color="r", linestyle=":", alpha=0.5)
            mag_ax.plot(phase_crossover_freq, phase_crossover_mag, "ro", markersize=8)

        # Mark phase margin
        if np.isfinite(pm):
            phase_ax.axhline(
                y=-180,
                color="orange",
                linestyle="--",
                alpha=0.7,
                label=f"PM = {pm:.1f}°",
            )
            phase_ax.axvline(
                x=gain_crossover_freq, color="orange", linestyle=":", alpha=0.5
            )
            phase_ax.plot(gain_crossover_freq, gain_crossover_phase, "go", markersize=8)

        # Add legends
        mag_ax.legend(loc="best")
        phase_ax.legend(loc="best")

    # Mark 0 dB line and -180° line
    mag_ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
    phase_ax.axhline(y=-180, color="k", linestyle="-", alpha=0.3)

    mag_ax.set_ylabel("Magnitude (dB)", fontsize=12)
    phase_ax.set_ylabel("Phase (deg)", fontsize=12)
    phase_ax.set_xlabel("Frequency (rad/s)", fontsize=12)

    plt.suptitle(f"Bode Plot{title_suffix}", fontsize=14, fontweight="bold")
    plt.tight_layout()

    return fig


def plot_nyquist(open_loop_tf, title_suffix=""):
    """
    Generate Nyquist plot for open-loop transfer function.

    Args:
        open_loop_tf: Open-loop transfer function
        title_suffix: String to append to plot title
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    nyquist_plot(open_loop_tf, ax=ax)

    # Mark the -1 point
    ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)
    ax.axvline(x=-1, color="r", linestyle="--", alpha=0.7, label="-1 point")
    ax.plot(-1, 0, "ro", markersize=10, label="Critical point (-1, j0)")

    ax.set_xlabel("Real", fontsize=12)
    ax.set_ylabel("Imaginary", fontsize=12)
    ax.set_title(f"Nyquist Diagram{title_suffix}", fontsize=14, fontweight="bold")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    return fig


def plot_pole_zero(closed_loop_tf, title_suffix=""):
    """
    Generate pole-zero map for closed-loop transfer function.

    Args:
        closed_loop_tf: Closed-loop transfer function
        title_suffix: String to append to plot title
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    pzmap(closed_loop_tf, ax=ax)

    # Add unit circle for reference
    theta = np.linspace(0, 2 * np.pi, 100)
    ax.plot(np.cos(theta), np.sin(theta), "k--", alpha=0.5, label="Unit circle")

    ax.set_xlabel("Real", fontsize=12)
    ax.set_ylabel("Imaginary", fontsize=12)
    ax.set_title(f"Pole-Zero Map{title_suffix}", fontsize=14, fontweight="bold")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    return fig


def plot_step_response(
    closed_loop_tf,
    settling_time=None,
    overshoot=None,
    settling_time_opt=None,
    overshoot_opt=None,
    title_suffix="",
):
    """
    Generate step response plot for closed-loop transfer function.

    Args:
        closed_loop_tf: Closed-loop transfer function
        settling_time: Hand-tuned settling time marker
        overshoot: Hand-tuned overshoot marker
        settling_time_opt: Optimized settling time marker
        overshoot_opt: Optimized overshoot marker
        title_suffix: String to append to plot title
    """
    fig, ax = plt.subplots(figsize=(12, 6))

    t, y = step_response(closed_loop_tf)

    # Normalize response
    y = y / y[-1] if y[-1] != 0 else y

    ax.plot(t, y, "b-", linewidth=2, label="Step response")
    ax.axhline(y=1.0, color="k", linestyle="-", alpha=0.3)
    ax.axhline(y=0.95, color="g", linestyle="--", alpha=0.5, label="±5% band")
    ax.axhline(y=1.05, color="g", linestyle="--", alpha=0.5)
    ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)

    # Mark overshoot for initial
    if overshoot is not None:
        peak_idx = np.argmax(y)
        peak_time = t[peak_idx]
        peak_value = y[peak_idx]
        overshoot_pct = (peak_value - 1.0) * 100
        ax.plot(peak_time, peak_value, "ro", markersize=10)
        ax.annotate(
            f"Overshoot: {overshoot_pct:.1f}%",
            xy=(peak_time, peak_value),
            xytext=(peak_time + 0.5, peak_value - 0.1),
            fontsize=10,
            arrowprops=dict(arrowstyle="->", color="r"),
        )

    # Mark settling time for initial
    if settling_time is not None:
        ax.axvline(
            x=settling_time,
            color="orange",
            linestyle="--",
            alpha=0.7,
            label=f"Settling time: {settling_time:.3f}s",
        )

    ax.set_xlabel("Time (s)", fontsize=12)
    ax.set_ylabel("Amplitude", fontsize=12)
    ax.set_title(f"Step Response{title_suffix}", fontsize=14, fontweight="bold")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, t[-1]])
    ax.set_ylim([min(0, min(y) * 1.1), max(1.1, max(y) * 1.1)])

    return fig


def plot_comparison(
    closed_loop_before, closed_loop_after, metrics_before, metrics_after
):
    """
    Generate side-by-side comparison plot.

    Args:
        closed_loop_before: Before optimization closed-loop TF
        closed_loop_after: After optimization closed-loop TF
        metrics_before: Dict of metrics before optimization
        metrics_after: Dict of metrics after optimization
    """
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    # Step response comparison
    ax1 = axes[0, 0]
    t1, y1 = step_response(closed_loop_before)
    t2, y2 = step_response(closed_loop_after)
    ax1.plot(t1, y1, "b-", linewidth=2, label="Before (hand-tuned)")
    ax1.plot(t2, y2, "r-", linewidth=2, label="After (optimized)")
    ax1.axhline(y=1.0, color="k", linestyle="-", alpha=0.3)
    ax1.axhline(y=0.95, color="g", linestyle="--", alpha=0.5)
    ax1.axhline(y=1.05, color="g", linestyle="--", alpha=0.5)
    ax1.set_xlabel("Time (s)", fontsize=12)
    ax1.set_ylabel("Amplitude", fontsize=12)
    ax1.set_title("Step Response Comparison", fontsize=14, fontweight="bold")
    ax1.legend(loc="best")
    ax1.grid(True, alpha=0.3)

    # Bode comparison
    ax2 = axes[0, 1]
    omega1, mag1, phase1 = bode_plot(closed_loop_before, plot=False)
    omega2, mag2, phase2 = bode_plot(closed_loop_after, plot=False)
    ax2.semilogx(
        omega1, 20 * np.log10(mag1), "b-", linewidth=2, label="Before (hand-tuned)"
    )
    ax2.semilogx(
        omega2, 20 * np.log10(mag2), "r-", linewidth=2, label="After (optimized)"
    )
    ax2.set_xlabel("Frequency (rad/s)", fontsize=12)
    ax2.set_ylabel("Magnitude (dB)", fontsize=12)
    ax2.set_title("Bode Magnitude Comparison", fontsize=14, fontweight="bold")
    ax2.legend(loc="best")
    ax2.grid(True, alpha=0.3)

    # Pole-zero comparison
    ax3 = axes[1, 0]
    # Get poles and zeros for both
    poles_before = np.roots(closed_loop_before.den[0][0])
    zeros_before = np.roots(closed_loop_before.num[0][0])
    poles_after = np.roots(closed_loop_after.den[0][0])
    zeros_after = np.roots(closed_loop_after.num[0][0])

    ax3.plot(
        np.real(poles_before),
        np.imag(poles_before),
        "bx",
        markersize=10,
        label="Poles (before)",
    )
    ax3.plot(
        np.real(zeros_before),
        np.imag(zeros_before),
        "bo",
        markersize=8,
        label="Zeros (before)",
    )
    ax3.plot(
        np.real(poles_after),
        np.imag(poles_after),
        "rx",
        markersize=10,
        label="Poles (after)",
    )
    ax3.plot(
        np.real(zeros_after),
        np.imag(zeros_after),
        "ro",
        markersize=8,
        label="Zeros (after)",
    )

    # Unit circle
    theta = np.linspace(0, 2 * np.pi, 100)
    ax3.plot(np.cos(theta), np.sin(theta), "k--", alpha=0.5)
    ax3.axhline(y=0, color="k", linestyle="-", alpha=0.3)
    ax3.axvline(x=0, color="k", linestyle="-", alpha=0.3)
    ax3.set_xlabel("Real", fontsize=12)
    ax3.set_ylabel("Imaginary", fontsize=12)
    ax3.set_title("Pole-Zero Map Comparison", fontsize=14, fontweight="bold")
    ax3.legend(loc="best")
    ax3.grid(True, alpha=0.3)
    ax3.set_aspect("equal")

    # Metrics table
    ax4 = axes[1, 1]
    ax4.axis("off")

    table_data = [
        ["Metric", "Before", "After", "Improvement"],
        ["Kp", f"{metrics_before['Kp']:.3f}", f"{metrics_after['Kp']:.3f}", "-"],
        ["Ki", f"{metrics_before['Ki']:.3f}", f"{metrics_after['Ki']:.3f}", "-"],
        ["Kd", f"{metrics_before['Kd']:.3f}", f"{metrics_after['Kd']:.3f}", "-"],
        [
            "Settling Time (s)",
            f"{metrics_before['settling_time']:.4f}",
            f"{metrics_after['settling_time']:.4f}",
            f"{((metrics_before['settling_time'] - metrics_after['settling_time']) / metrics_before['settling_time'] * 100):.1f}%",
        ],
        [
            "Overshoot (%)",
            f"{metrics_before['overshoot']:.2f}",
            f"{metrics_after['overshoot']:.2f}",
            f"{(metrics_before['overshoot'] - metrics_after['overshoot']):.1f}%",
        ],
        [
            "Gain Margin (dB)",
            f"{metrics_before['gm']:.2f}",
            f"{metrics_after['gm']:.2f}",
            "-",
        ],
        [
            "Phase Margin (°)",
            f"{metrics_before['pm']:.2f}",
            f"{metrics_after['pm']:.2f}",
            "-",
        ],
    ]

    table = ax4.table(
        cellText=table_data,
        loc="center",
        cellLoc="center",
        colWidths=[0.25, 0.2, 0.2, 0.2],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1.2, 1.8)

    # Style header row
    for i in range(4):
        table[(0, i)].set_facecolor("#4472C4")
        table[(0, i)].set_text_props(color="white", fontweight="bold")

    ax4.set_title(
        "Performance Metrics Comparison", fontsize=14, fontweight="bold", pad=20
    )

    plt.suptitle("PID Optimization Results", fontsize=16, fontweight="bold", y=1.02)
    plt.tight_layout()

    return fig


def plot_optimization_history(histories):
    """
    Plot optimization history for all design variables and objectives.

    Args:
        histories: Dict containing 'Kp', 'Ki', 'Kd', 'objective', 'constraint_violation'
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))

    # Design variables
    axes[0, 0].plot(histories["Kp"], "b-", linewidth=2)
    axes[0, 0].set_xlabel("Iteration")
    axes[0, 0].set_ylabel("Kp")
    axes[0, 0].set_title("Kp Evolution")
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].plot(histories["Ki"], "g-", linewidth=2)
    axes[0, 1].set_xlabel("Iteration")
    axes[0, 1].set_ylabel("Ki")
    axes[0, 1].set_title("Ki Evolution")
    axes[0, 1].grid(True, alpha=0.3)

    axes[0, 2].plot(histories["Kd"], "r-", linewidth=2)
    axes[0, 2].set_xlabel("Iteration")
    axes[0, 2].set_ylabel("Kd")
    axes[0, 2].set_title("Kd Evolution")
    axes[0, 2].grid(True, alpha=0.3)

    # Objective
    axes[1, 0].plot(histories["objective"], "purple", linewidth=2)
    axes[1, 0].set_xlabel("Iteration")
    axes[1, 0].set_ylabel("Objective Value")
    axes[1, 0].set_title("Objective Function")
    axes[1, 0].grid(True, alpha=0.3)

    # Constraints
    axes[1, 1].plot(histories["gm"], "b-", linewidth=2, label="GM (dB)")
    axes[1, 1].axhline(y=6, color="r", linestyle="--", alpha=0.7, label="GM constraint")
    axes[1, 1].set_xlabel("Iteration")
    axes[1, 1].set_ylabel("Gain Margin (dB)")
    axes[1, 1].set_title("Gain Margin Constraint")
    axes[1, 1].legend(loc="best")
    axes[1, 1].grid(True, alpha=0.3)

    axes[1, 2].plot(histories["pm"], "orange", linewidth=2, label="PM (°)")
    axes[1, 2].axhline(
        y=45, color="r", linestyle="--", alpha=0.7, label="PM constraint"
    )
    axes[1, 2].set_xlabel("Iteration")
    axes[1, 2].set_ylabel("Phase Margin (°)")
    axes[1, 2].set_title("Phase Margin Constraint")
    axes[1, 2].legend(loc="best")
    axes[1, 2].grid(True, alpha=0.3)

    plt.suptitle("Optimization History", fontsize=14, fontweight="bold")
    plt.tight_layout()

    return fig


def save_all_plots(
    before_tfs, after_tfs, metrics_before, metrics_after, output_dir="plots"
):
    """
    Generate and save all analysis plots.

    Args:
        before_tfs: Dict with 'open_loop', 'closed_loop' TFs before optimization
        after_tfs: Dict with 'open_loop', 'closed_loop' TFs after optimization
        metrics_before: Dict of metrics before optimization
        metrics_after: Dict of metrics after optimization
        output_dir: Directory to save plots
    """
    import os

    os.makedirs(output_dir, exist_ok=True)

    # Before optimization plots
    fig = plot_bode(
        before_tfs["open_loop"],
        before_tfs["closed_loop"],
        (metrics_before["gm"], metrics_before["pm"], None, None),
        " - Before Optimization",
    )
    fig.savefig(f"{output_dir}/bode_before.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig = plot_nyquist(before_tfs["open_loop"], " - Before Optimization")
    fig.savefig(f"{output_dir}/nyquist_before.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig = plot_pole_zero(before_tfs["closed_loop"], " - Before Optimization")
    fig.savefig(f"{output_dir}/polezero_before.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig = plot_step_response(
        before_tfs["closed_loop"],
        metrics_before["settling_time"],
        metrics_before["overshoot"],
        title_suffix=" - Before Optimization",
    )
    fig.savefig(f"{output_dir}/step_before.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    # After optimization plots
    fig = plot_bode(
        after_tfs["open_loop"],
        after_tfs["closed_loop"],
        (metrics_after["gm"], metrics_after["pm"], None, None),
        " - After Optimization",
    )
    fig.savefig(f"{output_dir}/bode_after.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig = plot_nyquist(after_tfs["open_loop"], " - After Optimization")
    fig.savefig(f"{output_dir}/nyquist_after.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig = plot_pole_zero(after_tfs["closed_loop"], " - After Optimization")
    fig.savefig(f"{output_dir}/polezero_after.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    fig = plot_step_response(
        after_tfs["closed_loop"],
        metrics_after["settling_time"],
        metrics_after["overshoot"],
        title_suffix=" - After Optimization",
    )
    fig.savefig(f"{output_dir}/step_after.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    # Comparison plot
    fig = plot_comparison(
        before_tfs["closed_loop"],
        after_tfs["closed_loop"],
        metrics_before,
        metrics_after,
    )
    fig.savefig(f"{output_dir}/comparison.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"All plots saved to {output_dir}/")
