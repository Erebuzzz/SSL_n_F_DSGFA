"""Static plots and the synchronized 4-panel animation for Phase 3 runs.

Artifacts written:

- ``trajectory.png``       -- robot trails, target circle, source, centroid path
- ``formation_error.png``  -- formation error vs time (raw + smoothed)
- ``localization_error.png`` -- localization error vs time with the epsilon bound
- ``animation.gif`` / ``.mp4`` -- 4 synchronized panels (roadmap Phase 3 requirement):
  robot formation + source, robot trails + centroid, live formation error, live
  localization error.

The animation degrades gracefully: MP4 is attempted only if requested and an
ffmpeg writer is available, otherwise it falls back to an animated GIF.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter

from .control import formation_slots
from .metrics import CoppeliaResult


# ----------------------------------------------------------------------
# static plots
# ----------------------------------------------------------------------
def save_plots(result: CoppeliaResult, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    _save_trajectory(result, output_dir / "trajectory.png")
    _save_series(
        result.times,
        result.formation_error,
        "Formation Error",
        "time [s]",
        "formation error",
        output_dir / "formation_error.png",
    )
    _save_series(
        result.times,
        result.localization_error,
        "Localization Error",
        "time [s]",
        "||centroid - source||",
        output_dir / "localization_error.png",
        bound=result.epsilon if result.bound_applicable else None,
    )
    _save_formation_error_per_robot(result, output_dir / "formation_error_per_robot.png")


def _save_formation_error_per_robot(result: CoppeliaResult, path: Path) -> None:
    """Per-robot formation error e_i = ||z_i - z*|| on the control points (paper Fig. 3).

    z_i = s_i - R * phi_i (shifted control-point state), z* = mean_j z_j. The aggregate
    ``formation_error`` is the L2 norm of these per-robot curves stacked.
    """

    config = result.config
    phi = formation_slots(config.n)  # (n, 2)
    z = result.control_points - config.radius * phi  # (steps+1, n, 2)
    z_star = z.mean(axis=1, keepdims=True)  # (steps+1, 1, 2)
    per_robot = np.linalg.norm(z - z_star, axis=2)  # (steps+1, n)

    fig, ax = plt.subplots(figsize=(7, 4))
    stride = max(1, len(result.times) // 5000)
    for i in range(config.n):
        ax.plot(result.times[::stride], per_robot[::stride, i], linewidth=1.0, label=f"robot {i}")
    ax.set_title("Per-Robot Formation Error (paper Fig. 3)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"$\|z_i - z^*\|$")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2, loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _save_trajectory(result: CoppeliaResult, path: Path) -> None:
    config = result.config
    phi = formation_slots(config.n)
    final_centroid = result.centroid[-1]
    source = np.asarray(config.source, dtype=float)

    fig, ax = plt.subplots(figsize=(7, 6))
    stride = max(1, len(result.times) // 4000)
    for i in range(config.n):
        trail = result.control_points[::stride, i, :]
        ax.plot(trail[:, 0], trail[:, 1], linewidth=1.2, label=f"robot {i}")
        ax.scatter(result.control_points[0, i, 0], result.control_points[0, i, 1], marker="o", s=22)
        ax.scatter(result.control_points[-1, i, 0], result.control_points[-1, i, 1], marker="s", s=28)

    circle = final_centroid + config.radius * phi
    closed = np.vstack((circle, circle[0]))
    ax.plot(closed[:, 0], closed[:, 1], color="black", linestyle=":", linewidth=1.2, label="final target circle")
    ax.plot(result.centroid[::stride, 0], result.centroid[::stride, 1], color="black", linewidth=1.0, alpha=0.6, label="centroid path")
    ax.scatter(source[0], source[1], marker="*", s=130, color="red", label="source")
    ax.scatter(final_centroid[0], final_centroid[1], marker="x", s=70, color="black", label="final centroid")
    ax.set_title(f"Robot Trajectories ({result.backend_name})")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _save_series(times, values, title, xlabel, ylabel, path, bound=None) -> None:
    fig, ax = plt.subplots(figsize=(7, 4))
    stride = max(1, len(times) // 5000)
    smooth = _moving_average(values, _smooth_window(times))

    ax.plot(times[::stride], values[::stride], color="#8a8f98", alpha=0.32, linewidth=0.7, label="raw")
    ax.plot(times[::stride], smooth[::stride], color="#1f77b4", linewidth=1.7, label="smoothed")
    if bound is not None:
        ax.axhline(bound, color="red", linestyle="--", linewidth=1.0, label="theoretical bound")
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _smooth_window(times) -> int:
    if len(times) < 3:
        return 1
    dt = float(times[1] - times[0])
    if dt <= 0:
        return 1
    window = max(1, int(round(0.25 / dt)))
    if window % 2 == 0:
        window += 1
    return min(window, len(times) if len(times) % 2 == 1 else len(times) - 1)


def _moving_average(values, window: int):
    if window <= 1:
        return np.asarray(values).copy()
    pad = window // 2
    padded = np.pad(values, pad_width=pad, mode="edge")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(padded, kernel, mode="valid")


# ----------------------------------------------------------------------
# animation
# ----------------------------------------------------------------------
def save_animation(result: CoppeliaResult, output_dir: Path) -> Path | None:
    """Write the synchronized 4-panel animation. Returns the written path."""

    output_dir.mkdir(parents=True, exist_ok=True)
    config = result.config
    fmt = config.animation_format
    frame_idx = _frame_indices(len(result.times), config.animation_max_frames)
    phi = formation_slots(config.n)
    source = np.asarray(config.source, dtype=float)

    fig = plt.figure(figsize=(12, 6))
    ax_form = fig.add_subplot(1, 2, 1)
    ax_fe = fig.add_subplot(2, 2, 2)
    ax_le = fig.add_subplot(2, 2, 4)

    # spatial extents with margin
    all_xy = result.control_points.reshape(-1, 2)
    pad = 1.0
    xmin, ymin = all_xy.min(axis=0) - pad
    xmax, ymax = all_xy.max(axis=0) + pad
    xmin = min(xmin, source[0] - pad)
    xmax = max(xmax, source[0] + pad)
    ymin = min(ymin, source[1] - pad)
    ymax = max(ymax, source[1] + pad)

    # static elements
    ax_form.scatter(source[0], source[1], marker="*", s=160, color="red", zorder=5, label="source")
    ax_form.set_xlim(xmin, xmax)
    ax_form.set_ylim(ymin, ymax)
    ax_form.set_aspect("equal")
    ax_form.grid(True, alpha=0.3)
    ax_form.set_title("Formation and Source")
    ax_form.set_xlabel("x [m]")
    ax_form.set_ylabel("y [m]")

    robot_dots = ax_form.scatter([], [], s=40, color="#1f77b4", zorder=4, label="robots")
    heading_quiver = ax_form.quiver(
        np.zeros(config.n), np.zeros(config.n), np.zeros(config.n), np.zeros(config.n),
        color="#1f77b4", scale=20, width=0.004, zorder=4,
    )
    centroid_dot = ax_form.scatter([], [], marker="x", s=70, color="black", zorder=6, label="centroid")
    (circle_line,) = ax_form.plot([], [], color="black", linestyle=":", linewidth=1.2, label="target circle")
    trails = [ax_form.plot([], [], linewidth=0.8, alpha=0.6)[0] for _ in range(config.n)]
    ax_form.legend(fontsize=7, loc="upper left")

    # error panels
    ax_fe.plot(result.times, result.formation_error, color="#1f77b4", linewidth=1.2)
    (fe_marker,) = ax_fe.plot([], [], "o", color="red", markersize=5)
    ax_fe.set_title("Formation Error")
    ax_fe.set_ylabel("formation error")
    ax_fe.grid(True, alpha=0.3)

    ax_le.plot(result.times, result.localization_error, color="#1f77b4", linewidth=1.2)
    (le_marker,) = ax_le.plot([], [], "o", color="red", markersize=5)
    if result.bound_applicable and result.epsilon is not None:
        ax_le.axhline(result.epsilon, color="red", linestyle="--", linewidth=1.0, label="epsilon bound")
        ax_le.legend(fontsize=7)
    ax_le.set_title("Localization Error")
    ax_le.set_xlabel("time [s]")
    ax_le.set_ylabel("||p* - p_s||")
    ax_le.grid(True, alpha=0.3)

    meta = (
        f"backend={result.backend_name}  topology={config.topology}  "
        f"alpha/beta={result.gain_ratio:g}  n={config.n}"
    )
    fig.suptitle(meta, fontsize=9)
    time_text = ax_form.text(0.02, 0.98, "", transform=ax_form.transAxes, va="top", fontsize=8)

    def update(frame: int):
        s = result.control_points[frame]
        centroid = result.centroid[frame]
        robot_dots.set_offsets(s)
        directions = np.column_stack((np.cos(result.headings[frame]), np.sin(result.headings[frame])))
        heading_quiver.set_offsets(s)
        heading_quiver.set_UVC(directions[:, 0], directions[:, 1])
        centroid_dot.set_offsets(centroid[None, :])
        circle = centroid + config.radius * phi
        closed = np.vstack((circle, circle[0]))
        circle_line.set_data(closed[:, 0], closed[:, 1])
        upto = max(1, frame + 1)
        for i, trail in enumerate(trails):
            trail.set_data(result.control_points[:upto, i, 0], result.control_points[:upto, i, 1])
        fe_marker.set_data([result.times[frame]], [result.formation_error[frame]])
        le_marker.set_data([result.times[frame]], [result.localization_error[frame]])
        time_text.set_text(f"t = {result.times[frame]:.2f} s")
        return (robot_dots, centroid_dot, circle_line, fe_marker, le_marker, time_text, *trails)

    anim = FuncAnimation(fig, update, frames=frame_idx, blit=False)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    path = _write_animation(anim, fig, output_dir, fmt, config.animation_fps)
    plt.close(fig)
    return path


def _write_animation(anim, fig, output_dir: Path, fmt: str, fps: int) -> Path | None:
    if fmt == "mp4":
        try:
            path = output_dir / "animation.mp4"
            anim.save(str(path), writer=FFMpegWriter(fps=fps))
            return path
        except Exception:
            # ffmpeg not present -> fall back to GIF rather than failing the run
            pass
    path = output_dir / "animation.gif"
    anim.save(str(path), writer=PillowWriter(fps=fps))
    return path


def _frame_indices(n_times: int, max_frames: int):
    if n_times <= max_frames:
        return list(range(n_times))
    return list(np.linspace(0, n_times - 1, max_frames).astype(int))
