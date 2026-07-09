"""Synchronized 4-panel GIF/MP4 animation for the Python simulators.

Works for both the Phase 1 single-integrator (`SimulationResult`, states are the
robot points) and the Phase 2 unicycle (`UnicycleResult`, states are the control
points plus headings). The layout mirrors the CoppeliaSim Phase 3 animation so
every runtime produces the same visual:

    +-----------------------------+-----------------------------+
    | Formation + source          | Formation error vs time     |
    | robot trails + centroid path| Localization error vs time  |
    +-----------------------------+-----------------------------+

with a live time cursor on both error panels, the target formation circle, the
centroid, the source, and a gain-ratio / topology metadata banner.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .control import formation_slots


def _resolve(result):
    """Pull the common fields out of a SimulationResult or UnicycleResult."""

    cfg = result.config
    base = getattr(cfg, "base", cfg)  # unicycle wraps a base SimulationConfig
    points = getattr(result, "control_points", None)
    if points is None:
        points = result.positions
    headings = getattr(result, "headings", None)
    return base, points, headings


def save_animation(result, output_dir: Path, *, fmt: str = "gif", fps: int = 20,
                   max_frames: int = 240, title: str | None = None) -> Path | None:
    """Write the 4-panel animation for a Python run. Returns the written path."""

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    base, points, headings = _resolve(result)
    n = base.n
    source = base.source_array()
    phi = formation_slots(n)

    n_times = len(result.times)
    if n_times <= max_frames:
        frames = list(range(n_times))
    else:
        frames = list(np.linspace(0, n_times - 1, max_frames).astype(int))

    fig = plt.figure(figsize=(12, 6))
    ax_form = fig.add_subplot(1, 2, 1)
    ax_fe = fig.add_subplot(2, 2, 2)
    ax_le = fig.add_subplot(2, 2, 4)

    all_xy = points.reshape(-1, 2)
    pad = 1.0
    xmin, ymin = all_xy.min(axis=0) - pad
    xmax, ymax = all_xy.max(axis=0) + pad
    xmin = min(xmin, source[0] - pad); xmax = max(xmax, source[0] + pad)
    ymin = min(ymin, source[1] - pad); ymax = max(ymax, source[1] + pad)

    ax_form.scatter(source[0], source[1], marker="*", s=160, color="red", zorder=5, label="source")
    ax_form.set_xlim(xmin, xmax); ax_form.set_ylim(ymin, ymax)
    ax_form.set_aspect("equal"); ax_form.grid(True, alpha=0.3)
    ax_form.set_title("Formation and source"); ax_form.set_xlabel("x [m]"); ax_form.set_ylabel("y [m]")

    robot_dots = ax_form.scatter([], [], s=40, color="#1f77b4", zorder=4, label="robots")
    heading_quiver = None
    if headings is not None:
        heading_quiver = ax_form.quiver(np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n),
                                        color="#1f77b4", scale=20, width=0.004, zorder=4)
    centroid_dot = ax_form.scatter([], [], marker="x", s=70, color="black", zorder=6, label="centroid")
    (circle_line,) = ax_form.plot([], [], color="black", linestyle=":", linewidth=1.2, label="target circle")
    trails = [ax_form.plot([], [], linewidth=0.8, alpha=0.6)[0] for _ in range(n)]
    ax_form.legend(fontsize=7, loc="upper left")

    ax_fe.plot(result.times, result.formation_error, color="#1f77b4", linewidth=1.2)
    (fe_marker,) = ax_fe.plot([], [], "o", color="red", markersize=5)
    ax_fe.set_title("Formation error"); ax_fe.set_ylabel("formation error"); ax_fe.grid(True, alpha=0.3)

    ax_le.plot(result.times, result.localization_error, color="#1f77b4", linewidth=1.2)
    (le_marker,) = ax_le.plot([], [], "o", color="red", markersize=5)
    if getattr(result, "bound_applicable", False) and getattr(result, "epsilon", None) is not None:
        ax_le.axhline(result.epsilon, color="red", linestyle="--", linewidth=1.0, label="epsilon bound")
        ax_le.legend(fontsize=7)
    ax_le.set_title("Localization error"); ax_le.set_xlabel("time [s]")
    ax_le.set_ylabel("||centroid - source||"); ax_le.grid(True, alpha=0.3)

    if title is None:
        bl = getattr(base, "sign_boundary_layer", 0.0)
        title = (f"topology={base.topology}  alpha/beta={result.gain_ratio:g}  "
                 f"n={n}  dt={base.dt:g}  noise={base.noise_model}  eps_bl={bl:g}")
    fig.suptitle(title, fontsize=9)
    time_text = ax_form.text(0.02, 0.98, "", transform=ax_form.transAxes, va="top", fontsize=8)

    def update(frame: int):
        s = points[frame]
        centroid = result.centroid[frame]
        robot_dots.set_offsets(s)
        if heading_quiver is not None:
            heading_quiver.set_offsets(s)
            heading_quiver.set_UVC(np.cos(headings[frame]), np.sin(headings[frame]))
        centroid_dot.set_offsets(centroid[None, :])
        circle = centroid + base.radius * phi
        closed = np.vstack((circle, circle[0]))
        circle_line.set_data(closed[:, 0], closed[:, 1])
        upto = max(1, frame + 1)
        for i, trail in enumerate(trails):
            trail.set_data(points[:upto, i, 0], points[:upto, i, 1])
        fe_marker.set_data([result.times[frame]], [result.formation_error[frame]])
        le_marker.set_data([result.times[frame]], [result.localization_error[frame]])
        time_text.set_text(f"t = {result.times[frame]:.2f} s")
        return (robot_dots, centroid_dot, circle_line, fe_marker, le_marker, time_text, *trails)

    anim = FuncAnimation(fig, update, frames=frames, blit=False)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    path = None
    if fmt == "mp4":
        try:
            path = output_dir / "animation.mp4"
            anim.save(str(path), writer=FFMpegWriter(fps=fps))
        except Exception:
            path = None
    if path is None:
        path = output_dir / "animation.gif"
        anim.save(str(path), writer=PillowWriter(fps=fps))
    plt.close(fig)
    return path
