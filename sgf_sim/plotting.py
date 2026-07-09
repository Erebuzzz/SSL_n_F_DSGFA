"""Plotting helpers for run artifacts."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from .control import formation_slots
from .simulation import SimulationResult


def save_plots(result: SimulationResult, output_dir: Path) -> None:
    """Write trajectory and error plots for one simulation result."""

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


def _save_trajectory(result: SimulationResult, path: Path) -> None:
    config = result.config
    phi = formation_slots(config.n)
    final_centroid = result.centroid[-1]
    source = np.asarray(config.source, dtype=float)

    fig, ax = plt.subplots(figsize=(7, 6))
    stride = max(1, len(result.times) // 4000)
    for i in range(config.n):
        trail = result.positions[::stride, i, :]
        ax.plot(trail[:, 0], trail[:, 1], linewidth=1.2, label=f"robot {i}")
        ax.scatter(result.positions[0, i, 0], result.positions[0, i, 1], marker="o", s=22)
        ax.scatter(result.positions[-1, i, 0], result.positions[-1, i, 1], marker="s", s=28)

    circle = final_centroid + config.radius * phi
    closed = np.vstack((circle, circle[0]))
    ax.plot(closed[:, 0], closed[:, 1], color="black", linestyle=":", linewidth=1.2, label="final target circle")
    ax.scatter(source[0], source[1], marker="*", s=130, color="red", label="source")
    ax.scatter(final_centroid[0], final_centroid[1], marker="x", s=70, color="black", label="final centroid")
    ax.set_title("Robot Trajectories")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _save_series(
    times: np.ndarray,
    values: np.ndarray,
    title: str,
    xlabel: str,
    ylabel: str,
    path: Path,
    bound: float | None = None,
) -> None:
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


def _smooth_window(times: np.ndarray) -> int:
    if len(times) < 3:
        return 1
    dt = float(times[1] - times[0])
    if dt <= 0:
        return 1
    # A quarter-second display window keeps the sign-control chatter visible in
    # the raw line while matching the visual scale of the paper's plots better.
    window = max(1, int(round(0.25 / dt)))
    if window % 2 == 0:
        window += 1
    return min(window, len(times) if len(times) % 2 == 1 else len(times) - 1)


def _moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    pad = window // 2
    padded = np.pad(values, pad_width=pad, mode="edge")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(padded, kernel, mode="valid")
