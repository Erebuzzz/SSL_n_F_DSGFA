"""Persist per-step run telemetry for offline analysis.

The control loop (:mod:`coppelia.experiment`) records the full raw signal history
of a run -- poses, control points, the single-integrator command ``f_i``, the
commanded body velocities ``(v_i, omega_i)``, the (saturated, noisy) field samples
``sigma_i``, per-robot informed flags, and the aggregate error metrics. This module
writes that history to disk so a run can be re-analysed, re-plotted, or compared
against another backend/config without re-simulating.

Two artifacts are written, both at full time resolution (no down-sampling, so the
files are the ground truth for analysis):

- ``telemetry.npz`` -- every array under a named key, loadable with
  ``numpy.load(path)``. This is the primary analysis artifact.
- ``telemetry.csv`` -- a tidy, one-row-per-(step, robot) table for quick
  inspection in a spreadsheet or ``pandas.read_csv``. Global per-step quantities
  (centroid, error metrics, informed count) are repeated across the robot rows of
  each step.

The terminal step issues no command, so its ``f``/``v``/``omega`` telemetry is
``NaN`` (written as the literal ``nan`` in the CSV, which pandas reads as missing).
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from .metrics import CoppeliaResult

#: Ordered CSV columns (units suffixed where they aid interpretation).
CSV_COLUMNS = [
    "time_s",
    "robot",
    "x_m",
    "y_m",
    "theta_rad",
    "cp_x_m",
    "cp_y_m",
    "f_x",
    "f_y",
    "v_cmd_mps",
    "omega_cmd_radps",
    "sigma",
    "informed",
    "centroid_x_m",
    "centroid_y_m",
    "formation_error",
    "localization_error",
    "n_informed",
]


def save_telemetry(result: CoppeliaResult, output_dir: Path) -> dict[str, Path]:
    """Write ``telemetry.npz`` and ``telemetry.csv`` into ``output_dir``.

    Returns a mapping of artifact name -> written path.
    """

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    steps_plus_one, n, _ = result.control_points.shape
    nan_frame = np.full((steps_plus_one, n, 2), np.nan, dtype=float)
    nan_col = np.full((steps_plus_one, n), np.nan, dtype=float)

    commands = result.commands if result.commands is not None else nan_frame
    v = result.linear_velocity if result.linear_velocity is not None else nan_col
    omega = result.angular_velocity if result.angular_velocity is not None else nan_col
    sigma = result.measurements if result.measurements is not None else nan_col
    informed = (
        result.informed_mask
        if result.informed_mask is not None
        else np.zeros((steps_plus_one, n), dtype=int)
    )

    npz_path = output_dir / "telemetry.npz"
    np.savez_compressed(
        npz_path,
        times=result.times,
        positions=result.positions,
        control_points=result.control_points,
        headings=result.headings,
        commands=commands,
        linear_velocity=v,
        angular_velocity=omega,
        measurements=sigma,
        informed_mask=informed,
        centroid=result.centroid,
        formation_error=result.formation_error,
        localization_error=result.localization_error,
        n_informed=result.n_informed,
    )

    csv_path = output_dir / "telemetry.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(CSV_COLUMNS)
        for step in range(steps_plus_one):
            t = float(result.times[step])
            cx, cy = float(result.centroid[step, 0]), float(result.centroid[step, 1])
            fe = float(result.formation_error[step])
            le = float(result.localization_error[step])
            ni = int(result.n_informed[step])
            for i in range(n):
                writer.writerow(
                    [
                        t,
                        i,
                        float(result.positions[step, i, 0]),
                        float(result.positions[step, i, 1]),
                        float(result.headings[step, i]),
                        float(result.control_points[step, i, 0]),
                        float(result.control_points[step, i, 1]),
                        float(commands[step, i, 0]),
                        float(commands[step, i, 1]),
                        float(v[step, i]),
                        float(omega[step, i]),
                        float(sigma[step, i]),
                        int(informed[step, i]),
                        cx,
                        cy,
                        fe,
                        le,
                        ni,
                    ]
                )

    return {"npz": npz_path, "csv": csv_path}
