"""Unicycle feedback linearization and differential-drive conversion (Section V).

The paper's control law (Eq. 4) is designed for single-integrator dynamics, but a
TurtleBot3 / Pioneer is a unicycle (non-holonomic). The standard point-offset
feedback-linearization trick controls a point ``s_i`` shifted a distance ``r`` ahead
of the wheel axle along the heading:

    s_i = p_i + r [cos(theta_i), sin(theta_i)]

Then ``s_i_dot = M(theta_i, r) [v_i, omega_i]^T`` with

    M = [[cos, -r sin],
         [sin,  r cos]]

so, given the desired single-integrator command ``f_i = s_i_dot`` from Eq. 4:

    [v_i, omega_i]^T = M^{-1} f_i,
    M^{-1} = [[ cos,       sin     ],
              [-sin / r,   cos / r ]]

The inversion is singular at ``r = 0`` -- ``r`` must be strictly positive.
"""

from __future__ import annotations

import numpy as np

from .config import FloatArray


def control_points(positions: FloatArray, headings: FloatArray, offset: float) -> FloatArray:
    """Return s_i = p_i + r [cos theta_i, sin theta_i] for all robots."""

    directions = np.column_stack((np.cos(headings), np.sin(headings)))
    return positions + offset * directions


def feedback_linearize(
    commands: FloatArray,
    headings: FloatArray,
    offset: float,
) -> tuple[FloatArray, FloatArray]:
    """Convert single-integrator commands f_i into unicycle (v_i, omega_i).

    Parameters
    ----------
    commands : (n, 2) desired control-point velocities f_i.
    headings : (n,) robot headings theta_i.
    offset : feedback-linearization shift r > 0.

    Returns
    -------
    (v, omega) : each shape (n,).
    """

    if offset <= 0:
        raise ValueError("control-point offset r must be positive (M is singular at r=0)")
    cos = np.cos(headings)
    sin = np.sin(headings)
    fx = commands[:, 0]
    fy = commands[:, 1]
    v = cos * fx + sin * fy
    omega = (-sin * fx + cos * fy) / offset
    return v, omega


def clip_commands(
    v: FloatArray,
    omega: FloatArray,
    max_v: float | None,
    max_omega: float | None,
) -> tuple[FloatArray, FloatArray]:
    """Saturate linear/angular velocity to the configured limits."""

    if max_v is not None:
        v = np.clip(v, -max_v, max_v)
    if max_omega is not None:
        omega = np.clip(omega, -max_omega, max_omega)
    return v, omega


def unicycle_to_wheel_speeds(
    v: FloatArray,
    omega: FloatArray,
    wheel_radius: float,
    wheel_base: float,
) -> tuple[FloatArray, FloatArray]:
    """Convert body (v, omega) into left/right wheel angular speeds [rad/s].

    v_r = (v + omega * L / 2) / R_wheel
    v_l = (v - omega * L / 2) / R_wheel
    """

    half_base = 0.5 * wheel_base
    right = (v + omega * half_base) / wheel_radius
    left = (v - omega * half_base) / wheel_radius
    return left, right


def wheel_speeds_to_unicycle(
    left: FloatArray,
    right: FloatArray,
    wheel_radius: float,
    wheel_base: float,
) -> tuple[FloatArray, FloatArray]:
    """Inverse of :func:`unicycle_to_wheel_speeds` (for backends reading wheels)."""

    v = 0.5 * wheel_radius * (right + left)
    omega = wheel_radius * (right - left) / wheel_base
    return v, omega
