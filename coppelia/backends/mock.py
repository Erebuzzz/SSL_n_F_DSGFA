"""Pure-Python differential-drive kinematic backend.

This backend requires no external software. It integrates the unicycle kinematics

    x_dot = v cos(theta),  y_dot = v sin(theta),  theta_dot = omega

with a fixed Euler step, optionally adding small actuator noise. It exists so the
full Phase 3 pipeline (control loop, metrics, plots, animation, ``summary.json``)
can be validated offline and covered by tests when CoppeliaSim is not installed.

It deliberately does **not** model wheel slip, inertia, or collisions -- that
realism is exactly what the CoppeliaSim backend adds. The mock is a kinematic
stand-in whose interface is identical, so switching to CoppeliaSim only changes
one config field (``backend``).
"""

from __future__ import annotations

import numpy as np

from ..config import CoppeliaConfig, FloatArray
from .base import RobotState, wrap_angle


class MockBackend:
    """Kinematic unicycle simulator implementing the RobotBackend protocol."""

    name = "mock"

    def __init__(self, config: CoppeliaConfig) -> None:
        self.config = config
        self._rng = np.random.default_rng(config.seed + 7919)  # distinct from measurement rng
        self._positions = config.resolved_initial_positions()
        self._headings = config.resolved_initial_headings()
        self._v = np.zeros(config.n, dtype=float)
        self._omega = np.zeros(config.n, dtype=float)
        # Optional actuator noise std (fraction of commanded speed). Zero keeps
        # the mock deterministic and the theorem checks clean.
        self.actuator_noise_std = 0.0

    def connect(self) -> None:  # noqa: D401 - part of the protocol
        """No-op: the mock backend is ready on construction."""

        self._positions = self.config.resolved_initial_positions()
        self._headings = self.config.resolved_initial_headings()

    def get_poses(self) -> RobotState:
        return RobotState(positions=self._positions.copy(), headings=self._headings.copy())

    def set_velocity_commands(self, v: FloatArray, omega: FloatArray) -> None:
        self._v = np.asarray(v, dtype=float).copy()
        self._omega = np.asarray(omega, dtype=float).copy()

    def step(self, dt: float) -> None:
        v = self._v
        omega = self._omega
        if self.actuator_noise_std > 0:
            v = v * (1.0 + self._rng.normal(0.0, self.actuator_noise_std, size=v.shape))
            omega = omega * (1.0 + self._rng.normal(0.0, self.actuator_noise_std, size=omega.shape))
        theta = self._headings
        self._positions = self._positions + dt * np.column_stack(
            (v * np.cos(theta), v * np.sin(theta))
        )
        self._headings = wrap_angle(theta + dt * omega)

    def close(self) -> None:  # noqa: D401 - part of the protocol
        """No-op teardown."""

        return None
