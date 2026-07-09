"""Abstract robot backend interface.

Both the mock kinematic simulator and the CoppeliaSim ZeroMQ backend implement
this small contract. The control loop calls, per step:

    poses = backend.get_poses()          # ground-truth (x, y, theta) per robot
    ...                                  # compute (v, omega) from the control law
    backend.set_velocity_commands(v, w)  # differential-drive body velocities
    backend.step(dt)                     # advance simulation by dt

``get_poses`` returns ground-truth poses, mirroring the paper's use of OptiTrack
as an external ground-truth source in Section V.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from ..config import FloatArray


@dataclass(frozen=True)
class RobotState:
    """Snapshot of all robot poses at one instant."""

    positions: FloatArray  # (n, 2) robot-center x, y
    headings: FloatArray  # (n,) yaw theta


class RobotBackend(Protocol):
    """Structural interface every backend must satisfy."""

    name: str

    def connect(self) -> None:
        """Prepare the backend (open connection, build/reset the scene)."""

    def get_poses(self) -> RobotState:
        """Return current ground-truth poses for all robots."""

    def set_velocity_commands(self, v: FloatArray, omega: FloatArray) -> None:
        """Command differential-drive body velocities (v [m/s], omega [rad/s])."""

    def step(self, dt: float) -> None:
        """Advance the simulation by one control period of ``dt`` seconds."""

    def close(self) -> None:
        """Tear down the backend (stop simulation, release resources)."""


def wrap_angle(theta: "np.ndarray | float") -> "np.ndarray | float":
    """Wrap angle(s) to (-pi, pi]."""

    return (np.asarray(theta) + np.pi) % (2.0 * np.pi) - np.pi
