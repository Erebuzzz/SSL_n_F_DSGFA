"""Robot backends for the Phase 3 build.

A backend hides where the robots physically live. The control loop in
:mod:`coppelia.experiment` only ever sees the :class:`~coppelia.backends.base.RobotBackend`
interface, so the exact same algorithm drives either the offline mock kinematic
simulator or a running CoppeliaSim scene.
"""

from __future__ import annotations

from .base import RobotBackend, RobotState

__all__ = ["RobotBackend", "RobotState", "make_backend"]


def make_backend(config):
    """Factory: build the backend named by ``config.backend``."""

    if config.backend == "mock":
        from .mock import MockBackend

        return MockBackend(config)
    if config.backend == "coppelia":
        from .zmq_backend import ZmqBackend

        return ZmqBackend(config)
    raise ValueError(f"unknown backend: {config.backend}")
