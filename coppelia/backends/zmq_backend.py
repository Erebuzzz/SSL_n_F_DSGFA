"""CoppeliaSim backend over the ZeroMQ remote API.

This backend drives a *running* CoppeliaSim instance through the modern ZeroMQ
remote API (package ``coppeliasim_zmqremoteapi_client``, bundled with CoppeliaSim
4.x under ``programming/zmqRemoteApi/clients/python``).

Because CoppeliaSim is a GUI simulator, this backend cannot be exercised in a
headless CI/offline environment -- it is validated by running against an actual
CoppeliaSim scene (see ``coppelia/scene/README.md``). The import of the client is
therefore guarded: importing this module never fails, and a clear, actionable
error is raised only when the backend is actually instantiated/connected without
the client available.

Robot / motor handles are resolved in one of two ways:

1. If ``config`` requests scene building, :mod:`coppelia.scene.build_scene` spawns
   the robots and returns their handles.
2. Otherwise the backend discovers existing objects in the open scene by alias,
   using ``robot_alias_template`` / motor templates (defaults target the
   CoppeliaSim Pioneer p3dx model).
"""

from __future__ import annotations

import numpy as np

from ..config import CoppeliaConfig, FloatArray
from ..unicycle import unicycle_to_wheel_speeds
from .base import RobotState, wrap_angle

try:  # pragma: no cover - availability depends on the host machine
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    _CLIENT_IMPORT_ERROR: Exception | None = None
except Exception as exc:  # pragma: no cover - offline path
    RemoteAPIClient = None  # type: ignore[assignment]
    _CLIENT_IMPORT_ERROR = exc


_INSTALL_HINT = (
    "The CoppeliaSim ZeroMQ remote API client is not available.\n"
    "Install it with:  python -m pip install coppeliasim-zmqremoteapi-client\n"
    "and make sure a CoppeliaSim 4.x instance is running with the ZMQ remote API "
    "add-on enabled (it is on by default; the default port is 23000)."
)


class ZmqBackend:
    """CoppeliaSim backend implementing the RobotBackend protocol."""

    name = "coppelia"

    #: Alias templates for discovering an already-populated scene. ``{i}`` is the
    #: zero-based robot index. These defaults match the Pioneer p3dx model when it
    #: is loaded n times and renamed ``/robot[i]`` (the scene builder does this).
    robot_alias_template = "/robot[{i}]"
    left_motor_alias_template = "/robot[{i}]/leftMotor"
    right_motor_alias_template = "/robot[{i}]/rightMotor"

    def __init__(self, config: CoppeliaConfig, build_scene: bool = True) -> None:
        self.config = config
        self.build_scene = build_scene
        self._client = None
        self._sim = None
        self._robot_handles: list[int] = []
        self._left_motors: list[int] = []
        self._right_motors: list[int] = []
        self._source_handle: int | None = None
        self._floor_handle: int | None = None
        self._field_contours_handle: int | None = None

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> None:
        if RemoteAPIClient is None:  # offline / not installed
            raise RuntimeError(_INSTALL_HINT) from _CLIENT_IMPORT_ERROR

        self._client = RemoteAPIClient(host=self.config.coppelia_host, port=self.config.coppelia_port)
        self._sim = self._client.require("sim")
        sim = self._sim

        if self.build_scene:
            from ..scene.build_scene import build_scene

            handles = build_scene(sim, self.config)
            self._robot_handles = handles["robots"]
            self._left_motors = handles["left_motors"]
            self._right_motors = handles["right_motors"]
            self._source_handle = handles.get("source")
            self._floor_handle = handles.get("floor")
            self._field_contours_handle = handles.get("field_contours")
        else:
            self._resolve_existing_handles(sim)

        sim.setStepping(bool(self.config.stepped))
        sim.startSimulation()

    def _resolve_existing_handles(self, sim) -> None:
        self._robot_handles = []
        self._left_motors = []
        self._right_motors = []
        for i in range(self.config.n):
            self._robot_handles.append(sim.getObject(self.robot_alias_template.format(i=i)))
            self._left_motors.append(sim.getObject(self.left_motor_alias_template.format(i=i)))
            self._right_motors.append(sim.getObject(self.right_motor_alias_template.format(i=i)))

    # ------------------------------------------------------------------
    # per-step interface
    # ------------------------------------------------------------------
    def get_poses(self) -> RobotState:
        sim = self._require_sim()
        positions = np.zeros((self.config.n, 2), dtype=float)
        headings = np.zeros(self.config.n, dtype=float)
        for i, handle in enumerate(self._robot_handles):
            pos = sim.getObjectPosition(handle, sim.handle_world)
            orient = sim.getObjectOrientation(handle, sim.handle_world)
            positions[i] = (pos[0], pos[1])
            headings[i] = orient[2]  # yaw (Z Euler angle)
        return RobotState(positions=positions, headings=wrap_angle(headings))

    def set_velocity_commands(self, v: FloatArray, omega: FloatArray) -> None:
        sim = self._require_sim()
        left, right = unicycle_to_wheel_speeds(
            np.asarray(v, dtype=float),
            np.asarray(omega, dtype=float),
            self.config.wheel_radius,
            self.config.wheel_base,
        )
        for i in range(self.config.n):
            sim.setJointTargetVelocity(self._left_motors[i], float(left[i]))
            sim.setJointTargetVelocity(self._right_motors[i], float(right[i]))

    def step(self, dt: float) -> None:  # noqa: ARG002 - dt fixed by scene dynamics dt
        sim = self._require_sim()
        if self.config.stepped:
            sim.step()
        # In non-stepped (real-time) mode CoppeliaSim advances on its own clock; the
        # control loop paces itself. Nothing to do here in that case.

    def close(self) -> None:
        if self._sim is not None:
            try:
                self._sim.stopSimulation()
            except Exception:  # pragma: no cover - best effort teardown
                pass
        self._sim = None
        self._client = None

    # ------------------------------------------------------------------
    def _require_sim(self):
        if self._sim is None:
            raise RuntimeError("ZmqBackend.connect() must be called before use.")
        return self._sim
