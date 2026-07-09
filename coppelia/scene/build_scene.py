"""Programmatically construct the Phase 3 scene inside a running CoppeliaSim.

Rather than shipping a binary ``.ttt`` scene file (which cannot be authored or
diffed as text), the scene is built at runtime through the ZeroMQ remote API. This
keeps the whole Phase 3 build text-based and reviewable, and lets the topology /
robot count follow the config.

``build_scene(sim, config)`` loads ``config.n`` differential-drive robot models
(default: the CoppeliaSim Pioneer p3dx), lays them out at the configured initial
positions, drops a non-colliding marker sphere at the source location, and returns
the handles the backend needs::

    {
        "robots": [baseHandle, ...],
        "left_motors": [handle, ...],
        "right_motors": [handle, ...],
        "source": sourceHandle,
    }

This function is only reachable when a CoppeliaSim instance is connected, so it is
exercised against the real simulator, not in offline CI. It is written defensively
(subtree search for the wheel motors, graceful model-path resolution) precisely
because it cannot be unit-tested here.
"""

from __future__ import annotations

from ..config import CoppeliaConfig

# Relative model paths within the CoppeliaSim install ``models`` directory.
ROBOT_MODEL_PATHS = {
    "pioneer": "robots/mobile/pioneer p3dx.ttm",
    "dr12": "robots/mobile/dr12 (Dr Robot).ttm",
}


def build_scene(sim, config: CoppeliaConfig) -> dict:
    """Build the scene and return robot / motor / source handles."""

    positions = config.resolved_initial_positions()
    headings = config.resolved_initial_headings()
    model_path = _resolve_model_path(sim, config)

    robots: list[int] = []
    left_motors: list[int] = []
    right_motors: list[int] = []

    for i in range(config.n):
        base = sim.loadModel(model_path)
        sim.setObjectAlias(base, f"robot[{i}]")
        z = _base_height(sim, base)
        sim.setObjectPosition(base, sim.handle_world, [float(positions[i, 0]), float(positions[i, 1]), z])
        sim.setObjectOrientation(base, sim.handle_world, [0.0, 0.0, float(headings[i])])
        left, right = _find_wheel_motors(sim, base)
        left_motors.append(left)
        right_motors.append(right)
        robots.append(base)

    source = _create_source_marker(sim, config)

    return {
        "robots": robots,
        "left_motors": left_motors,
        "right_motors": right_motors,
        "source": source,
    }


def _resolve_model_path(sim, config: CoppeliaConfig) -> str:
    """Return an absolute model path for the configured robot model."""

    relative = ROBOT_MODEL_PATHS.get(config.robot_model, ROBOT_MODEL_PATHS["pioneer"])
    # sim.stringparam_application_path points at the CoppeliaSim install root.
    app_path = sim.getStringParam(sim.stringparam_application_path)
    separator = "\\" if "\\" in app_path else "/"
    return separator.join([app_path.rstrip("/\\"), "models", relative.replace("/", separator)])


def _base_height(sim, base: int) -> float:
    """Keep the loaded model's own vertical placement."""

    try:
        pos = sim.getObjectPosition(base, sim.handle_world)
        return float(pos[2])
    except Exception:  # pragma: no cover - depends on live sim
        return 0.1387  # Pioneer p3dx default base height [m]


def _find_wheel_motors(sim, base: int) -> tuple[int, int]:
    """Locate the left/right drive joints within a robot model subtree.

    Matches by alias substring so it works regardless of the exact model naming
    (``leftMotor``/``rightMotor`` on the Pioneer, ``leftWheel`` etc. elsewhere).
    """

    joints = sim.getObjectsInTree(base, sim.object_joint_type, 0)
    left = right = None
    for handle in joints:
        alias = sim.getObjectAlias(handle, 5).lower()  # 5 -> full path form
        if "left" in alias and left is None:
            left = handle
        elif "right" in alias and right is None:
            right = handle
    if left is None or right is None:
        raise RuntimeError(
            "Could not identify left/right drive motors in the loaded robot model. "
            "Set ZmqBackend.*_motor_alias_template or adjust the model."
        )
    return left, right


def _create_source_marker(sim, config: CoppeliaConfig):
    """Add a small red, non-respondable sphere marking the source location."""

    source = config.source_array()
    diameter = 0.25
    options = 0  # not respondable, not dynamic
    handle = sim.createPrimitiveShape(sim.primitiveshape_spheroid, [diameter, diameter, diameter], options)
    sim.setObjectAlias(handle, "source_marker")
    sim.setObjectPosition(handle, sim.handle_world, [float(source[0]), float(source[1]), diameter / 2.0])
    try:
        sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, [1.0, 0.1, 0.1])
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    except Exception:  # pragma: no cover - cosmetic only
        pass
    return handle
