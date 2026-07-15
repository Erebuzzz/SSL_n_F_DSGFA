"""Programmatically construct the Phase 3 scene inside a running CoppeliaSim.

Rather than shipping a binary ``.ttt`` scene file (which cannot be authored or
diffed as text), the scene is built at runtime through the ZeroMQ remote API. This
keeps the whole Phase 3 build text-based and reviewable, and lets the topology /
robot count follow the config.

``build_scene(sim, config)`` builds a large static floor sized to the swarm,
loads ``config.n`` differential-drive robot models (default: the CoppeliaSim
Pioneer p3dx) resting on it, drops a non-colliding marker sphere at the source
location, draws the scalar-field contour rings, and returns the handles the
backend needs::

    {
        "robots": [baseHandle, ...],
        "left_motors": [handle, ...],
        "right_motors": [handle, ...],
        "source": sourceHandle,
        "floor": floorHandle,
        "field_contours": drawingHandle,
    }

This function is only reachable when a CoppeliaSim instance is connected, so it is
exercised against the real simulator, not in offline CI. It is written defensively
(subtree search for the wheel motors, graceful model-path resolution) precisely
because it cannot be unit-tested here.
"""

from __future__ import annotations

import numpy as np

from ..config import CoppeliaConfig

# Relative model paths within the CoppeliaSim install ``models`` directory.
ROBOT_MODEL_PATHS = {
    "pioneer": "robots/mobile/pioneer p3dx.ttm",
    "dr12": "robots/mobile/dr12 (Dr Robot).ttm",
}


def build_scene(sim, config: CoppeliaConfig) -> dict:
    """Build the scene and return robot / motor / source handles.

    Build order matters for the physics: the floor is established **first** (sized
    to cover the whole swarm, top surface at ``z = 0``) so that when the robots are
    loaded and the simulation starts they rest on solid ground at their natural
    height -- instead of spawning over the void (and falling) or inside a thick
    up-scaled floor (and being ejected/toppling on the first physics step).
    """

    positions = config.resolved_initial_positions()
    headings = config.resolved_initial_headings()
    model_path = _resolve_model_path(sim, config)

    # 1. Floor first, so robots have ground to rest on the moment they are placed.
    floor = _build_floor(sim, config)

    # 2. Robots at their (x, y) and the model's natural base height above z = 0.
    robots: list[int] = []
    left_motors: list[int] = []
    right_motors: list[int] = []

    for i in range(config.n):
        base = sim.loadModel(model_path)
        sim.setObjectAlias(base, f"robot[{i}]")
        z = _base_height(sim, base) + _SPAWN_CLEARANCE
        sim.setObjectPosition(base, sim.handle_world, [float(positions[i, 0]), float(positions[i, 1]), z])
        sim.setObjectOrientation(base, sim.handle_world, [0.0, 0.0, float(headings[i])])
        left, right = _find_wheel_motors(sim, base)
        left_motors.append(left)
        right_motors.append(right)
        robots.append(base)

    source = _create_source_marker(sim, config)
    field_contours = _draw_field_contours(sim, config)

    return {
        "robots": robots,
        "left_motors": left_motors,
        "right_motors": right_motors,
        "source": source,
        "floor": floor,
        "field_contours": field_contours,
    }


def _resolve_model_path(sim, config: CoppeliaConfig) -> str:
    """Return an absolute model path for the configured robot model."""

    relative = ROBOT_MODEL_PATHS.get(config.robot_model, ROBOT_MODEL_PATHS["pioneer"])
    # sim.stringparam_application_path points at the CoppeliaSim install root.
    app_path = sim.getStringParam(sim.stringparam_application_path)
    separator = "\\" if "\\" in app_path else "/"
    return separator.join([app_path.rstrip("/\\"), "models", relative.replace("/", separator)])


# Small vertical gap [m] the robots are lifted above their natural resting height
# so they settle gently onto the floor instead of interpenetrating it on the first
# physics step (interpenetration is what previously ejected/toppled them).
_SPAWN_CLEARANCE = 0.005


def _base_height(sim, base: int) -> float:
    """The loaded model's natural base height above a ``z = 0`` floor top.

    The robot models are authored to rest on a floor whose top surface is at
    ``z = 0``; :func:`_build_floor` guarantees exactly that, so keeping the model's
    own loaded z places the wheels on the ground.
    """

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


# Aliases the default new-scene floor object has gone by across CoppeliaSim
# releases. The tiny (5 m) default floor is removed so it cannot be tripped over.
_FLOOR_ALIASES = ("/Floor", "/ResizableFloor_5_25", "/ResizableFloor_5_25/element")

#: Floor thickness [m]. Only the top surface (kept at z = 0) matters to the robots.
_FLOOR_THICKNESS = 0.2
#: Fixed clearance [m] added to every side of the swarm/source bounding box, on top
#: of the ``floor_scale`` safety factor, so the localization drift never runs off.
_FLOOR_MARGIN = 2.0


def _floor_extents(config: CoppeliaConfig) -> tuple[float, float, float, float]:
    """Return ``(center_x, center_y, size_x, size_y)`` for the scene floor [m].

    The floor must cover every robot's *entire trajectory*, not just its start, so
    it is sized to the swarm+source bounding box, expanded by the ``floor_scale``
    safety factor (default 7x the half-extents) plus a fixed :data:`_FLOOR_MARGIN`.
    Pure/geometric so it is unit-testable without a live simulator.
    """

    positions = config.resolved_initial_positions()
    points = np.vstack([positions, config.source_array()[None, :]])
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) / 2.0
    half_span = (maxs - mins) / 2.0
    half = half_span * float(config.floor_scale) + _FLOOR_MARGIN
    return float(center[0]), float(center[1]), float(2.0 * half[0]), float(2.0 * half[1])


def _build_floor(sim, config: CoppeliaConfig):
    """Create a large static floor under the swarm, top surface at ``z = 0``.

    Replaces the previous ``scaleObject`` approach, which scaled the shipped floor's
    *thickness* too and lifted its top surface above the robots (ejecting them). A
    fresh cuboid gives a reliably-sized, correctly-positioned floor across
    CoppeliaSim versions. The tiny default floor is removed best-effort. Cosmetic
    failures never abort the run: on error the function returns ``None``.
    """

    _remove_default_floor(sim)
    cx, cy, sx, sy = _floor_extents(config)
    try:
        floor = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [sx, sy, _FLOOR_THICKNESS], 0
        )
        sim.setObjectAlias(floor, "sgf_floor")
        # Centre the slab so its TOP face sits exactly at z = 0.
        sim.setObjectPosition(floor, sim.handle_world, [cx, cy, -_FLOOR_THICKNESS / 2.0])
        # Static + respondable => robots rest on it but it never moves or falls.
        sim.setObjectInt32Param(floor, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(floor, sim.shapeintparam_respondable, 1)
        try:
            sim.setShapeColor(floor, None, sim.colorcomponent_ambient_diffuse, [0.55, 0.55, 0.6])
        except Exception:  # pragma: no cover - cosmetic only
            pass
        return floor
    except Exception:  # pragma: no cover - depends on live sim / API version
        return None


def _remove_default_floor(sim) -> None:
    """Best-effort removal of the shipped 5 m default floor, if present."""

    floor = _find_floor(sim)
    if floor is None:
        return
    for remover in ("removeObjects", "removeObject", "removeModel"):
        try:
            method = getattr(sim, remover)
            method([floor] if remover == "removeObjects" else floor)
            return
        except Exception:  # pragma: no cover - API/version differences
            continue


def _find_floor(sim):
    """Return the default floor object handle, or ``None`` if absent."""

    for alias in _FLOOR_ALIASES:
        try:
            return sim.getObject(alias)
        except Exception:  # pragma: no cover - alias absent in this scene
            continue
    return None


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


def _contour_ring_radii(config: CoppeliaConfig) -> list[float]:
    """Radii (m) of the field contour rings to draw around the source.

    The scalar field f = kappa*||z - p_s||^2 has circular level sets, so a contour
    at field level c is a circle of radius sqrt(c/kappa) about the source. We pick
    six evenly-spaced radii bracketing the initial swarm spread (max robot->source
    distance) so the rings frame where the robots start and where they converge.
    Pure/geometric, so it is unit-testable without a live simulator.
    """

    source = config.source_array()
    positions = config.resolved_initial_positions()
    max_dist = float(np.max(np.linalg.norm(positions - source, axis=1)))
    if max_dist <= 1e-6:
        return [float(config.radius)]
    return [float(r) for r in np.linspace(max_dist / 6.0, max_dist, 6)]


def _draw_field_contours(sim, config: CoppeliaConfig, segments: int = 72):
    """Draw concentric field contour rings on the floor around the source.

    Uses the CoppeliaSim drawing API (``sim.drawing_lines``): each ring is a closed
    polyline of ``segments`` short line items at a small height above the floor
    (avoids z-fighting). Best-effort like :func:`_scale_floor` / the source marker:
    any failure (older API, unsupported constant) returns ``None`` and leaves the
    run untouched. Not covered by the offline test-suite (needs a live simulator);
    the pure radius helper :func:`_contour_ring_radii` is tested instead.
    """

    try:
        source = config.source_array()
        radii = _contour_ring_radii(config)
        # (type, size/width, duplicateTolerance, parentHandle, maxItemCount, color)
        handle = sim.addDrawingObject(sim.drawing_lines, 2, 0.0, -1, 100000, [0.1, 0.8, 1.0])
        height = 0.02
        angles = np.linspace(0.0, 2.0 * np.pi, segments + 1)
        for radius in radii:
            xs = source[0] + radius * np.cos(angles)
            ys = source[1] + radius * np.sin(angles)
            for k in range(segments):
                sim.addDrawingObjectItem(
                    handle,
                    [float(xs[k]), float(ys[k]), height, float(xs[k + 1]), float(ys[k + 1]), height],
                )
        return handle
    except Exception:  # pragma: no cover - depends on live sim / API version
        return None
