"""Backend-agnostic control loop for the Phase 3 build.

The same loop drives the mock kinematic backend and the CoppeliaSim backend. Each
control period it:

1. reads ground-truth robot poses (x, y, theta),
2. forms the feedback-linearization control points s_i,
3. measures the (saturated, noisy) source field at each s_i,
4. evaluates the Eq. 4 single-integrator command f_i,
5. inverts the unicycle map to (v_i, omega_i) and clips to limits,
6. sends commands and advances the backend by dt,
7. records poses and error metrics.

All measurement randomness uses a single seeded generator so a given
``(seed, backend)`` pair is reproducible.
"""

from __future__ import annotations

import numpy as np

from .backends import RobotBackend, make_backend
from .config import CoppeliaConfig
from .control import control_input, formation_slots, measurements
from .metrics import (
    CoppeliaResult,
    formation_error,
    localization_error,
    theory_metrics,
)
from .unicycle import (
    clip_commands,
    control_points,
    feedback_linearize,
)


def run_experiment(config: CoppeliaConfig, backend: RobotBackend | None = None) -> CoppeliaResult:
    """Run one Phase 3 experiment and return the recorded result."""

    config.validate()
    if backend is None:
        backend = make_backend(config)

    rng = np.random.default_rng(config.seed)
    adjacency = config.resolved_adjacency()
    phi = formation_slots(config.n)
    source = config.source_array()
    r = config.control_point_offset
    steps = int(round(config.duration / config.dt))

    times = np.linspace(0.0, steps * config.dt, steps + 1)
    positions = np.zeros((steps + 1, config.n, 2), dtype=float)
    cp_history = np.zeros((steps + 1, config.n, 2), dtype=float)
    heading_history = np.zeros((steps + 1, config.n), dtype=float)
    centroid = np.zeros((steps + 1, 2), dtype=float)
    formation_hist = np.zeros(steps + 1, dtype=float)
    localization_hist = np.zeros(steps + 1, dtype=float)
    n_informed = np.zeros(steps + 1, dtype=int)

    backend.connect()
    try:
        for step in range(steps + 1):
            state = backend.get_poses()
            s = control_points(state.positions, state.headings, r)

            positions[step] = state.positions
            heading_history[step] = state.headings
            cp_history[step] = s
            centroid[step] = np.mean(s, axis=0)
            formation_hist[step] = formation_error(s, config.radius, phi)
            localization_hist[step] = localization_error(s, source)

            sigma, informed = measurements(s, config, rng)
            n_informed[step] = int(np.sum(informed))

            if step == steps:
                break

            f = control_input(s, adjacency, phi, sigma, config)
            v, omega = feedback_linearize(f, state.headings, r)
            v, omega = clip_commands(
                v, omega, config.max_linear_velocity, config.max_angular_velocity
            )
            backend.set_velocity_commands(v, omega)
            backend.step(config.dt)
    finally:
        backend.close()

    threshold, epsilon, bound_applicable = theory_metrics(config, int(np.min(n_informed)))

    return CoppeliaResult(
        config=config,
        backend_name=getattr(backend, "name", config.backend),
        times=times,
        positions=positions,
        control_points=cp_history,
        headings=heading_history,
        centroid=centroid,
        formation_error=formation_hist,
        localization_error=localization_hist,
        n_informed=n_informed,
        gain_threshold=threshold,
        gain_ratio=config.alpha / config.beta,
        epsilon=epsilon,
        bound_applicable=bound_applicable,
    )
