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

import warnings

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


class PartialRunError(RuntimeError):
    """Raised when a run is interrupted or crashes mid-loop after recording data.

    Carries the :class:`CoppeliaResult` built from the steps that *were* recorded
    before the failure, so the caller can still persist partial telemetry/plots for
    analysis. The original cause is chained via ``raise ... from`` and is also kept
    on :attr:`__cause__`. Only raised once at least two time steps were recorded;
    earlier failures (e.g. the backend never connected) propagate unchanged.
    """

    def __init__(self, result: CoppeliaResult, recorded: int) -> None:
        self.result = result
        self.recorded = recorded
        super().__init__(
            f"run interrupted after {recorded} recorded step(s); "
            "partial result is available on the .result attribute"
        )


def run_experiment(config: CoppeliaConfig, backend: RobotBackend | None = None) -> CoppeliaResult:
    """Run one Phase 3 experiment and return the recorded result.

    If the control loop is interrupted (``KeyboardInterrupt``) or raises after at
    least two steps were recorded, a :class:`PartialRunError` carrying the partial
    :class:`CoppeliaResult` is raised instead of returning, so callers can still
    save what was collected.
    """

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

    # --- telemetry: raw control/measurement signals recorded every step ---
    # Commands (f_i, v_i, omega_i) exist for steps 0..steps-1; the terminal step
    # only reads poses, so its command row stays NaN. Measurements/informed flags
    # are sampled at every step (including the terminal one).
    command_hist = np.full((steps + 1, config.n, 2), np.nan, dtype=float)
    v_hist = np.full((steps + 1, config.n), np.nan, dtype=float)
    omega_hist = np.full((steps + 1, config.n), np.nan, dtype=float)
    sigma_hist = np.zeros((steps + 1, config.n), dtype=float)
    informed_hist = np.zeros((steps + 1, config.n), dtype=int)

    arrays = dict(
        times=times,
        positions=positions,
        control_points=cp_history,
        headings=heading_history,
        centroid=centroid,
        formation_error=formation_hist,
        localization_error=localization_hist,
        n_informed=n_informed,
        commands=command_hist,
        linear_velocity=v_hist,
        angular_velocity=omega_hist,
        measurements=sigma_hist,
        informed_mask=informed_hist,
    )
    backend_name = getattr(backend, "name", config.backend)

    # ``recorded`` counts the time steps whose pose/measurement rows are fully
    # populated. It advances only after a step's data has been written, so a crash
    # mid-step leaves it pointing at the last complete row.
    recorded = 0
    error: BaseException | None = None

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
            sigma_hist[step] = sigma
            informed_hist[step] = informed.astype(int)
            recorded = step + 1

            if step == steps:
                break

            f = control_input(s, adjacency, phi, sigma, config)
            v, omega = feedback_linearize(f, state.headings, r)
            v, omega = clip_commands(
                v, omega, config.max_linear_velocity, config.max_angular_velocity
            )
            command_hist[step] = f
            v_hist[step] = v
            omega_hist[step] = omega
            backend.set_velocity_commands(v, omega)
            backend.step(config.dt)
    except (KeyboardInterrupt, Exception) as exc:  # noqa: BLE001 - re-raised below
        error = exc
    finally:
        backend.close()

    if error is not None and recorded < 2:
        # Too little collected to be worth analysing (e.g. the very first pose read
        # failed): surface the original error unchanged rather than an empty run.
        raise error

    result = _build_result(config, backend_name, arrays, recorded)

    if error is not None:
        raise PartialRunError(result, recorded) from error

    return result


def _build_result(
    config: CoppeliaConfig, backend_name: str, arrays: dict, recorded: int
) -> CoppeliaResult:
    """Assemble a :class:`CoppeliaResult`, slicing every array to ``recorded`` steps.

    On a fully-completed run ``recorded == steps + 1`` so the slices are no-ops; on
    an interrupted run this drops the untouched trailing rows so plots/telemetry and
    the theory metrics reflect only the data that was actually recorded.
    """

    sliced = {key: value[:recorded] for key, value in arrays.items()}

    min_informed = int(np.min(sliced["n_informed"])) if recorded else 0
    if min_informed == 0:
        warnings.warn(
            "No robot is ever within Dmax of the source (0 informed robots): there "
            "is no source signal, so the centroid cannot localize. Move the source "
            "closer, raise Dmax, or start the robots near the source.",
            RuntimeWarning,
            stacklevel=2,
        )
    threshold, epsilon, bound_applicable = theory_metrics(config, min_informed)

    return CoppeliaResult(
        config=config,
        backend_name=backend_name,
        times=sliced["times"],
        positions=sliced["positions"],
        control_points=sliced["control_points"],
        headings=sliced["headings"],
        centroid=sliced["centroid"],
        formation_error=sliced["formation_error"],
        localization_error=sliced["localization_error"],
        n_informed=sliced["n_informed"],
        gain_threshold=threshold,
        gain_ratio=config.alpha / config.beta,
        epsilon=epsilon,
        bound_applicable=bound_applicable,
        commands=sliced["commands"],
        linear_velocity=sliced["linear_velocity"],
        angular_velocity=sliced["angular_velocity"],
        measurements=sliced["measurements"],
        informed_mask=sliced["informed_mask"],
    )
