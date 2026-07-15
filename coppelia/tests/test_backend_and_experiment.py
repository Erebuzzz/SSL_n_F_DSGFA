import csv

import numpy as np

from coppelia.backends import make_backend
from coppelia.backends.mock import MockBackend
from coppelia.config import CoppeliaConfig
from coppelia.control import signal_field
from coppelia.experiment import PartialRunError, run_experiment
from coppelia.plotting import save_plots
from coppelia.scene.build_scene import _contour_ring_radii, _floor_extents
from coppelia.telemetry import CSV_COLUMNS, save_telemetry
from coppelia.topology import TOPOLOGY_NAMES, adjacency_for_topology, is_connected


def test_make_backend_returns_mock():
    config = CoppeliaConfig(backend="mock")
    backend = make_backend(config)
    assert isinstance(backend, MockBackend)
    assert backend.name == "mock"


def test_mock_backend_moves_robots_under_commands():
    config = CoppeliaConfig(
        n=3,
        initial_positions=np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]),
        initial_headings=np.zeros(3),
    )
    backend = MockBackend(config)
    backend.connect()
    start = backend.get_poses().positions.copy()
    backend.set_velocity_commands(np.ones(3), np.zeros(3))
    backend.step(0.1)
    moved = backend.get_poses().positions
    # heading 0 => pure +x motion of 0.1 m
    np.testing.assert_allclose(moved[:, 0] - start[:, 0], np.full(3, 0.1), atol=1e-12)
    np.testing.assert_allclose(moved[:, 1] - start[:, 1], np.zeros(3), atol=1e-12)


def test_named_topologies_connected_and_undirected():
    for name in TOPOLOGY_NAMES:
        adjacency = adjacency_for_topology(name, 6)
        assert adjacency.shape == (6, 6)
        assert np.array_equal(adjacency, adjacency.T)
        assert is_connected(adjacency)


def test_experiment_runs_and_reduces_errors_no_noise():
    config = CoppeliaConfig(
        duration=40.0,
        dt=0.004,
        noise_model="none",
        backend="mock",
        save_plots=False,
        save_animation=False,
    )
    result = run_experiment(config)

    steps = int(round(config.duration / config.dt)) + 1
    assert result.positions.shape == (steps, 6, 2)
    assert result.control_points.shape == (steps, 6, 2)
    # Formation should converge tightly; localization should move strongly toward
    # the source (the unicycle build localizes gradually, see README).
    assert result.formation_error[-1] < 0.3
    assert result.localization_error[-1] < 0.5 * result.localization_error[0]


def test_bounded_run_reports_theory_metadata_and_converges():
    config = CoppeliaConfig(
        duration=50.0,
        dt=0.004,
        noise_model="bounded",
        backend="mock",
        seed=3,
        save_plots=False,
        save_animation=False,
    )
    result = run_experiment(config)
    summary = result.summary()

    # theory metadata is always reported for a bounded run
    assert summary["validation"]["bound_applicable"] is True
    assert summary["validation"]["epsilon"] is not None
    assert summary["validation"]["min_n_informed"] >= 1
    # The physically-sane default gains give ratio 200 (< the ~1730 sufficient
    # threshold), so the conservative gain condition is intentionally NOT claimed.
    assert summary["validation"]["gain_condition_passed"] is False
    # Localization must still make strong progress toward the source.
    assert result.localization_error[-1] < 0.4 * result.localization_error[0]


def test_summary_shape_matches_numeric_phase_format():
    config = CoppeliaConfig(duration=2.0, dt=0.02, backend="mock", save_plots=False, save_animation=False)
    summary = run_experiment(config).summary()
    for key in ("backend", "parameters", "validation", "metrics"):
        assert key in summary
    assert "gain_ratio" in summary["validation"]
    assert "final_localization_error" in summary["metrics"]


def test_experiment_records_telemetry_signals():
    config = CoppeliaConfig(
        duration=1.0, dt=0.02, backend="mock", save_plots=False, save_animation=False
    )
    result = run_experiment(config)
    steps = int(round(config.duration / config.dt)) + 1

    for arr in (result.commands, result.linear_velocity, result.angular_velocity,
                result.measurements, result.informed_mask):
        assert arr is not None
    assert result.commands.shape == (steps, config.n, 2)
    assert result.linear_velocity.shape == (steps, config.n)
    assert result.measurements.shape == (steps, config.n)
    assert result.informed_mask.shape == (steps, config.n)
    # The terminal step issues no command (NaN); every prior step has a finite one.
    assert np.all(np.isnan(result.commands[-1]))
    assert np.all(np.isfinite(result.commands[:-1]))
    # informed flags are boolean-valued and their per-step sum matches n_informed.
    assert set(np.unique(result.informed_mask)).issubset({0, 1})
    np.testing.assert_array_equal(result.informed_mask.sum(axis=1), result.n_informed)


def test_save_telemetry_writes_loadable_artifacts(tmp_path):
    config = CoppeliaConfig(
        duration=0.4, dt=0.02, backend="mock", save_plots=False, save_animation=False
    )
    result = run_experiment(config)
    paths = save_telemetry(result, tmp_path)

    assert paths["npz"].exists() and paths["csv"].exists()

    data = np.load(paths["npz"])
    np.testing.assert_allclose(data["control_points"], result.control_points)
    np.testing.assert_allclose(data["localization_error"], result.localization_error)

    with paths["csv"].open(encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    steps = int(round(config.duration / config.dt)) + 1
    assert rows[0] == CSV_COLUMNS
    assert len(rows) == 1 + steps * config.n  # header + one row per (step, robot)


class _FailAfter:
    """Wrap a backend so ``get_poses`` raises after ``fail_at`` successful reads."""

    name = "mock"

    def __init__(self, inner, fail_at, exc=RuntimeError("boom")):
        self._inner = inner
        self._fail_at = fail_at
        self._exc = exc
        self._reads = 0

    def connect(self):
        self._inner.connect()

    def get_poses(self):
        self._reads += 1
        if self._reads > self._fail_at:
            raise self._exc
        return self._inner.get_poses()

    def set_velocity_commands(self, v, omega):
        self._inner.set_velocity_commands(v, omega)

    def step(self, dt):
        self._inner.step(dt)

    def close(self):
        self._inner.close()


def test_interrupted_run_raises_partial_result_with_recorded_steps():
    config = CoppeliaConfig(
        duration=1.0, dt=0.02, backend="mock", save_plots=False, save_animation=False
    )
    backend = _FailAfter(make_backend(config), fail_at=13)
    try:
        run_experiment(config, backend=backend)
        raise AssertionError("expected PartialRunError")
    except PartialRunError as exc:
        result = exc.result
        assert exc.recorded == 13
        # Every array is truncated to exactly the recorded steps -- no zero padding.
        assert result.times.shape == (13,)
        assert result.positions.shape == (13, config.n, 2)
        assert result.control_points.shape == (13, config.n, 2)
        assert result.measurements.shape == (13, config.n)
        # The original cause is chained so the caller can report it.
        assert isinstance(exc.__cause__, RuntimeError)


def test_partial_result_saves_telemetry_and_plots(tmp_path):
    config = CoppeliaConfig(
        duration=1.0, dt=0.02, backend="mock", save_plots=False, save_animation=False
    )
    backend = _FailAfter(make_backend(config), fail_at=13)
    try:
        run_experiment(config, backend=backend)
    except PartialRunError as exc:
        save_telemetry(exc.result, tmp_path)
        save_plots(exc.result, tmp_path)
    assert (tmp_path / "telemetry.npz").exists()
    assert (tmp_path / "telemetry.csv").exists()
    assert (tmp_path / "trajectory.png").stat().st_size > 0
    # Telemetry row count matches the truncated step count.
    with (tmp_path / "telemetry.csv").open(encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    assert len(rows) == 1 + 13 * config.n


def test_immediate_failure_propagates_original_error():
    """A crash before 2 steps are recorded surfaces the real error, not a partial."""
    config = CoppeliaConfig(
        duration=1.0, dt=0.02, backend="mock", save_plots=False, save_animation=False
    )
    backend = _FailAfter(make_backend(config), fail_at=0, exc=ValueError("no sim"))
    try:
        run_experiment(config, backend=backend)
        raise AssertionError("expected the original error")
    except PartialRunError:
        raise AssertionError("should not wrap an empty run in PartialRunError")
    except ValueError as exc:
        assert str(exc) == "no sim"


def test_signal_field_is_quadratic_bowl_centred_on_source():
    """The plotted contour reuses signal_field; verify its bowl shape."""
    source = np.array([5.5, 5.5])
    kappa = 1.3
    # Zero at the source (minimum of the bowl).
    assert signal_field(source[None, :], source, kappa)[0] == 0.0
    # f = kappa * ||z - p_s||^2 at arbitrary offsets.
    points = np.array([[5.5, 7.5], [8.5, 5.5], [5.5, 5.5]])  # r = 2, 3, 0
    expected = kappa * np.array([4.0, 9.0, 0.0])
    np.testing.assert_allclose(signal_field(points, source, kappa), expected)


def test_save_plots_with_field_writes_trajectory(tmp_path):
    config = CoppeliaConfig(
        duration=0.4, dt=0.02, backend="mock", save_animation=False
    )
    result = run_experiment(config)
    save_plots(result, tmp_path)
    trajectory = tmp_path / "trajectory.png"
    assert trajectory.exists() and trajectory.stat().st_size > 0


def test_floor_covers_swarm_and_source_with_margin():
    config = CoppeliaConfig()  # default 6-robot layout, source (5.5, 5.5)
    cx, cy, sx, sy = _floor_extents(config)
    assert sx > 0 and sy > 0

    xmin, xmax = cx - sx / 2.0, cx + sx / 2.0
    ymin, ymax = cy - sy / 2.0, cy + sy / 2.0
    positions = config.resolved_initial_positions()
    source = config.source_array()
    points = np.vstack([positions, source[None, :]])
    # Every robot start and the source lie comfortably inside the floor (>= margin).
    assert np.all(points[:, 0] > xmin + 1.0) and np.all(points[:, 0] < xmax - 1.0)
    assert np.all(points[:, 1] > ymin + 1.0) and np.all(points[:, 1] < ymax - 1.0)


def test_floor_grows_with_floor_scale():
    from dataclasses import replace

    small = _floor_extents(CoppeliaConfig(floor_scale=1.0))
    large = _floor_extents(replace(CoppeliaConfig(), floor_scale=10.0))
    assert large[2] > small[2] and large[3] > small[3]


def test_contour_ring_radii_are_increasing_and_positive():
    config = CoppeliaConfig()  # default 6-robot layout
    radii = _contour_ring_radii(config)
    assert len(radii) == 6
    assert all(r > 0 for r in radii)
    assert radii == sorted(radii)
    # The outermost ring brackets the initial swarm spread.
    positions = config.resolved_initial_positions()
    max_dist = float(np.max(np.linalg.norm(positions - config.source_array(), axis=1)))
    assert radii[-1] == max_dist
