import numpy as np

from coppelia.backends import make_backend
from coppelia.backends.mock import MockBackend
from coppelia.config import CoppeliaConfig
from coppelia.experiment import run_experiment
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
