import json
from pathlib import Path

import numpy as np
import pytest

from sgf_sim.config import SimulationConfig
from sgf_sim.experiments import run_experiment_suite
from sgf_sim.shared_config import load_shared_config
from sgf_sim.simulation import run_simulation
from sgf_sim.topology import TOPOLOGY_NAMES, adjacency_for_topology, edge_list, is_connected


def test_default_run_completes_and_improves_errors():
    config = SimulationConfig(duration=5.0, dt=0.01, noise_model="none")

    result = run_simulation(config)

    assert result.positions.shape == (501, 6, 2)
    assert result.formation_error[-1] < result.formation_error[0]
    assert result.localization_error[-1] < result.localization_error[0]


def test_bounded_validation_reports_theory_metadata():
    config = SimulationConfig(duration=10.0, dt=0.01, noise_model="bounded", seed=4)

    result = run_simulation(config)
    summary = result.summary()

    assert summary["validation"]["gain_condition_passed"] is True
    assert summary["validation"]["bound_applicable"] is True
    assert summary["validation"]["epsilon"] is not None
    assert summary["validation"]["min_n_informed"] >= 1


def test_summary_includes_convergence_metrics():
    config = SimulationConfig(duration=5.0, dt=0.01, noise_model="none")

    summary = run_simulation(config).summary()
    metrics = summary["metrics"]

    assert "formation_entry_time" in metrics
    assert "localization_entry_time" in metrics
    assert "time_inside_localization_threshold_after_entry" in metrics
    assert metrics["formation_threshold"] > 0
    assert metrics["localization_threshold"] > 0


def test_named_topologies_are_undirected_and_connected():
    for name in TOPOLOGY_NAMES:
        adjacency = adjacency_for_topology(name, 6)

        assert adjacency.shape == (6, 6)
        assert np.array_equal(adjacency, adjacency.T)
        assert is_connected(adjacency)


def test_paper_fig1_reconstructed_edges_are_documented():
    adjacency = adjacency_for_topology("paper_fig1_reconstructed", 6)

    assert edge_list(adjacency) == [
        (0, 1),
        (0, 2),
        (0, 5),
        (1, 2),
        (1, 4),
        (2, 3),
        (3, 4),
        (4, 5),
    ]


def test_experiment_suite_writes_aggregate_reports(tmp_path):
    config = SimulationConfig(duration=1.0, dt=0.01, noise_model="bounded", output_dir=tmp_path)

    rows = run_experiment_suite("paper-suite", config, tmp_path)

    assert len(rows) == 2
    assert (tmp_path / "paper-suite" / "summary.json").exists()
    assert (tmp_path / "paper-suite" / "summary.csv").exists()
    assert (tmp_path / "paper-suite" / "report.md").exists()
    loaded = json.loads((tmp_path / "paper-suite" / "summary.json").read_text())
    assert loaded[0]["suite"] == "paper-suite"


def test_shared_config_loads_paper_default():
    shared = load_shared_config(Path("configs") / "paper_default.json")

    assert shared.mode == "single_integrator"
    assert shared.run_id == "config_paper_default"
    assert shared.simulation.noise_model == "gaussian"
    assert shared.simulation.topology == "paper_fig1_reconstructed"
    assert shared.simulation.resolved_adjacency().shape == (6, 6)


def test_shared_config_blocks_planned_modes():
    with pytest.raises(NotImplementedError):
        load_shared_config(Path("configs") / "unicycle_default.json")

def test_matlab_phase13_parity_files_are_present():
    expected_files = [
        "README.md",
        "run_from_config.m",
        "run_paper_validation.m",
        "run_single_simulation.m",
        "sgf_config.m",
        "sgf_control.m",
        "sgf_measurement.m",
        "sgf_theory_bounds.m",
        "sgf_topology.m",
        "sgf_plot_results.m",
    ]

    for filename in expected_files:
        assert (Path("matlab") / filename).exists()

    config_parser = (Path("matlab") / "sgf_config.m").read_text()
    assert "jsondecode" in config_parser
    assert "single_integrator" in config_parser

