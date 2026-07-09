"""Tests for the headless launcher core (no GUI/display needed)."""

from __future__ import annotations

import numpy as np

from launcher.core import (
    LauncherState,
    compute_readout,
    dispatch,
    to_shared_json,
    valid_modes,
)


def test_readout_all_informed_default():
    state = LauncherState(duration=1.0, dt=0.01)
    r = compute_readout(state)
    assert r.gain_ratio == 2000.0
    assert abs(r.gain_threshold - 1730.4) < 1e-6
    assert r.gain_condition_passed is True
    assert abs(r.epsilon_all_informed - 0.1) < 1e-9
    assert r.initial_informed == 6
    assert abs(r.epsilon_inflation - 1.0) < 1e-9


def test_readout_partial_informed_inflates_epsilon():
    state = LauncherState(duration=1.0, dt=0.01, informed=np.array([1, 1, 1, 0, 0, 0]))
    r = compute_readout(state)
    assert r.initial_informed == 3
    assert r.epsilon_initial_informed > r.epsilon_all_informed
    assert r.epsilon_inflation > 1.0


def test_readout_zero_informed_warns():
    # source far from all robots so none start within Dmax
    state = LauncherState(source=(1000.0, 1000.0), dmax=5.0)
    r = compute_readout(state)
    assert r.initial_informed == 0
    assert not r.bound_applicable
    assert any("no source signal" in n.lower() for n in r.notes)


def test_platform_mode_gating():
    assert "unicycle" not in valid_modes("MATLAB")
    bad = LauncherState(platform="MATLAB", mode="unicycle")
    problems = bad.validate()
    assert any("not available" in p for p in problems)


def test_shared_json_carries_positions_and_informed():
    data = to_shared_json(LauncherState(n=6))
    ic = data["initial_conditions"]
    assert np.array(ic["positions"]).shape == (6, 2)
    assert len(ic["informed"]) == 6


def test_dispatch_python_single_integrator(tmp_path):
    state = LauncherState(
        platform="Python", mode="single_integrator",
        duration=1.0, dt=0.01, run_id="t_si", output_dir=tmp_path,
    )
    result = dispatch(state)
    assert result["platform"] == "Python"
    out = tmp_path / "t_si"
    assert (out / "formation_error_per_robot.png").exists()
    assert (out / "summary.json").exists()


def test_dispatch_coppelia_mock(tmp_path):
    state = LauncherState(
        platform="CoppeliaSim", mode="differential_drive", coppelia_backend="mock",
        duration=1.0, dt=0.01, run_id="t_cop", output_dir=tmp_path,
    )
    result = dispatch(state)
    assert result["platform"] == "CoppeliaSim"
    assert (tmp_path / "t_cop" / "summary.json").exists()
