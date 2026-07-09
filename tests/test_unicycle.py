import math

import numpy as np

from sgf_sim.config import SimulationConfig
from sgf_sim.control import (
    component_sign,
    control_input,
    formation_slots,
    measurements,
    saturated_sign,
)
from sgf_sim.unicycle import (
    UnicycleConfig,
    clip_commands,
    compare_single_integrator,
    control_points,
    feedback_linearize,
    run_unicycle_simulation,
    wrap_angle,
)


def test_feedback_linearization_inverts_the_unicycle_map():
    headings = np.array([0.0, math.pi / 2, math.pi / 3])
    r = 0.5
    commands = np.array([[1.0, 0.5], [-0.3, 0.2], [0.4, -0.6]])
    v, omega = feedback_linearize(commands, headings, r)
    cos, sin = np.cos(headings), np.sin(headings)
    sx = cos * v - r * sin * omega
    sy = sin * v + r * cos * omega
    np.testing.assert_allclose(np.column_stack((sx, sy)), commands, atol=1e-12)


def test_feedback_linearization_requires_positive_offset():
    try:
        feedback_linearize(np.zeros((1, 2)), np.zeros(1), 0.0)
    except ValueError:
        return
    raise AssertionError("expected ValueError for r=0")


def test_clip_commands_respects_limits():
    v = np.array([5.0, -5.0, 0.5])
    omega = np.array([10.0, -10.0, 0.1])
    cv, cw = clip_commands(v, omega, 1.0, 2.0)
    assert np.all(np.abs(cv) <= 1.0 + 1e-12)
    assert np.all(np.abs(cw) <= 2.0 + 1e-12)


def test_clip_commands_none_disables():
    v = np.array([5.0]); omega = np.array([9.0])
    cv, cw = clip_commands(v, omega, None, None)
    np.testing.assert_array_equal(cv, v)
    np.testing.assert_array_equal(cw, omega)


def test_control_points_offset_along_heading():
    positions = np.array([[0.0, 0.0], [1.0, 1.0]])
    headings = np.array([0.0, math.pi / 2])
    s = control_points(positions, headings, 0.5)
    np.testing.assert_allclose(s, np.array([[0.5, 0.0], [1.0, 1.5]]), atol=1e-12)


def test_wrap_angle_range():
    a = wrap_angle(np.array([0.0, 3 * math.pi, -3 * math.pi, math.pi]))
    assert np.all(a > -math.pi - 1e-9) and np.all(a <= math.pi + 1e-9)


def test_unicycle_config_validation():
    ok = UnicycleConfig(base=SimulationConfig(duration=1.0, dt=0.01))
    ok.validate()
    try:
        UnicycleConfig(base=SimulationConfig(), control_point_offset=0.0).validate()
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for offset 0")
    try:
        UnicycleConfig(base=SimulationConfig(), comms_period=-1.0).validate()
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for negative comms_period")


def test_unicycle_run_shapes_and_convergence_no_noise():
    # Unicycle-appropriate gains (the `unicycle` CLI defaults). The Phase 1
    # defaults (alpha=100) command angular velocities the unicycle map cannot
    # track -- see the module docstring.
    base = SimulationConfig(alpha=10.0, beta=0.05, duration=40.0, dt=0.004, noise_model="none")
    config = UnicycleConfig(base=base, control_point_offset=2.0, comms_period=0.0)
    result = run_unicycle_simulation(config)

    steps = int(round(base.duration / base.dt)) + 1
    assert result.positions.shape == (steps, 6, 2)
    assert result.control_points.shape == (steps, 6, 2)
    assert result.headings.shape == (steps, 6)
    # formation converges and holds; localization moves strongly toward source
    assert result.formation_error[-1] < 0.5
    assert result.localization_error[-1] < 0.5 * result.localization_error[0]


def test_unicycle_summary_shape_and_command_stats():
    base = SimulationConfig(duration=5.0, dt=0.01, noise_model="bounded")
    config = UnicycleConfig(base=base, max_linear_velocity=1.0, max_angular_velocity=3.0)
    summary = run_unicycle_simulation(config).summary()

    assert summary["model"] == "unicycle"
    for key in ("parameters", "validation", "metrics"):
        assert key in summary
    assert summary["parameters"]["comms_period"] == config.comms_period
    m = summary["metrics"]
    assert "max_commanded_linear_velocity" in m
    assert "max_commanded_angular_velocity" in m
    # limits must be respected
    assert m["max_commanded_linear_velocity"] <= 1.0 + 1e-9
    assert m["max_commanded_angular_velocity"] <= 3.0 + 1e-9


def test_sampled_comms_holds_between_ticks():
    # With T = 10*dt the command must be constant across each 10-step window.
    base = SimulationConfig(duration=1.0, dt=0.01, noise_model="none")
    config = UnicycleConfig(base=base, comms_period=0.1)
    result = run_unicycle_simulation(config)
    # commands recomputed every 10 steps; check a mid window is held constant
    window = result.commanded_v[10:20]
    assert np.allclose(window, window[0])


def test_compare_single_integrator_returns_metrics():
    base = SimulationConfig(duration=5.0, dt=0.01, noise_model="none")
    comparison, uni, si = compare_single_integrator(UnicycleConfig(base=base))
    assert "comparison" in comparison
    c = comparison["comparison"]
    assert "centroid_path_rmse" in c
    assert "unicycle_final_localization_error" in c
    assert c["centroid_path_rmse"] >= 0.0


# ----------------------------------------------------------------------
# boundary-layer (chattering-free) sign option
# ----------------------------------------------------------------------
def test_saturated_sign_zero_width_matches_pure_sign():
    x = np.array([[-2.0, 0.0, 0.3, 5.0, -0.01]])
    assert np.array_equal(saturated_sign(x, 0.0), component_sign(x))
    assert np.array_equal(saturated_sign(x, -1.0), component_sign(x))  # <=0 -> exact sgn


def test_saturated_sign_saturates_inside_layer():
    x = np.array([0.05, 0.1, 0.2, -0.3])
    out = saturated_sign(x, 0.1)
    assert np.allclose(out, np.clip(x / 0.1, -1.0, 1.0))
    assert out.max() <= 1.0 and out.min() >= -1.0


def test_control_input_default_is_exact_sgn():
    # Default config (sign_boundary_layer = 0) must reproduce the pure-sgn law,
    # so MATLAB<->Python parity and the Phase 1 golden are untouched.
    cfg = SimulationConfig(noise_model="none")
    assert cfg.sign_boundary_layer == 0.0
    rng = np.random.default_rng(1)
    phi = formation_slots(cfg.n)
    pos = cfg.resolved_initial_positions()
    adj = cfg.resolved_adjacency()
    sigma, _ = measurements(pos, cfg, rng)
    got = control_input(pos, adj, phi, sigma, cfg)

    z = pos - cfg.radius * phi
    ref = np.zeros_like(pos)
    for i in range(cfg.n):
        nb = np.flatnonzero(adj[i])
        if nb.size:
            ref[i] += cfg.alpha * np.sum(component_sign(z[nb] - z[i]), axis=0)
        ref[i] -= (2.0 * cfg.beta / cfg.radius) * sigma[i] * phi[i]
    assert np.array_equal(got, ref)


def test_boundary_layer_recovers_localization_on_unicycle():
    # eps > 0 removes the sgn chattering, so the unicycle control point tracks the
    # ideal single-integrator flow and localization drops far below the pure-sgn
    # case (which stalls ~0.8 from the source at these gains).
    def run(eps):
        base = SimulationConfig(
            alpha=10.0, beta=0.05, duration=40.0, dt=0.004,
            noise_model="none", sign_boundary_layer=eps,
        )
        return run_unicycle_simulation(UnicycleConfig(base=base, control_point_offset=2.0))

    pure = run(0.0)
    bl = run(0.2)
    assert pure.localization_error[-1] > 0.5            # pure sgn stalls
    assert bl.localization_error[-1] < 0.2              # boundary layer localizes
    assert bl.localization_error[-1] < 0.3 * pure.localization_error[-1]
    assert bl.formation_error[-1] < 0.1                 # and forms tightly


def test_sign_boundary_layer_rejects_negative():
    try:
        SimulationConfig(sign_boundary_layer=-0.1).validate()
    except ValueError:
        pass
    else:
        raise AssertionError("expected ValueError for negative sign_boundary_layer")
