import math

import numpy as np

from coppelia.config import CoppeliaConfig
from coppelia.control import component_sign, formation_slots, measurements
from coppelia.theory import all_informed_epsilon, epsilon_bound, gain_ratio_threshold
from coppelia.unicycle import (
    control_points,
    feedback_linearize,
    unicycle_to_wheel_speeds,
    wheel_speeds_to_unicycle,
)


def test_component_sign_is_per_coordinate():
    values = np.array([[2.0, -3.0], [0.0, 4.0], [-0.1, 0.0]])
    actual = component_sign(values)
    np.testing.assert_array_equal(actual, np.array([[1.0, -1.0], [0.0, 1.0], [-1.0, 0.0]]))


def test_formation_slots_sum_to_zero():
    phi = formation_slots(6)
    np.testing.assert_allclose(np.sum(phi, axis=0), np.zeros(2), atol=1e-12)


def test_measurement_saturates_outside_range():
    config = CoppeliaConfig(noise_model="none", dmax=1.0, noise_bound=0.2)
    positions = np.array([[5.5, 5.5], [20.0, 20.0], [5.6, 5.5], [5.5, 5.6], [5.4, 5.5], [5.5, 5.4]])
    rng = np.random.default_rng(1)
    sigma, informed = measurements(positions, config, rng)
    assert not informed[1]
    assert math.isclose(sigma[1], 1.2)


def test_all_informed_bound_matches_remark_four():
    assert math.isclose(epsilon_bound(6, 6, 1.0, 2.0, 0.2), all_informed_epsilon(1.0, 2.0, 0.2))


def test_gain_threshold_uses_paper_formula():
    assert math.isclose(gain_ratio_threshold(6, 1.0, 12.0, 2.0, 0.2), 1730.4)


def test_feedback_linearization_recovers_commanded_velocity():
    # Given (v, omega), the control point velocity should be M @ [v, omega];
    # feedback_linearize should invert that exactly.
    headings = np.array([0.0, math.pi / 2, math.pi / 3])
    r = 0.05
    commands = np.array([[1.0, 0.5], [-0.3, 0.2], [0.4, -0.6]])
    v, omega = feedback_linearize(commands, headings, r)
    # Reconstruct s_dot = M @ [v, omega] and compare to commands.
    cos = np.cos(headings)
    sin = np.sin(headings)
    sx = cos * v - r * sin * omega
    sy = sin * v + r * cos * omega
    np.testing.assert_allclose(np.column_stack((sx, sy)), commands, atol=1e-12)


def test_feedback_linearization_rejects_zero_offset():
    try:
        feedback_linearize(np.zeros((1, 2)), np.zeros(1), 0.0)
    except ValueError:
        return
    raise AssertionError("expected ValueError for r = 0")


def test_wheel_speed_roundtrip():
    v = np.array([0.5, -0.2, 1.0])
    omega = np.array([0.1, -0.3, 0.0])
    left, right = unicycle_to_wheel_speeds(v, omega, 0.033, 0.16)
    v2, omega2 = wheel_speeds_to_unicycle(left, right, 0.033, 0.16)
    np.testing.assert_allclose(v, v2, atol=1e-12)
    np.testing.assert_allclose(omega, omega2, atol=1e-12)


def test_control_points_offset_along_heading():
    positions = np.array([[0.0, 0.0], [1.0, 1.0]])
    headings = np.array([0.0, math.pi / 2])
    s = control_points(positions, headings, 0.5)
    np.testing.assert_allclose(s, np.array([[0.5, 0.0], [1.0, 1.5]]), atol=1e-12)
