import math

import numpy as np

from sgf_sim.control import component_sign, formation_slots, measurements
from sgf_sim.config import SimulationConfig
from sgf_sim.theory import all_informed_epsilon, epsilon_bound, gain_ratio_threshold


def test_component_sign_is_per_coordinate():
    values = np.array([[2.0, -3.0], [0.0, 4.0], [-0.1, 0.0]])

    actual = component_sign(values)

    np.testing.assert_array_equal(actual, np.array([[1.0, -1.0], [0.0, 1.0], [-1.0, 0.0]]))


def test_formation_slots_are_balanced():
    phi = formation_slots(6)

    np.testing.assert_allclose(np.sum(phi, axis=0), np.zeros(2), atol=1e-12)


def test_measurement_saturates_outside_sensing_range():
    config = SimulationConfig(noise_model="none", dmax=1.0, noise_bound=0.2)
    positions = np.array([[5.5, 5.5], [20.0, 20.0], [5.6, 5.5], [5.5, 5.6], [5.4, 5.5], [5.5, 5.4]])
    rng = np.random.default_rng(1)

    sigma, informed = measurements(positions, config, rng)

    assert not informed[1]
    assert math.isclose(sigma[1], 1.2)


def test_all_informed_bound_matches_remark_four():
    n = 6
    delta = 0.2
    radius = 2.0
    kappa = 1.0

    assert math.isclose(epsilon_bound(n, n, kappa, radius, delta), all_informed_epsilon(kappa, radius, delta))


def test_gain_threshold_uses_paper_formula():
    threshold = gain_ratio_threshold(n=6, kappa=1.0, dmax=12.0, radius=2.0, delta=0.2)

    assert math.isclose(threshold, 1730.4)
