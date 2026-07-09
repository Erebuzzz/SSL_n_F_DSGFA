"""Control-law and measurement primitives for the simulator."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from .config import FloatArray, IntArray, SimulationConfig


def component_sign(values: FloatArray) -> FloatArray:
    """Apply the paper's component-wise signum convention."""

    return np.sign(values).astype(float)


def saturated_sign(values: FloatArray, boundary_layer: float) -> FloatArray:
    """Component-wise sgn, or its boundary-layer saturation when the width > 0.

    ``boundary_layer <= 0`` returns the exact ``component_sign`` (the paper's
    controller). ``boundary_layer > 0`` returns ``clip(values / boundary_layer,
    -1, 1)``, the standard sliding-mode boundary-layer approximation that is
    continuous and vanishes at equilibrium (removing chattering).
    """

    if boundary_layer > 0:
        return np.clip(values / boundary_layer, -1.0, 1.0)
    return component_sign(values)


def formation_slots(n: int) -> FloatArray:
    """Return fixed circular slot vectors phi(theta_i)."""

    indices = np.arange(n, dtype=float)
    theta = 2.0 * np.pi * indices / n
    return np.column_stack((np.cos(theta), np.sin(theta)))


def shifted_positions(positions: FloatArray, radius: float, phi: FloatArray) -> FloatArray:
    """Return z_i = p_i - R phi_i."""

    return positions - radius * phi


def signal_field(points: FloatArray, source: FloatArray, kappa: float) -> FloatArray:
    """Evaluate f(z) = kappa ||z - p_s||^2 for each point."""

    delta = points - source
    return kappa * np.sum(delta * delta, axis=1)


def sample_noise(config: SimulationConfig, rng: np.random.Generator, count: int) -> FloatArray:
    """Sample measurement noise for the configured model."""

    if config.noise_model == "none":
        return np.zeros(count, dtype=float)
    if config.noise_model == "gaussian":
        return rng.normal(0.0, config.noise_std, size=count)
    if config.noise_model == "bounded":
        return rng.uniform(-config.noise_bound, config.noise_bound, size=count)
    raise ValueError(f"unsupported noise model: {config.noise_model}")


def measurements(
    positions: FloatArray,
    config: SimulationConfig,
    rng: np.random.Generator,
) -> tuple[FloatArray, NDArray[np.bool_]]:
    """Return sigma(p_i) and informed flags for all robots."""

    source = config.source_array()
    distances = np.linalg.norm(positions - source, axis=1)
    informed = distances < config.dmax
    sigma = np.full(
        config.n,
        config.kappa * config.dmax * config.dmax + config.noise_bound,
        dtype=float,
    )
    if np.any(informed):
        sigma[informed] = (
            signal_field(positions[informed], source, config.kappa)
            + sample_noise(config, rng, int(np.sum(informed)))
        )
    return sigma, informed


def control_input(
    positions: FloatArray,
    adjacency: IntArray,
    phi: FloatArray,
    sigma: FloatArray,
    config: SimulationConfig,
) -> FloatArray:
    """Compute Eq. 4 for every robot."""

    z = shifted_positions(positions, config.radius, phi)
    eps = config.sign_boundary_layer
    controls = np.zeros_like(positions)
    for i in range(config.n):
        neighbors = np.flatnonzero(adjacency[i])
        if neighbors.size:
            controls[i] += config.alpha * np.sum(saturated_sign(z[neighbors] - z[i], eps), axis=0)
        controls[i] -= (2.0 * config.beta / config.radius) * sigma[i] * phi[i]
    return controls
