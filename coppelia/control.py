"""Sign gradient-free control primitives (self-contained Phase 3 copy).

These mirror ``sgf_sim.control`` (Eq. 4 of Du et al. 2024) but operate on the
robots' *control-point* positions ``s_i`` -- the feedback-linearized point that
behaves as a single integrator. The single-integrator command ``f_i`` produced
here is fed to :mod:`coppelia.unicycle` to recover wheel-level ``(v, omega)``.

Critical paper details preserved:

- ``sgn`` is applied **component-wise** to the shifted-state difference (ternary
  ``{-1, 0, 1}`` communication), not as a normalized direction vector.
- The measurement saturates to ``kappa * Dmax^2 + noise_bound`` outside sensing
  range ``Dmax``.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from .config import CoppeliaConfig, FloatArray, IntArray


def component_sign(values: FloatArray) -> FloatArray:
    """Component-wise signum with sgn(0) = 0 (paper convention)."""

    return np.sign(values).astype(float)


def formation_slots(n: int) -> FloatArray:
    """Fixed circular slot vectors phi(theta_i), theta_i = 2 pi i / n."""

    indices = np.arange(n, dtype=float)
    theta = 2.0 * np.pi * indices / n
    return np.column_stack((np.cos(theta), np.sin(theta)))


def shifted_positions(positions: FloatArray, radius: float, phi: FloatArray) -> FloatArray:
    """Shifted (consensus) states z_i = s_i - R phi_i."""

    return positions - radius * phi


def signal_field(points: FloatArray, source: FloatArray, kappa: float) -> FloatArray:
    """Quadratic source field f(z) = kappa ||z - p_s||^2."""

    delta = points - source
    return kappa * np.sum(delta * delta, axis=1)


def sample_noise(config: CoppeliaConfig, rng: np.random.Generator, count: int) -> FloatArray:
    if config.noise_model == "none":
        return np.zeros(count, dtype=float)
    if config.noise_model == "gaussian":
        return rng.normal(0.0, config.noise_std, size=count)
    if config.noise_model == "bounded":
        return rng.uniform(-config.noise_bound, config.noise_bound, size=count)
    raise ValueError(f"unsupported noise model: {config.noise_model}")


def measurements(
    positions: FloatArray,
    config: CoppeliaConfig,
    rng: np.random.Generator,
) -> tuple[FloatArray, NDArray[np.bool_]]:
    """Return sigma(s_i) and informed flags for all robots."""

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
    config: CoppeliaConfig,
) -> FloatArray:
    """Compute the Eq. 4 single-integrator command f_i for every robot.

    ``positions`` are the control-point positions s_i (shape ``(n, 2)``).
    Returns the desired control-point velocities f_i (shape ``(n, 2)``).
    """

    z = shifted_positions(positions, config.radius, phi)
    eps = getattr(config, "sign_boundary_layer", 0.0)
    controls = np.zeros_like(positions)
    for i in range(config.n):
        neighbors = np.flatnonzero(adjacency[i])
        if neighbors.size:
            controls[i] += config.alpha * np.sum(_sat_sign(z[neighbors] - z[i], eps), axis=0)
        controls[i] -= (2.0 * config.beta / config.radius) * sigma[i] * phi[i]
    return controls


def _sat_sign(values: FloatArray, boundary_layer: float) -> FloatArray:
    """Component-wise sgn, or its boundary-layer saturation when width > 0."""

    if boundary_layer > 0:
        return np.clip(values / boundary_layer, -1.0, 1.0)
    return component_sign(values)
