"""Theory helpers for Du et al. 2024 validation checks."""

from __future__ import annotations

import math


def f_dmax(kappa: float, dmax: float, delta: float) -> float:
    """Maximum sensed value used by the paper's saturation model."""

    return kappa * dmax * dmax + delta


def gain_ratio_threshold(n: int, kappa: float, dmax: float, radius: float, delta: float) -> float:
    """Return the sufficient gain threshold alpha / beta from Theorem 1."""

    return 4.0 * n * f_dmax(kappa, dmax, delta) / radius


def epsilon_bound(n: int, n_informed: int, kappa: float, radius: float, delta: float) -> float:
    """Return the theorem's ultimate localization error bound."""

    if not 1 <= n_informed <= n:
        raise ValueError("n_informed must be between 1 and n")
    denominator = kappa * radius * (
        2.0 * math.pi * n_informed
        - n * abs(math.sin(2.0 * math.pi * n_informed / n))
    )
    if denominator <= 0:
        raise ValueError("epsilon denominator must be positive")
    return 2.0 * math.pi * n * delta / denominator


def all_informed_epsilon(kappa: float, radius: float, delta: float) -> float:
    """Return the Remark 4 simplification for all robots informed."""

    return delta / (kappa * radius)
