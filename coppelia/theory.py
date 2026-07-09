"""Theorem-1 helpers (self-contained copy of the Phase 1 theory formulas).

Kept identical to ``sgf_sim.theory`` so the Phase 3 CoppeliaSim run reports the
same gain condition and localization error bound as the numerical baseline.
"""

from __future__ import annotations

import math


def f_dmax(kappa: float, dmax: float, delta: float) -> float:
    """Saturation ceiling value f_Dmax = kappa * Dmax^2 + delta."""

    return kappa * dmax * dmax + delta


def gain_ratio_threshold(n: int, kappa: float, dmax: float, radius: float, delta: float) -> float:
    """Sufficient gain threshold alpha / beta = 4 n f_Dmax / R (Theorem 1)."""

    return 4.0 * n * f_dmax(kappa, dmax, delta) / radius


def epsilon_bound(n: int, n_informed: int, kappa: float, radius: float, delta: float) -> float:
    """Ultimate localization error bound epsilon from Theorem 1."""

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
    """Remark 4 simplification: n_informed = n gives epsilon = delta / (kappa R)."""

    return delta / (kappa * radius)
