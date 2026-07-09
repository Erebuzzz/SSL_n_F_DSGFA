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


def min_informed_for_valid_bound(n: int) -> int:
    """Smallest informed count in [1, n] with a positive epsilon denominator.

    The denominator ``2*pi*k - n*|sin(2*pi*k/n)|`` is strictly positive for every
    ``k`` in ``[1, n]`` (since ``theta - |sin theta| > 0`` on ``(0, 2*pi]``), so this
    is always **1**. It is exposed for explicit reporting/guarding. The real caveat
    is not validity but *inflation*: as the informed fraction ``k/n`` shrinks, the
    bound ``epsilon`` grows without limit (valid, but practically useless). With
    ``k = 0`` there is no source signal at all and localization is undefined.
    """

    if n < 1:
        raise ValueError("n must be >= 1")
    for k in range(1, n + 1):
        if 2.0 * math.pi * k - n * abs(math.sin(2.0 * math.pi * k / n)) > 0.0:
            return k
    raise ValueError("no valid informed count found")  # unreachable for n >= 1
