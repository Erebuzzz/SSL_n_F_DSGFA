"""Numerical simulator for the sign gradient-free localization paper."""

from .config import SimulationConfig
from .simulation import SimulationResult, run_simulation
from .theory import epsilon_bound, gain_ratio_threshold

__all__ = [
    "SimulationConfig",
    "SimulationResult",
    "epsilon_bound",
    "gain_ratio_threshold",
    "run_simulation",
]
