"""Configuration objects for the sign gradient-free simulator."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int_]


def default_initial_positions() -> FloatArray:
    """Return a deterministic six-robot starting layout near the paper source."""

    return np.array(
        [
            [0.0, 0.0],
            [2.5, -0.5],
            [5.0, 0.0],
            [0.5, 3.5],
            [3.0, 4.0],
            [5.5, 3.0],
        ],
        dtype=float,
    )


@dataclass(frozen=True)
class SimulationConfig:
    """All parameters needed for one numerical experiment."""

    n: int = 6
    source: tuple[float, float] = (5.5, 5.5)
    kappa: float = 1.0
    radius: float = 2.0
    dmax: float = 12.0
    alpha: float = 100.0
    beta: float = 0.05
    duration: float = 60.0
    dt: float = 0.0005
    seed: int = 1
    noise_model: str = "gaussian"
    noise_std: float = 0.2
    noise_bound: float = 0.2
    topology: str = "paper_fig1_reconstructed"
    # Boundary-layer sliding-mode width for the formation term. 0.0 (default) uses
    # the paper's exact component-wise sgn; > 0 replaces sgn(x) with the saturated
    # approximation sat(x / eps) = clip(x / eps, -1, 1), which removes the
    # chattering that corrupts localization on unicycle/differential-drive robots.
    # eps = 0 keeps MATLAB<->Python parity bit-identical.
    sign_boundary_layer: float = 0.0
    initial_positions: FloatArray | None = None
    # Optional per-robot "can sense the source" mask (1 = informed-capable,
    # 0 = forced blind). None => all robots informed-capable. The EFFECTIVE
    # informed status is still gated by the paper's sensing-radius rule at
    # runtime: a capable robot only senses while within Dmax of the source.
    informed: IntArray | None = None
    adjacency: IntArray | None = None
    output_dir: Path = field(default_factory=lambda: Path("outputs") / "runs")

    def resolved_initial_positions(self) -> FloatArray:
        if self.initial_positions is None:
            positions = default_initial_positions()
        else:
            positions = np.asarray(self.initial_positions, dtype=float)
        if positions.shape != (self.n, 2):
            raise ValueError(f"initial_positions must have shape {(self.n, 2)}")
        return positions.copy()

    def resolved_informed_mask(self) -> NDArray[np.bool_]:
        """Per-robot sensing-capability mask (all True when unset)."""

        if self.informed is None:
            return np.ones(self.n, dtype=bool)
        mask = np.asarray(self.informed, dtype=int)
        if mask.shape != (self.n,):
            raise ValueError(f"informed must have shape {(self.n,)}")
        if not np.all(np.isin(mask, (0, 1))):
            raise ValueError("informed entries must be 0 or 1")
        return mask.astype(bool)

    def resolved_adjacency(self) -> IntArray:
        if self.adjacency is None:
            from .topology import adjacency_for_topology, is_connected

            adjacency = adjacency_for_topology(self.topology, self.n)
        else:
            from .topology import is_connected

            adjacency = np.asarray(self.adjacency, dtype=int)
        if adjacency.shape != (self.n, self.n):
            raise ValueError(f"adjacency must have shape {(self.n, self.n)}")
        if np.any(np.diag(adjacency) != 0):
            raise ValueError("adjacency diagonal must be zero")
        if not np.array_equal(adjacency, adjacency.T):
            raise ValueError("phase 1 requires an undirected adjacency matrix")
        if not is_connected(adjacency):
            raise ValueError("adjacency must be connected")
        return adjacency.copy()

    def source_array(self) -> FloatArray:
        return np.asarray(self.source, dtype=float)

    def validate(self) -> None:
        if self.n <= 2:
            raise ValueError("the paper identities require n > 2")
        if self.radius <= 0:
            raise ValueError("radius must be positive")
        if self.radius > self.dmax:
            raise ValueError("radius must satisfy R <= Dmax")
        if self.dmax <= 0:
            raise ValueError("dmax must be positive")
        if self.alpha <= 0 or self.beta <= 0:
            raise ValueError("alpha and beta must be positive")
        if self.duration <= 0 or self.dt <= 0:
            raise ValueError("duration and dt must be positive")
        if self.noise_model not in {"none", "gaussian", "bounded"}:
            raise ValueError("noise_model must be one of: none, gaussian, bounded")
        if self.noise_std < 0 or self.noise_bound < 0:
            raise ValueError("noise magnitudes must be non-negative")
        if self.sign_boundary_layer < 0:
            raise ValueError("sign_boundary_layer must be non-negative (0 = exact sgn)")
        self.resolved_initial_positions()
        self.resolved_informed_mask()
        self.resolved_adjacency()
