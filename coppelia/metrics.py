"""Error metrics and run summary in the numeric-phase format.

Metrics are computed on the robots' **control-point** positions ``s_i`` because
those are the states the single-integrator algorithm actually governs (the paper's
``p_i``). This keeps the CoppeliaSim metrics directly comparable to the Phase 1/2
numerical outputs.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import CoppeliaConfig, FloatArray, IntArray
from .control import formation_slots, shifted_positions
from .theory import epsilon_bound, gain_ratio_threshold


def formation_error(control_points: FloatArray, radius: float, phi: FloatArray) -> float:
    """Consensus error of shifted states: sqrt(sum ||z_i - z_mean||^2)."""

    z = shifted_positions(control_points, radius, phi)
    z_centroid = np.mean(z, axis=0)
    return float(np.sqrt(np.sum((z - z_centroid) ** 2)))


def localization_error(control_points: FloatArray, source: FloatArray) -> float:
    """Distance from the formation centroid p* to the source p_s."""

    centroid = np.mean(control_points, axis=0)
    return float(np.linalg.norm(centroid - source))


@dataclass(frozen=True)
class CoppeliaResult:
    """Complete time history and derived metrics for one CoppeliaSim/mock run."""

    config: CoppeliaConfig
    backend_name: str
    times: FloatArray
    positions: FloatArray  # (steps+1, n, 2) robot centers
    control_points: FloatArray  # (steps+1, n, 2)
    headings: FloatArray  # (steps+1, n)
    centroid: FloatArray  # (steps+1, 2)
    formation_error: FloatArray  # (steps+1,)
    localization_error: FloatArray  # (steps+1,)
    n_informed: IntArray  # (steps+1,)
    gain_threshold: float
    gain_ratio: float
    epsilon: float | None
    bound_applicable: bool

    # --- per-step telemetry (see coppelia.telemetry); optional for callers that
    # build a result without the raw signals. The terminal step has no command,
    # so the last row of commands/linear_velocity/angular_velocity is NaN. ---
    commands: FloatArray | None = None  # (steps+1, n, 2) single-integrator f_i
    linear_velocity: FloatArray | None = None  # (steps+1, n) commanded v_i [m/s]
    angular_velocity: FloatArray | None = None  # (steps+1, n) commanded omega_i [rad/s]
    measurements: FloatArray | None = None  # (steps+1, n) field samples sigma_i
    informed_mask: IntArray | None = None  # (steps+1, n) per-robot informed flag (0/1)

    def summary(self) -> dict[str, object]:
        final_formation = float(self.formation_error[-1])
        final_localization = float(self.localization_error[-1])
        inside_bound = None
        if self.epsilon is not None and self.bound_applicable:
            inside_bound = final_localization <= self.epsilon
        tail_start = int(0.75 * len(self.localization_error))
        localization_tail = self.localization_error[tail_start:]
        formation_tail = self.formation_error[tail_start:]
        return {
            "backend": self.backend_name,
            "parameters": {
                "n": self.config.n,
                "source": list(self.config.source),
                "kappa": self.config.kappa,
                "radius": self.config.radius,
                "dmax": self.config.dmax,
                "alpha": self.config.alpha,
                "beta": self.config.beta,
                "duration": self.config.duration,
                "dt": self.config.dt,
                "seed": self.config.seed,
                "noise_model": self.config.noise_model,
                "noise_std": self.config.noise_std,
                "noise_bound": self.config.noise_bound,
                "topology": self.config.topology,
                "control_point_offset": self.config.control_point_offset,
                "sign_boundary_layer": self.config.sign_boundary_layer,
                "max_linear_velocity": self.config.max_linear_velocity,
                "max_angular_velocity": self.config.max_angular_velocity,
            },
            "validation": {
                "gain_ratio": self.gain_ratio,
                "gain_threshold": self.gain_threshold,
                "gain_condition_passed": self.gain_ratio > self.gain_threshold,
                "min_n_informed": int(np.min(self.n_informed)),
                "final_n_informed": int(self.n_informed[-1]),
                "epsilon": self.epsilon,
                "bound_applicable": self.bound_applicable,
                "inside_bound": inside_bound,
            },
            "metrics": {
                "initial_formation_error": float(self.formation_error[0]),
                "final_formation_error": final_formation,
                "initial_localization_error": float(self.localization_error[0]),
                "final_localization_error": final_localization,
                "formation_error_drop": float(self.formation_error[0] - final_formation),
                "localization_error_drop": float(self.localization_error[0] - final_localization),
                "tail_formation_error_span": float(np.max(formation_tail) - np.min(formation_tail)),
                "tail_formation_error_std": float(np.std(formation_tail)),
                "tail_localization_error_span": float(np.max(localization_tail) - np.min(localization_tail)),
                "tail_localization_error_std": float(np.std(localization_tail)),
            },
        }


def theory_metrics(config: CoppeliaConfig, min_informed: int) -> tuple[float, float | None, bool]:
    """Return (gain_threshold, epsilon, bound_applicable)."""

    threshold = gain_ratio_threshold(
        config.n, config.kappa, config.dmax, config.radius, config.noise_bound
    )
    epsilon = None
    if min_informed > 0:
        epsilon = epsilon_bound(
            config.n, min_informed, config.kappa, config.radius, config.noise_bound
        )
    bound_applicable = config.noise_model in {"bounded", "none"} and min_informed > 0
    return threshold, epsilon, bound_applicable


def make_formation_slots(n: int) -> FloatArray:
    return formation_slots(n)
