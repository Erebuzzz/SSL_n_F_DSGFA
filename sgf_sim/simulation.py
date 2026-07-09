"""Numerical integration and metrics for the sign gradient-free algorithm."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import FloatArray, IntArray, SimulationConfig
from .control import control_input, formation_slots, measurements, shifted_positions
from .theory import (
    all_informed_epsilon,
    epsilon_bound,
    gain_ratio_threshold,
    min_informed_for_valid_bound,
)


@dataclass(frozen=True)
class SimulationResult:
    """Complete time history and derived metrics for one run."""

    config: SimulationConfig
    times: FloatArray
    positions: FloatArray
    centroid: FloatArray
    formation_error: FloatArray
    localization_error: FloatArray
    n_informed: IntArray
    gain_threshold: float
    gain_ratio: float
    epsilon: float | None
    bound_applicable: bool

    def summary(self) -> dict[str, object]:
        final_formation = float(self.formation_error[-1])
        final_localization = float(self.localization_error[-1])
        inside_bound = None
        if self.epsilon is not None and self.bound_applicable:
            inside_bound = final_localization <= self.epsilon
        min_informed = int(np.min(self.n_informed))
        # Honest "informed robots matter" reporting: the bound is valid for any
        # informed count >= 1, but it inflates as the informed fraction shrinks.
        eps_all_informed = all_informed_epsilon(
            self.config.kappa, self.config.radius, self.config.noise_bound
        )
        epsilon_inflation = (
            float(self.epsilon / eps_all_informed)
            if (self.epsilon is not None and eps_all_informed > 0)
            else None
        )
        tail_start = int(0.75 * len(self.localization_error))
        localization_tail = self.localization_error[tail_start:]
        formation_tail = self.formation_error[tail_start:]
        formation_threshold = max(0.1, 0.05 * float(self.formation_error[0]))
        localization_threshold = self.epsilon if self.epsilon is not None else 0.1
        return {
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
                "sign_boundary_layer": self.config.sign_boundary_layer,
            },
            "validation": {
                "gain_ratio": self.gain_ratio,
                "gain_threshold": self.gain_threshold,
                "gain_condition_passed": self.gain_ratio > self.gain_threshold,
                "min_n_informed": min_informed,
                "final_n_informed": int(self.n_informed[-1]),
                "min_informed_for_valid_bound": min_informed_for_valid_bound(self.config.n),
                "all_informed": bool(min_informed == self.config.n),
                "no_source_signal": bool(min_informed == 0),
                "epsilon": self.epsilon,
                "epsilon_all_informed": eps_all_informed,
                "epsilon_inflation_factor": epsilon_inflation,
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
                "formation_threshold": formation_threshold,
                "formation_entry_time": _first_entry_time(self.times, self.formation_error, formation_threshold),
                "localization_threshold": localization_threshold,
                "localization_entry_time": _first_entry_time(self.times, self.localization_error, localization_threshold),
                "time_inside_localization_threshold_after_entry": _time_inside_after_entry(
                    self.times,
                    self.localization_error,
                    localization_threshold,
                ),
            },
        }


def _formation_error(positions: FloatArray, radius: float, phi: FloatArray) -> float:
    z = shifted_positions(positions, radius, phi)
    z_centroid = np.mean(z, axis=0)
    return float(np.sqrt(np.sum((z - z_centroid) ** 2)))


def _first_entry_time(times: FloatArray, values: FloatArray, threshold: float) -> float | None:
    matches = np.flatnonzero(values <= threshold)
    if matches.size == 0:
        return None
    return float(times[int(matches[0])])


def _time_inside_after_entry(times: FloatArray, values: FloatArray, threshold: float) -> float:
    matches = np.flatnonzero(values <= threshold)
    if matches.size == 0 or len(times) < 2:
        return 0.0
    start = int(matches[0])
    dt = float(times[1] - times[0])
    return float(np.sum(values[start:] <= threshold) * dt)


def run_simulation(config: SimulationConfig) -> SimulationResult:
    """Run a fixed-step Euler simulation of the paper control law."""

    config.validate()
    rng = np.random.default_rng(config.seed)
    adjacency = config.resolved_adjacency()
    phi = formation_slots(config.n)
    source = config.source_array()
    steps = int(round(config.duration / config.dt))

    times = np.linspace(0.0, steps * config.dt, steps + 1)
    positions = np.zeros((steps + 1, config.n, 2), dtype=float)
    centroid = np.zeros((steps + 1, 2), dtype=float)
    formation_error = np.zeros(steps + 1, dtype=float)
    localization_error = np.zeros(steps + 1, dtype=float)
    n_informed = np.zeros(steps + 1, dtype=int)

    positions[0] = config.resolved_initial_positions()

    for step in range(steps + 1):
        current = positions[step]
        sigma, informed = measurements(current, config, rng)
        centroid[step] = np.mean(current, axis=0)
        formation_error[step] = _formation_error(current, config.radius, phi)
        localization_error[step] = float(np.linalg.norm(centroid[step] - source))
        n_informed[step] = int(np.sum(informed))
        if step == steps:
            break
        u = control_input(current, adjacency, phi, sigma, config)
        positions[step + 1] = current + config.dt * u

    min_informed = int(np.min(n_informed))
    epsilon = None
    if min_informed > 0:
        epsilon = epsilon_bound(
            config.n,
            min_informed,
            config.kappa,
            config.radius,
            config.noise_bound,
        )
    threshold = gain_ratio_threshold(
        config.n,
        config.kappa,
        config.dmax,
        config.radius,
        config.noise_bound,
    )
    return SimulationResult(
        config=config,
        times=times,
        positions=positions,
        centroid=centroid,
        formation_error=formation_error,
        localization_error=localization_error,
        n_informed=n_informed,
        gain_threshold=threshold,
        gain_ratio=config.alpha / config.beta,
        epsilon=epsilon,
        bound_applicable=config.noise_model in {"bounded", "none"} and min_informed > 0,
    )
