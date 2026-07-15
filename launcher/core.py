"""Headless core for the unified launcher.

No tkinter, no matplotlib import at module load -- only numpy plus the project's
own config/theory modules, so this can be unit-tested without a display.

Responsibilities:
  * hold the full run configuration (`LauncherState`),
  * gate invalid platform/mode combinations,
  * compute the pre-run theory readout (gain condition, epsilon, min-informed),
  * dispatch the run to the chosen platform.
"""

from __future__ import annotations

import math
import subprocess
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

# ----------------------------------------------------------------------
# platform / mode grid
# ----------------------------------------------------------------------
PLATFORMS = ("Python", "MATLAB", "CoppeliaSim")

# Which run modes each platform can actually execute (see the integration audit).
MODES_BY_PLATFORM: dict[str, tuple[str, ...]] = {
    "Python": ("single_integrator", "unicycle"),
    "MATLAB": ("single_integrator", "turtlebot_numeric", "turtlebot_simulink"),
    "CoppeliaSim": ("differential_drive",),  # mock or physics backend
}

NOISE_MODELS = ("none", "gaussian", "bounded")
SGN_MODES = ("exact", "boundary_layer")  # exact -> eps 0; boundary_layer -> eps 0.2


def valid_modes(platform: str) -> tuple[str, ...]:
    return MODES_BY_PLATFORM.get(platform, ())


PAPER_SOURCE = np.array([5.5, 5.5], dtype=float)
DEFAULT_POSITIONS = np.array(
    [[0.0, 0.0], [2.5, -0.5], [5.0, 0.0], [0.5, 3.5], [3.0, 4.0], [5.5, 3.0]],
    dtype=float,
)


def default_positions(n: int, source: tuple[float, float] | np.ndarray = (5.5, 5.5)) -> np.ndarray:
    """A deterministic starting layout for n robots, shifted to sit near ``source``
    so every robot starts within sensing range (the paper 6-layout for n=6,
    otherwise an evenly spread grid). source == PAPER_SOURCE reproduces the
    original paper layout exactly."""

    src = np.asarray(source, dtype=float)
    if n == 6:
        # keep the paper geometry, translate it to follow the source
        return DEFAULT_POSITIONS + (src - PAPER_SOURCE)
    # spread on a coarse grid, then center that grid on the source
    side = int(math.ceil(math.sqrt(n)))
    pts = np.array([[float(i % side), float(i // side)] for i in range(n)], dtype=float)
    return pts + (src - pts.mean(axis=0))


@dataclass
class LauncherState:
    """Everything the user can set. Mutated in place by the GUI."""

    platform: str = "Python"
    mode: str = "single_integrator"
    n: int = 6
    source: tuple[float, float] = (5.5, 5.5)
    kappa: float = 1.0
    radius: float = 2.0  # R
    dmax: float = 12.0
    alpha: float = 100.0
    beta: float = 0.05
    duration: float = 60.0
    dt: float = 0.001
    seed: int = 1
    noise_model: str = "bounded"
    noise_std: float = 0.2
    noise_bound: float = 0.2
    sgn_mode: str = "exact"  # exact | boundary_layer
    boundary_layer_width: float = 0.2  # used when sgn_mode == boundary_layer
    control_point_offset: float = 2.0  # r, for unicycle / turtlebot / coppelia
    coppelia_backend: str = "mock"  # mock | coppelia
    # per-robot state; None => defaults
    positions: np.ndarray | None = None
    informed: np.ndarray | None = None  # length-n 0/1 mask
    run_id: str = "launcher_run"
    output_dir: Path = field(default_factory=lambda: Path("outputs") / "launcher")

    # -- derived helpers ------------------------------------------------
    def resolved_positions(self) -> np.ndarray:
        if self.positions is None:
            return default_positions(self.n, self.source)
        p = np.asarray(self.positions, dtype=float)
        if p.shape != (self.n, 2):
            raise ValueError(f"positions must have shape {(self.n, 2)}")
        return p

    def resolved_informed(self) -> np.ndarray:
        if self.informed is None:
            return np.ones(self.n, dtype=int)
        m = np.asarray(self.informed, dtype=int)
        if m.shape != (self.n,):
            raise ValueError(f"informed must have shape {(self.n,)}")
        if not np.all(np.isin(m, (0, 1))):
            raise ValueError("informed entries must be 0 or 1")
        return m

    def sign_boundary_layer(self) -> float:
        return self.boundary_layer_width if self.sgn_mode == "boundary_layer" else 0.0

    def validate(self) -> list[str]:
        """Return a list of human-readable problems (empty => OK)."""

        problems: list[str] = []
        if self.platform not in PLATFORMS:
            problems.append(f"unknown platform {self.platform!r}")
        elif self.mode not in valid_modes(self.platform):
            problems.append(
                f"mode {self.mode!r} is not available on {self.platform} "
                f"(choose one of: {', '.join(valid_modes(self.platform))})"
            )
        if self.n < 3:
            problems.append("n must be >= 3 (paper identities require n > 2)")
        if self.radius <= 0 or self.dmax <= 0 or self.radius > self.dmax:
            problems.append("require 0 < R <= Dmax")
        if self.alpha <= 0 or self.beta <= 0:
            problems.append("alpha and beta must be positive")
        if self.dt <= 0 or self.duration <= 0:
            problems.append("dt and duration must be positive")
        if self.noise_model not in NOISE_MODELS:
            problems.append(f"noise_model must be one of {NOISE_MODELS}")
        try:
            self.resolved_positions()
        except ValueError as exc:
            problems.append(str(exc))
        try:
            self.resolved_informed()
        except ValueError as exc:
            problems.append(str(exc))
        return problems


# ----------------------------------------------------------------------
# pre-run theory readout
# ----------------------------------------------------------------------
def _f_dmax(kappa: float, dmax: float, delta: float) -> float:
    return kappa * dmax * dmax + delta


def _gain_threshold(n: int, kappa: float, dmax: float, radius: float, delta: float) -> float:
    return 4.0 * n * _f_dmax(kappa, dmax, delta) / radius


def _epsilon(n: int, n_informed: int, kappa: float, radius: float, delta: float) -> float | None:
    if not 1 <= n_informed <= n:
        return None
    denom = kappa * radius * (
        2.0 * math.pi * n_informed - n * abs(math.sin(2.0 * math.pi * n_informed / n))
    )
    if denom <= 0:
        return None
    return 2.0 * math.pi * n * delta / denom


@dataclass
class TheoryReadout:
    gain_ratio: float
    gain_threshold: float
    gain_condition_passed: bool
    epsilon_all_informed: float
    initial_informed: int
    epsilon_initial_informed: float | None
    epsilon_inflation: float | None
    min_informed_for_valid_bound: int
    bound_applicable: bool
    notes: list[str]

    def as_lines(self) -> list[str]:
        lines = [
            f"gain ratio        = {self.gain_ratio:.1f}",
            f"gain threshold    = {self.gain_threshold:.1f}   "
            f"({'PASS' if self.gain_condition_passed else 'FAIL (sufficient cond. not met)'})",
            f"epsilon (all informed) = {self.epsilon_all_informed:.4f}",
            f"initial informed  = {self.initial_informed}",
        ]
        if self.epsilon_initial_informed is not None:
            infl = f" (x{self.epsilon_inflation:.1f} inflation)" if self.epsilon_inflation else ""
            lines.append(f"epsilon (initial informed) = {self.epsilon_initial_informed:.4f}{infl}")
        lines.append(f"min informed for valid bound = {self.min_informed_for_valid_bound}")
        if not self.bound_applicable:
            lines.append("bound N/A (gaussian noise or zero informed robots)")
        lines.extend(self.notes)
        return lines


def _min_informed_for_valid_bound(n: int) -> int:
    for k in range(1, n + 1):
        if 2.0 * math.pi * k - n * abs(math.sin(2.0 * math.pi * k / n)) > 0.0:
            return k
    return 1


def compute_readout(state: LauncherState) -> TheoryReadout:
    """Compute the gain condition, epsilon bound, and informed-count diagnostics
    without running any simulation. `initial informed` is derived from the initial
    positions + mask (how many robots start within Dmax AND are sensing-capable)."""

    delta = state.noise_bound
    ratio = state.alpha / state.beta if state.beta else float("inf")
    thr = _gain_threshold(state.n, state.kappa, state.dmax, state.radius, delta)
    eps_all = delta / (state.kappa * state.radius)

    positions = state.resolved_positions()
    mask = state.resolved_informed().astype(bool)
    source = np.asarray(state.source, dtype=float)
    within = np.linalg.norm(positions - source, axis=1) < state.dmax
    initial_informed = int(np.sum(within & mask))

    bound_applicable = state.noise_model in ("bounded", "none") and initial_informed > 0
    eps_init = _epsilon(state.n, initial_informed, state.kappa, state.radius, delta)
    inflation = (eps_init / eps_all) if (eps_init is not None and eps_all > 0) else None

    notes: list[str] = []
    if initial_informed == 0:
        notes.append("WARNING: no robot starts within sensing range -> no source signal; "
                     "localization cannot converge.")
    elif initial_informed < state.n:
        notes.append(f"NOTE: only {initial_informed}/{state.n} robots start informed; "
                     "epsilon is inflated vs the all-informed case.")
    if not state.noise_model == "bounded" and state.noise_model != "none":
        notes.append("NOTE: theorem bound assumes bounded noise; gaussian is paper-like only.")

    return TheoryReadout(
        gain_ratio=ratio,
        gain_threshold=thr,
        gain_condition_passed=ratio > thr,
        epsilon_all_informed=eps_all,
        initial_informed=initial_informed,
        epsilon_initial_informed=eps_init,
        epsilon_inflation=inflation,
        min_informed_for_valid_bound=_min_informed_for_valid_bound(state.n),
        bound_applicable=bound_applicable,
        notes=notes,
    )


# ----------------------------------------------------------------------
# shared-JSON serialization (for MATLAB dispatch + reproducibility)
# ----------------------------------------------------------------------
def _mode_to_json_mode(state: LauncherState) -> str:
    if state.platform == "MATLAB":
        return state.mode  # single_integrator / turtlebot_numeric / turtlebot_simulink
    if state.platform == "CoppeliaSim":
        return "coppelia"
    return state.mode


def to_shared_json(state: LauncherState) -> dict[str, Any]:
    """Serialize the state into the shared JSON schema (the one all paths read)."""

    positions = state.resolved_positions().tolist()
    informed = state.resolved_informed().tolist()
    data: dict[str, Any] = {
        "experiment": {
            "name": state.run_id,
            "mode": _mode_to_json_mode(state),
            "seed": state.seed,
            "duration": state.duration,
            "dt": state.dt,
        },
        "paper_parameters": {
            "n": state.n,
            "source": list(state.source),
            "kappa": state.kappa,
            "R": state.radius,
            "Dmax": state.dmax,
            "alpha": state.alpha,
            "beta": state.beta,
        },
        "noise": {
            "model": state.noise_model,
            "std": state.noise_std,
            "bound": state.noise_bound,
        },
        # A scalable topology by default so n != 6 works out of the box.
        "topology": {"name": "ring" if state.n != 6 else "paper_fig1_reconstructed",
                     "adjacency": None, "edges": None},
        "robot_model": {
            "type": state.mode,
            "unicycle_shift_r": state.control_point_offset,
            "max_linear_velocity": None,
            "max_angular_velocity": None,
        },
        "controller": {"sign_boundary_layer": state.sign_boundary_layer()},
        "initial_conditions": {"positions": positions, "informed": informed},
        "outputs": {
            "folder": str(state.output_dir),
            "run_id": state.run_id,
            "save_plots": True,
            "save_report": True,
            "save_animation": True,
            "animation_format": "gif",
            "animation_fps": 20,
        },
    }
    return data


# ----------------------------------------------------------------------
# dispatch
# ----------------------------------------------------------------------
def dispatch(state: LauncherState, *, matlab_exe: str = "matlab") -> dict[str, Any]:
    """Run the configured experiment on the chosen platform.

    Returns a dict describing the outcome: ``{"platform", "output_dir", ...}`` plus
    ``"summary"`` for in-process Python/CoppeliaSim runs, or ``"command"`` /
    ``"stdout"`` for the MATLAB subprocess. Raises ValueError on invalid config.
    """

    problems = state.validate()
    if problems:
        raise ValueError("invalid configuration:\n  - " + "\n  - ".join(problems))

    if state.platform == "Python":
        return _dispatch_python(state)
    if state.platform == "CoppeliaSim":
        return _dispatch_coppelia(state)
    if state.platform == "MATLAB":
        return _dispatch_matlab(state, matlab_exe=matlab_exe)
    raise ValueError(f"unknown platform {state.platform!r}")


def _dispatch_python(state: LauncherState) -> dict[str, Any]:
    out = Path(state.output_dir) / state.run_id
    if state.mode == "single_integrator":
        from sgf_sim.config import SimulationConfig
        from sgf_sim.simulation import run_simulation
        from sgf_sim.plotting import save_plots

        cfg = SimulationConfig(
            n=state.n, source=tuple(state.source), kappa=state.kappa,
            radius=state.radius, dmax=state.dmax, alpha=state.alpha, beta=state.beta,
            duration=state.duration, dt=state.dt, seed=state.seed,
            noise_model=state.noise_model, noise_std=state.noise_std,
            noise_bound=state.noise_bound,
            topology="ring" if state.n != 6 else "paper_fig1_reconstructed",
            sign_boundary_layer=state.sign_boundary_layer(),
            initial_positions=state.resolved_positions(),
            informed=state.resolved_informed(),
        )
        result = run_simulation(cfg)
        out.mkdir(parents=True, exist_ok=True)
        save_plots(result, out)
        import json
        (out / "summary.json").write_text(json.dumps(result.summary(), indent=2))
        return {"platform": "Python", "mode": state.mode, "output_dir": str(out),
                "summary": result.summary()}

    if state.mode == "unicycle":
        from sgf_sim.config import SimulationConfig
        from sgf_sim.unicycle import (
            UnicycleConfig, run_unicycle_simulation, save_unicycle_run,
        )

        base = SimulationConfig(
            n=state.n, source=tuple(state.source), kappa=state.kappa,
            radius=state.radius, dmax=state.dmax, alpha=state.alpha, beta=state.beta,
            duration=state.duration, dt=state.dt, seed=state.seed,
            noise_model=state.noise_model, noise_std=state.noise_std,
            noise_bound=state.noise_bound,
            topology="ring" if state.n != 6 else "paper_fig1_reconstructed",
            sign_boundary_layer=state.sign_boundary_layer(),
            initial_positions=state.resolved_positions(),
            informed=state.resolved_informed(),
        )
        ucfg = UnicycleConfig(base=base, control_point_offset=state.control_point_offset)
        result = run_unicycle_simulation(ucfg)
        save_unicycle_run(result, out)
        return {"platform": "Python", "mode": state.mode, "output_dir": str(out),
                "summary": result.summary()}

    raise ValueError(f"unsupported Python mode {state.mode!r}")


def _dispatch_coppelia(state: LauncherState) -> dict[str, Any]:
    from coppelia.config import CoppeliaConfig
    from coppelia.experiment import run_experiment
    from coppelia.plotting import save_plots, save_animation
    from coppelia.reporting import write_report

    cfg = CoppeliaConfig(
        n=state.n, source=tuple(state.source), kappa=state.kappa,
        radius=state.radius, dmax=state.dmax, alpha=state.alpha, beta=state.beta,
        duration=state.duration, dt=state.dt, seed=state.seed,
        noise_model=state.noise_model, noise_std=state.noise_std,
        noise_bound=state.noise_bound,
        topology="ring" if state.n != 6 else "paper_fig1_reconstructed",
        sign_boundary_layer=state.sign_boundary_layer(),
        control_point_offset=state.control_point_offset,
        backend=state.coppelia_backend,
        initial_positions=state.resolved_positions(),
        informed=state.resolved_informed(),
        output_dir=Path(state.output_dir),
    )
    result = run_experiment(cfg)
    out = Path(state.output_dir) / state.run_id
    out.mkdir(parents=True, exist_ok=True)
    save_plots(result, out)
    anim = save_animation(result, out) if cfg.save_animation else None
    write_report(result, out, title=state.run_id, animation_path=anim)
    import json
    (out / "summary.json").write_text(json.dumps(result.summary(), indent=2))
    return {"platform": "CoppeliaSim", "backend": state.coppelia_backend,
            "output_dir": str(out), "summary": result.summary()}


def _dispatch_matlab(state: LauncherState, *, matlab_exe: str = "matlab") -> dict[str, Any]:
    import json

    cfg_dir = Path(state.output_dir)
    cfg_dir.mkdir(parents=True, exist_ok=True)
    cfg_path = cfg_dir / f"{state.run_id}.json"
    cfg_path.write_text(json.dumps(to_shared_json(state), indent=2), encoding="utf-8")

    if state.mode == "single_integrator":
        addpath, func = "matlab", "run_from_config"
    else:  # turtlebot_numeric / turtlebot_simulink
        addpath, func = "matlab_turtlebot", "run_turtlebot_from_config"

    abs_cfg = str(cfg_path.resolve()).replace("\\", "/")
    matlab_cmd = (
        f"addpath('{addpath}'); addpath('matlab'); "
        f"addpath('matlab_turtlebot/helpers'); {func}('{abs_cfg}')"
    )
    command = [matlab_exe, "-batch", matlab_cmd]
    try:
        proc = subprocess.run(command, capture_output=True, text=True, timeout=1800)
        return {"platform": "MATLAB", "mode": state.mode, "config": str(cfg_path),
                "command": " ".join(command), "returncode": proc.returncode,
                "stdout": proc.stdout, "stderr": proc.stderr}
    except FileNotFoundError:
        # MATLAB not on PATH -> hand back the exact command to run manually.
        return {"platform": "MATLAB", "mode": state.mode, "config": str(cfg_path),
                "command": " ".join(command), "returncode": None,
                "stdout": "", "stderr": f"MATLAB executable {matlab_exe!r} not found on PATH; "
                                        "run the printed command manually."}

