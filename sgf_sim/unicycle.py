"""Phase 2: numerical unicycle model for the sign gradient-free algorithm.

Phase 1 integrates single-integrator point robots (``p_dot_i = u_i``). Section V of
Du et al. runs the same control law on non-holonomic unicycle robots (TurtleBot3)
by feedback-linearizing a control point ``s_i`` shifted a distance ``r`` ahead of
the wheel axle:

    s_i = p_i + r [cos(theta_i), sin(theta_i)]

The single-integrator command ``f_i`` from Eq. 4 is then realized as body
velocities ``(v_i, omega_i) = M(theta_i, r)^{-1} f_i`` (see :func:`feedback_linearize`),
optionally saturated to actuator limits, with the sign messages exchanged on a
sampled communication period ``T`` (Section V uses ``T = 0.1 s``).

This module stays a *numerical* model (fast kinematic integration), reusing the
Phase 1 control/measurement/theory primitives so the mathematics is single-sourced
with the single-integrator simulator. It is additive: it does not modify any
existing Phase 1 module.

Design note on gains: the Phase 1 defaults (alpha=100, beta=0.05, ratio 2000)
command control-point speeds of hundreds of m/s. For a point integrator that is
fine, but the unicycle map turns them into enormous angular velocities
(omega ~ |f|/r), which no finite step can track. The unicycle CLI therefore
defaults to physically-sane gains (alpha=10, beta=0.05); the localization term is
then not starved and the model reaches the circular formation and localizes toward
the source. See ``coppelia/README.md`` (Phase 3) for the full theory-vs-physical
discussion, which applies identically here.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .config import FloatArray, SimulationConfig
from .control import control_input, formation_slots, measurements, shifted_positions
from .theory import epsilon_bound, gain_ratio_threshold


# ----------------------------------------------------------------------
# kinematics / feedback linearization
# ----------------------------------------------------------------------
def control_points(positions: FloatArray, headings: FloatArray, offset: float) -> FloatArray:
    """Return s_i = p_i + r [cos theta_i, sin theta_i] for all robots."""

    directions = np.column_stack((np.cos(headings), np.sin(headings)))
    return positions + offset * directions


def feedback_linearize(
    commands: FloatArray, headings: FloatArray, offset: float
) -> tuple[FloatArray, FloatArray]:
    """Convert single-integrator commands f_i into unicycle (v_i, omega_i).

    Inverts s_i_dot = M(theta, r) [v, omega]^T. Singular at r = 0, so r > 0.
    """

    if offset <= 0:
        raise ValueError("control-point offset r must be positive (M is singular at r=0)")
    cos = np.cos(headings)
    sin = np.sin(headings)
    fx = commands[:, 0]
    fy = commands[:, 1]
    v = cos * fx + sin * fy
    omega = (-sin * fx + cos * fy) / offset
    return v, omega


def clip_commands(
    v: FloatArray, omega: FloatArray, max_v: float | None, max_omega: float | None
) -> tuple[FloatArray, FloatArray]:
    """Saturate linear/angular velocity to configured limits (None disables)."""

    if max_v is not None:
        v = np.clip(v, -max_v, max_v)
    if max_omega is not None:
        omega = np.clip(omega, -max_omega, max_omega)
    return v, omega


def wrap_angle(theta: FloatArray) -> FloatArray:
    """Wrap angle(s) to (-pi, pi]."""

    return (theta + np.pi) % (2.0 * np.pi) - np.pi


# ----------------------------------------------------------------------
# configuration
# ----------------------------------------------------------------------
@dataclass(frozen=True)
class UnicycleConfig:
    """Unicycle-model parameters, composed around a Phase 1 SimulationConfig."""

    base: SimulationConfig = field(default_factory=SimulationConfig)
    control_point_offset: float = 2.0  # r > 0 [m]
    max_linear_velocity: float | None = None  # [m/s]; None disables
    max_angular_velocity: float | None = None  # [rad/s]; None disables
    # Sampled sign-communication period T [s]. 0 (default) => continuous, i.e.
    # recompute every dt, matching Section IV's instantaneous-communication
    # assumption. Set to 0.1 for the Section V sampled-data value (see the module
    # docstring / README: with the sign controller's large feedback-linearized
    # angular velocities, T=0.1 needs velocity limits to stay stable and does not
    # converge as tightly -- a documented numerical-vs-hardware difference).
    comms_period: float = 0.0
    initial_headings: FloatArray | None = None

    def resolved_initial_headings(self) -> FloatArray:
        n = self.base.n
        if self.initial_headings is None:
            return np.zeros(n, dtype=float)
        headings = np.asarray(self.initial_headings, dtype=float)
        if headings.shape != (n,):
            raise ValueError(f"initial_headings must have shape {(n,)}")
        return headings.copy()

    def validate(self) -> None:
        self.base.validate()
        if self.control_point_offset <= 0:
            raise ValueError("control_point_offset r must be positive")
        if self.comms_period < 0:
            raise ValueError("comms_period must be non-negative (0 = continuous)")
        if self.max_linear_velocity is not None and self.max_linear_velocity <= 0:
            raise ValueError("max_linear_velocity must be positive when set")
        if self.max_angular_velocity is not None and self.max_angular_velocity <= 0:
            raise ValueError("max_angular_velocity must be positive when set")
        self.resolved_initial_headings()


# ----------------------------------------------------------------------
# result
# ----------------------------------------------------------------------
@dataclass(frozen=True)
class UnicycleResult:
    """Time history and metrics for one unicycle run (metrics on control points)."""

    config: UnicycleConfig
    times: FloatArray
    positions: FloatArray  # (steps+1, n, 2) robot centers p_i
    control_points: FloatArray  # (steps+1, n, 2) s_i
    headings: FloatArray  # (steps+1, n)
    centroid: FloatArray  # (steps+1, 2) mean of s_i
    formation_error: FloatArray
    localization_error: FloatArray
    n_informed: FloatArray
    commanded_v: FloatArray  # (steps, n)
    commanded_omega: FloatArray  # (steps, n)
    gain_threshold: float
    gain_ratio: float
    epsilon: float | None
    bound_applicable: bool

    def summary(self) -> dict[str, object]:
        base = self.config.base
        final_formation = float(self.formation_error[-1])
        final_localization = float(self.localization_error[-1])
        inside_bound = None
        if self.epsilon is not None and self.bound_applicable:
            inside_bound = final_localization <= self.epsilon
        tail_start = int(0.75 * len(self.localization_error))
        localization_tail = self.localization_error[tail_start:]
        formation_tail = self.formation_error[tail_start:]
        max_v = float(np.max(np.abs(self.commanded_v))) if self.commanded_v.size else 0.0
        max_omega = float(np.max(np.abs(self.commanded_omega))) if self.commanded_omega.size else 0.0
        return {
            "model": "unicycle",
            "parameters": {
                "n": base.n,
                "source": list(base.source),
                "kappa": base.kappa,
                "radius": base.radius,
                "dmax": base.dmax,
                "alpha": base.alpha,
                "beta": base.beta,
                "duration": base.duration,
                "dt": base.dt,
                "seed": base.seed,
                "noise_model": base.noise_model,
                "noise_std": base.noise_std,
                "noise_bound": base.noise_bound,
                "topology": base.topology,
                "sign_boundary_layer": base.sign_boundary_layer,
                "control_point_offset": self.config.control_point_offset,
                "comms_period": self.config.comms_period,
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
                "max_commanded_linear_velocity": max_v,
                "max_commanded_angular_velocity": max_omega,
                "linear_limit_active": (
                    self.config.max_linear_velocity is not None
                    and max_v >= self.config.max_linear_velocity - 1e-9
                ),
                "angular_limit_active": (
                    self.config.max_angular_velocity is not None
                    and max_omega >= self.config.max_angular_velocity - 1e-9
                ),
            },
        }


def _formation_error(control_pts: FloatArray, radius: float, phi: FloatArray) -> float:
    z = shifted_positions(control_pts, radius, phi)
    z_centroid = np.mean(z, axis=0)
    return float(np.sqrt(np.sum((z - z_centroid) ** 2)))


# ----------------------------------------------------------------------
# simulation
# ----------------------------------------------------------------------
def run_unicycle_simulation(config: UnicycleConfig) -> UnicycleResult:
    """Run a fixed-step unicycle simulation with sampled sign communication."""

    config.validate()
    base = config.base
    rng = np.random.default_rng(base.seed)
    adjacency = base.resolved_adjacency()
    phi = formation_slots(base.n)
    source = base.source_array()
    r = config.control_point_offset
    steps = int(round(base.duration / base.dt))
    hold = max(1, int(round(config.comms_period / base.dt)))

    times = np.linspace(0.0, steps * base.dt, steps + 1)
    positions = np.zeros((steps + 1, base.n, 2), dtype=float)
    cp_history = np.zeros((steps + 1, base.n, 2), dtype=float)
    heading_history = np.zeros((steps + 1, base.n), dtype=float)
    centroid = np.zeros((steps + 1, 2), dtype=float)
    formation_error = np.zeros(steps + 1, dtype=float)
    localization_error = np.zeros(steps + 1, dtype=float)
    n_informed = np.zeros(steps + 1, dtype=int)
    commanded_v = np.zeros((steps, base.n), dtype=float)
    commanded_omega = np.zeros((steps, base.n), dtype=float)

    positions[0] = base.resolved_initial_positions()
    heading_history[0] = config.resolved_initial_headings()
    v = np.zeros(base.n, dtype=float)
    omega = np.zeros(base.n, dtype=float)

    for step in range(steps + 1):
        p = positions[step]
        theta = heading_history[step]
        s = control_points(p, theta, r)

        cp_history[step] = s
        centroid[step] = np.mean(s, axis=0)
        formation_error[step] = _formation_error(s, base.radius, phi)
        localization_error[step] = float(np.linalg.norm(centroid[step] - source))

        # Recompute the sampled control (and take a fresh measurement) only on
        # communication ticks; hold (v, omega) between ticks (zero-order hold).
        if step % hold == 0:
            sigma, informed = measurements(s, base, rng)
            n_informed[step] = int(np.sum(informed))
            f = control_input(s, adjacency, phi, sigma, base)
            v, omega = feedback_linearize(f, theta, r)
            v, omega = clip_commands(v, omega, config.max_linear_velocity, config.max_angular_velocity)
        else:
            # record informed count each step for min/final reporting
            distances = np.linalg.norm(s - source, axis=1)
            n_informed[step] = int(np.sum((distances < base.dmax) & base.resolved_informed_mask()))

        if step == steps:
            break

        commanded_v[step] = v
        commanded_omega[step] = omega
        positions[step + 1] = p + base.dt * np.column_stack((v * np.cos(theta), v * np.sin(theta)))
        heading_history[step + 1] = wrap_angle(theta + base.dt * omega)

    min_informed = int(np.min(n_informed))
    epsilon = None
    if min_informed > 0:
        epsilon = epsilon_bound(base.n, min_informed, base.kappa, base.radius, base.noise_bound)
    threshold = gain_ratio_threshold(base.n, base.kappa, base.dmax, base.radius, base.noise_bound)

    return UnicycleResult(
        config=config,
        times=times,
        positions=positions,
        control_points=cp_history,
        headings=heading_history,
        centroid=centroid,
        formation_error=formation_error,
        localization_error=localization_error,
        n_informed=n_informed,
        commanded_v=commanded_v,
        commanded_omega=commanded_omega,
        gain_threshold=threshold,
        gain_ratio=base.alpha / base.beta,
        epsilon=epsilon,
        bound_applicable=base.noise_model in {"bounded", "none"} and min_informed > 0,
    )


def compare_single_integrator(config: UnicycleConfig) -> dict[str, object]:
    """Run the unicycle model and the Phase 1 single integrator, and compare.

    Both use the same base parameters and seed. The single integrator is the
    Phase 1 ``run_simulation`` on ``config.base`` (its states are the control
    points directly). Comparison is on the control-point centroid path and the
    error metrics.
    """

    from .simulation import run_simulation

    uni = run_unicycle_simulation(config)
    si = run_simulation(config.base)

    # Align lengths (identical dt/duration => identical time grids).
    m = min(len(uni.times), len(si.times))
    centroid_rmse = float(
        np.sqrt(np.mean(np.sum((uni.centroid[:m] - si.centroid[:m]) ** 2, axis=1)))
    )
    final_centroid_gap = float(np.linalg.norm(uni.centroid[-1] - si.centroid[-1]))

    return {
        "unicycle": uni.summary(),
        "single_integrator": si.summary(),
        "comparison": {
            "centroid_path_rmse": centroid_rmse,
            "final_centroid_gap": final_centroid_gap,
            "unicycle_final_localization_error": float(uni.localization_error[-1]),
            "single_integrator_final_localization_error": float(si.localization_error[-1]),
            "unicycle_final_formation_error": float(uni.formation_error[-1]),
            "single_integrator_final_formation_error": float(si.formation_error[-1]),
        },
    }, uni, si


def unicycle_config_from_namespace(args) -> UnicycleConfig:
    """Build a UnicycleConfig from parsed CLI arguments."""

    base = SimulationConfig(
        alpha=args.alpha,
        beta=args.beta,
        radius=args.radius,
        dmax=args.dmax,
        duration=args.duration,
        dt=args.dt,
        seed=args.seed,
        noise_model=args.noise,
        noise_std=args.noise_std,
        noise_bound=args.noise_bound,
        topology=args.topology,
        sign_boundary_layer=args.sign_boundary_layer,
        output_dir=args.output_dir,
    )
    return UnicycleConfig(
        base=base,
        control_point_offset=args.offset,
        max_linear_velocity=args.max_v,
        max_angular_velocity=args.max_omega,
        comms_period=args.comms_period,
    )


# ----------------------------------------------------------------------
# plotting / artifacts
# ----------------------------------------------------------------------
def save_unicycle_run(result: UnicycleResult, output_dir: Path) -> None:
    """Write trajectory + error plots and summary.json for a unicycle run."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    base = result.config.base
    phi = formation_slots(base.n)
    source = base.source_array()

    # trajectory
    fig, ax = plt.subplots(figsize=(7, 6))
    stride = max(1, len(result.times) // 4000)
    for i in range(base.n):
        trail = result.control_points[::stride, i, :]
        ax.plot(trail[:, 0], trail[:, 1], linewidth=1.0, label=f"robot {i}")
        ax.scatter(result.control_points[0, i, 0], result.control_points[0, i, 1], marker="o", s=20)
        ax.scatter(result.control_points[-1, i, 0], result.control_points[-1, i, 1], marker="s", s=24)
    final_centroid = result.centroid[-1]
    circle = final_centroid + base.radius * phi
    circle = np.vstack((circle, circle[0]))
    ax.plot(circle[:, 0], circle[:, 1], "k:", linewidth=1.2, label="final target circle")
    ax.plot(result.centroid[::stride, 0], result.centroid[::stride, 1], "k-", linewidth=1.0, alpha=0.6, label="centroid path")
    ax.scatter(source[0], source[1], marker="*", s=130, color="red", label="source")
    ax.scatter(final_centroid[0], final_centroid[1], marker="x", s=70, color="black", label="final centroid")
    ax.set_title("Unicycle Robot Trajectories (control points)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.axis("equal"); ax.grid(True, alpha=0.3); ax.legend(fontsize=8, loc="best")
    fig.tight_layout(); fig.savefig(output_dir / "trajectory.png", dpi=160); plt.close(fig)

    _save_series(result.times, result.formation_error, "Formation Error (unicycle)",
                 "formation error", None, output_dir / "formation_error.png")
    _save_series(result.times, result.localization_error, "Localization Error (unicycle)",
                 "||centroid - source||",
                 result.epsilon if result.bound_applicable else None,
                 output_dir / "localization_error.png")

    # Per-robot formation error e_i = ||z_i - z*|| on the control points (paper Fig. 3).
    z = result.control_points - base.radius * phi          # (steps+1, n, 2)
    z_star = z.mean(axis=1, keepdims=True)                  # (steps+1, 1, 2)
    per_robot = np.linalg.norm(z - z_star, axis=2)          # (steps+1, n)
    fig, ax = plt.subplots(figsize=(7, 4))
    for i in range(base.n):
        ax.plot(result.times[::stride], per_robot[::stride, i], linewidth=1.0, label=f"robot {i}")
    ax.set_title("Per-Robot Formation Error (unicycle, paper Fig. 3)")
    ax.set_xlabel("time [s]"); ax.set_ylabel(r"$\|z_i - z^*\|$")
    ax.grid(True, alpha=0.3); ax.legend(fontsize=8, ncol=2, loc="best")
    fig.tight_layout(); fig.savefig(output_dir / "formation_error_per_robot.png", dpi=160); plt.close(fig)

    (output_dir / "summary.json").write_text(json.dumps(result.summary(), indent=2), encoding="utf-8")


def _save_series(times, values, title, ylabel, bound, path) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 4))
    stride = max(1, len(times) // 5000)
    ax.plot(times[::stride], values[::stride], color="#1f77b4", linewidth=1.4)
    if bound is not None:
        ax.axhline(bound, color="red", linestyle="--", linewidth=1.0, label="theoretical bound")
        ax.legend(fontsize=8)
    ax.set_title(title); ax.set_xlabel("time [s]"); ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=160); plt.close(fig)


# ----------------------------------------------------------------------
# CLI integration (wired into sgf_sim.cli with two lines)
# ----------------------------------------------------------------------
def add_unicycle_parser(subcommands) -> None:
    """Register the ``unicycle`` subcommand on an argparse subparsers object."""

    from .topology import TOPOLOGY_NAMES

    p = subcommands.add_parser("unicycle", help="Phase 2 numerical unicycle model")
    p.add_argument("action", choices=["run", "validate", "compare-single-integrator"])
    p.add_argument("--duration", type=float, default=90.0)
    p.add_argument("--dt", type=float, default=0.004)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--noise", choices=["none", "gaussian", "bounded"], default="gaussian")
    p.add_argument("--alpha", type=float, default=10.0)
    p.add_argument("--beta", type=float, default=0.05)
    p.add_argument("--radius", type=float, default=2.0)
    p.add_argument("--dmax", type=float, default=12.0)
    p.add_argument("--noise-std", type=float, default=0.2)
    p.add_argument("--noise-bound", type=float, default=0.2)
    p.add_argument("--topology", choices=TOPOLOGY_NAMES, default="paper_fig1_reconstructed")
    p.add_argument("--offset", type=float, default=2.0, help="feedback-linearization shift r [m]")
    p.add_argument("--max-v", type=float, default=None, help="max linear velocity [m/s]; omit to disable")
    p.add_argument("--max-omega", type=float, default=None, help="max angular velocity [rad/s]; omit to disable")
    p.add_argument("--comms-period", type=float, default=0.0, help="sampled comms period T [s]; 0 = continuous")
    p.add_argument(
        "--sign-boundary-layer",
        type=float,
        default=0.0,
        help="boundary-layer width eps for sat(x/eps); 0 = exact paper sgn (default), 0.2 removes chattering",
    )
    p.add_argument("--run-id", default=None)
    p.add_argument("--output-dir", type=Path, default=Path("outputs") / "unicycle")


def unicycle_command(args) -> int:
    """Dispatch the ``unicycle`` subcommand. Returns a process exit code."""

    if args.action == "validate":
        args.noise = "bounded"  # theorem validation uses bounded noise
    config = unicycle_config_from_namespace(args)

    if args.action == "compare-single-integrator":
        comparison, uni, si = compare_single_integrator(config)
        run_id = args.run_id or f"compare_{config.base.topology}_{config.base.noise_model}_seed{config.base.seed}"
        output_dir = config.base.output_dir / run_id
        save_unicycle_run(uni, output_dir / "unicycle")
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "comparison.json").write_text(json.dumps(comparison, indent=2), encoding="utf-8")
        print(json.dumps(comparison["comparison"], indent=2))
        print(f"Wrote comparison artifacts to {output_dir}")
        return 0

    result = run_unicycle_simulation(config)
    run_id = args.run_id or f"{args.action}_{config.base.topology}_{config.base.noise_model}_seed{config.base.seed}"
    output_dir = config.base.output_dir / run_id
    save_unicycle_run(result, output_dir)
    summary = result.summary()
    print(json.dumps(summary, indent=2))
    print(f"Wrote artifacts to {output_dir}")
    if args.action == "validate" and summary["validation"]["inside_bound"] is False:
        return 3
    return 0
