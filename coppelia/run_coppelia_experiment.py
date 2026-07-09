"""Command-line entry point for the Phase 3 CoppeliaSim build.

Examples
--------
Run offline against the mock kinematic backend (no CoppeliaSim required)::

    python -m coppelia run --backend mock --run-id mock_demo

Run against a live CoppeliaSim instance (ZMQ remote API on the default port)::

    python -m coppelia run --backend coppelia --run-id coppelia_demo

Run from a shared Phase 1.25 JSON config::

    python -m coppelia run-config configs/coppelia_default.json
"""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

from .config import CoppeliaConfig
from .experiment import run_experiment
from .metrics import CoppeliaResult
from .topology import TOPOLOGY_NAMES


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m coppelia",
        description="Phase 3 CoppeliaSim multi-robot build for the sign gradient-free algorithm.",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    run_parser = sub.add_parser("run", help="run one experiment")
    _add_common_arguments(run_parser)
    run_parser.add_argument("--run-id", default=None, help="output folder name")
    run_parser.add_argument("--no-report", action="store_true", help="skip the markdown report")

    cfg_parser = sub.add_parser("run-config", help="run from a shared JSON config file")
    cfg_parser.add_argument("config_path", type=Path)
    cfg_parser.add_argument("--backend", choices=["mock", "coppelia"], default=None)
    cfg_parser.add_argument("--run-id", default=None)

    return parser


def _add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--backend", choices=["mock", "coppelia"], default="mock")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--dt", type=float, default=0.004)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--noise", choices=["none", "gaussian", "bounded"], default="gaussian")
    parser.add_argument("--alpha", type=float, default=10.0)
    parser.add_argument("--beta", type=float, default=0.05)
    parser.add_argument("--radius", type=float, default=2.0)
    parser.add_argument("--dmax", type=float, default=12.0)
    parser.add_argument("--noise-std", type=float, default=0.2)
    parser.add_argument("--noise-bound", type=float, default=0.2)
    parser.add_argument("--topology", choices=TOPOLOGY_NAMES, default="paper_fig1_reconstructed")
    parser.add_argument("--offset", type=float, default=0.5, help="feedback-linearization shift r [m]")
    parser.add_argument("--sign-boundary-layer", type=float, default=0.0,
                        help="boundary-layer width eps for sat(x/eps); 0 = exact paper sgn (default)")
    parser.add_argument("--max-v", type=float, default=None, help="max linear velocity [m/s]; omit to disable")
    parser.add_argument("--max-omega", type=float, default=None, help="max angular velocity [rad/s]; omit to disable")
    parser.add_argument("--coppelia-host", default="localhost")
    parser.add_argument("--coppelia-port", type=int, default=23000)
    parser.add_argument("--no-animation", action="store_true", help="skip the animation export")
    parser.add_argument("--animation-format", choices=["gif", "mp4"], default="gif")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs") / "coppelia")


def config_from_args(args: argparse.Namespace) -> CoppeliaConfig:
    return CoppeliaConfig(
        backend=args.backend,
        duration=args.duration,
        dt=args.dt,
        seed=args.seed,
        noise_model=args.noise,
        alpha=args.alpha,
        beta=args.beta,
        radius=args.radius,
        dmax=args.dmax,
        noise_std=args.noise_std,
        noise_bound=args.noise_bound,
        topology=args.topology,
        control_point_offset=args.offset,
        sign_boundary_layer=args.sign_boundary_layer,
        max_linear_velocity=args.max_v,
        max_angular_velocity=args.max_omega,
        coppelia_host=args.coppelia_host,
        coppelia_port=args.coppelia_port,
        save_animation=not args.no_animation,
        animation_format=args.animation_format,
        output_dir=args.output_dir,
    )


def run_command(args: argparse.Namespace) -> int:
    config = config_from_args(args)
    result = run_experiment(config)
    run_id = args.run_id or _default_run_id(config)
    output_dir = config.output_dir / run_id
    _save_run(result, output_dir, report=not args.no_report, title="Phase 3 CoppeliaSim Run")
    return _exit_code(result)


def run_config_command(args: argparse.Namespace) -> int:
    config = CoppeliaConfig.from_json(args.config_path)
    if args.backend is not None:
        config = replace(config, backend=args.backend)
    result = run_experiment(config)
    run_id = args.run_id or _default_run_id(config)
    output_dir = config.output_dir / run_id
    _save_run(result, output_dir, report=True, title="Phase 3 CoppeliaSim Run (from config)")
    return _exit_code(result)


def _save_run(result: CoppeliaResult, output_dir: Path, report: bool, title: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    animation_path = None
    if result.config.save_plots:
        from .plotting import save_plots

        save_plots(result, output_dir)
    if result.config.save_animation:
        from .plotting import save_animation

        animation_path = save_animation(result, output_dir)
    (output_dir / "summary.json").write_text(
        json.dumps(result.summary(), indent=2), encoding="utf-8"
    )
    if report:
        from .reporting import write_report

        write_report(result, output_dir, title, animation_path)
    print(json.dumps(result.summary(), indent=2))
    print(f"Wrote artifacts to {output_dir}")


def _exit_code(result: CoppeliaResult) -> int:
    validation = result.summary()["validation"]
    if validation["inside_bound"] is False:
        return 3
    return 0


def _default_run_id(config: CoppeliaConfig) -> str:
    return f"{config.backend}_{config.topology}_{config.noise_model}_seed{config.seed}"


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "run":
        return run_command(args)
    if args.command == "run-config":
        return run_config_command(args)
    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
