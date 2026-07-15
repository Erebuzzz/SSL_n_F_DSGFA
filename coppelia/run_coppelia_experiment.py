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
import sys
from dataclasses import replace
from pathlib import Path

from .config import CoppeliaConfig
from .experiment import PartialRunError, run_experiment
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
    parser.add_argument("--no-telemetry", action="store_true",
                        help="skip the telemetry.npz/telemetry.csv export (on by default)")
    parser.add_argument("--floor-scale", type=float, default=7.0,
                        help="isometric scale for the CoppeliaSim floor (coppelia backend only)")
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
        floor_scale=args.floor_scale,
        save_animation=not args.no_animation,
        animation_format=args.animation_format,
        save_telemetry=not args.no_telemetry,
        output_dir=args.output_dir,
    )


def run_command(args: argparse.Namespace) -> int:
    config = config_from_args(args)
    run_id = args.run_id or _default_run_id(config)
    output_dir = config.output_dir / run_id
    return _run_and_save(
        config, output_dir, report=not args.no_report, title="Phase 3 CoppeliaSim Run"
    )


def run_config_command(args: argparse.Namespace) -> int:
    config = CoppeliaConfig.from_json(args.config_path)
    if args.backend is not None:
        config = replace(config, backend=args.backend)
    run_id = args.run_id or _default_run_id(config)
    output_dir = config.output_dir / run_id
    return _run_and_save(
        config, output_dir, report=True, title="Phase 3 CoppeliaSim Run (from config)"
    )


#: Exit code returned when a run was interrupted/crashed but partial artifacts were
#: still saved. Distinct from 0 (ok) and 3 (completed but outside the error bound).
EXIT_PARTIAL = 4


def _run_and_save(config: CoppeliaConfig, output_dir: Path, report: bool, title: str) -> int:
    """Run one experiment and persist artifacts, even if the run is interrupted.

    On a normal completion the full result is saved and the usual exit code is
    returned. If the control loop is interrupted or crashes after recording some
    data, the partial telemetry/plots are still written (so the run can be analysed)
    and :data:`EXIT_PARTIAL` is returned.
    """

    try:
        result = run_experiment(config)
    except PartialRunError as exc:
        _save_run(exc.result, output_dir, report=report, title=f"{title} (partial)")
        print(
            f"Run interrupted after {exc.recorded} step(s) "
            f"({exc.__cause__!r}); saved partial artifacts to {output_dir}",
            file=sys.stderr,
        )
        return EXIT_PARTIAL
    _save_run(result, output_dir, report=report, title=title)
    return _exit_code(result)


def _write_artifact(name: str, fn, output_dir: Path) -> None:
    """Run one artifact-writing step, reporting (not raising) on failure.

    Keeps a single failing artifact from aborting the rest of the save. A
    ``KeyboardInterrupt`` is deliberately NOT swallowed so the user can still stop
    the process -- by the time each step runs, everything written before it is
    already safely on disk.
    """

    try:
        fn()
    except Exception as exc:  # noqa: BLE001 - one artifact failing must not lose others
        print(f"warning: failed to write {name}: {exc!r}", file=sys.stderr)


def _save_run(result: CoppeliaResult, output_dir: Path, report: bool, title: str) -> None:
    """Persist run artifacts cheapest-and-most-valuable first.

    Order matters: the raw telemetry and the summary are the ground truth for
    debugging and are cheap to write, so they go FIRST -- before the plots and,
    crucially, before the slow multi-megabyte animation render. That way a second
    Ctrl-C, an out-of-memory, or an ffmpeg/Pillow failure during the animation can
    never cost you the telemetry (the exact failure mode that produced a run with
    plots+animation but no telemetry). Each artifact is written independently so one
    failing does not abort the rest.
    """

    output_dir.mkdir(parents=True, exist_ok=True)

    if result.config.save_telemetry:
        def _telemetry() -> None:
            from .telemetry import save_telemetry

            save_telemetry(result, output_dir)

        _write_artifact("telemetry", _telemetry, output_dir)

    def _summary() -> None:
        (output_dir / "summary.json").write_text(
            json.dumps(result.summary(), indent=2), encoding="utf-8"
        )

    _write_artifact("summary.json", _summary, output_dir)

    if result.config.save_plots:
        def _plots() -> None:
            from .plotting import save_plots

            save_plots(result, output_dir)

        _write_artifact("plots", _plots, output_dir)

    # Animation LAST: it is the slowest artifact and the one most likely to be
    # interrupted, so everything valuable is already on disk by the time it runs.
    animation_path = None
    if result.config.save_animation:
        from .plotting import save_animation

        try:
            animation_path = save_animation(result, output_dir)
        except Exception as exc:  # noqa: BLE001 - never let the GIF sink the run
            print(f"warning: animation export failed: {exc!r}", file=sys.stderr)

    if report:
        def _report() -> None:
            from .reporting import write_report

            write_report(result, output_dir, title, animation_path)

        _write_artifact("report", _report, output_dir)

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
