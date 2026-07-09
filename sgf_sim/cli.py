"""Command line interface for simulations and validations."""

from __future__ import annotations

import argparse
import json
from dataclasses import replace
from pathlib import Path

from .config import SimulationConfig
from .experiments import run_experiment_suite
from .plotting import save_plots
from .reporting import write_validation_report
from .shared_config import load_shared_config
from .simulation import SimulationResult, run_simulation
from .topology import TOPOLOGY_NAMES


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m sgf_sim",
        description="Run the Du et al. sign gradient-free numerical simulator.",
    )
    subcommands = parser.add_subparsers(dest="command", required=True)

    run_parser = subcommands.add_parser("run", help="run one simulation")
    _add_common_arguments(run_parser)
    run_parser.add_argument("--run-id", default=None, help="output folder name")
    run_parser.add_argument("--report", action="store_true", help="write validation_report.md")

    validate_parser = subcommands.add_parser("validate", help="run theorem-oriented bounded-noise validation")
    _add_common_arguments(validate_parser)
    validate_parser.set_defaults(noise="bounded")
    validate_parser.add_argument("--report", action="store_true", default=True)

    paper_parser = subcommands.add_parser("validate-paper", help="run Gaussian and bounded paper-matching presets")
    _add_common_arguments(paper_parser)
    paper_parser.set_defaults(noise="gaussian", topology="paper_fig1_reconstructed")

    sweep_parser = subcommands.add_parser("sweep", help="run a small parameter sweep")
    _add_common_arguments(sweep_parser)
    sweep_parser.add_argument("--param", choices=["R", "delta", "beta", "topology"], default="R")
    sweep_parser.add_argument("--values", nargs="+", default=None)

    gain_parser = subcommands.add_parser("gain-sweep", help="run paper-style gain candidates with fixed alpha/beta ratio")
    _add_common_arguments(gain_parser)
    gain_parser.add_argument("--betas", nargs="+", type=float, default=[0.025, 0.0375, 0.05, 0.075])
    gain_parser.add_argument("--ratio", type=float, default=2000.0)

    experiment_parser = subcommands.add_parser("experiment", help="run a Phase 1.2 aggregate experiment suite")
    _add_common_arguments(experiment_parser)
    experiment_parser.add_argument("suite", choices=["paper-suite", "gain-ratio-sweep", "radius-delta-sweep", "seed-sweep"])
    experiment_parser.add_argument("--seeds", nargs="+", type=int, default=None)

    config_parser = subcommands.add_parser("run-config", help="run from a shared JSON config file")
    config_parser.add_argument("config", type=Path, help="path to shared JSON config")
    config_parser.add_argument("--run-id", default=None, help="override output folder name")
    config_parser.add_argument("--output-dir", type=Path, default=None, help="override output folder")

    from .unicycle import add_unicycle_parser

    add_unicycle_parser(subcommands)

    return parser


def _add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--dt", type=float, default=0.0005)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--noise", choices=["none", "gaussian", "bounded"], default="gaussian")
    parser.add_argument("--alpha", type=float, default=100.0)
    parser.add_argument("--beta", type=float, default=0.05)
    parser.add_argument("--radius", type=float, default=2.0)
    parser.add_argument("--dmax", type=float, default=12.0)
    parser.add_argument("--noise-std", type=float, default=0.2)
    parser.add_argument("--noise-bound", type=float, default=0.2)
    parser.add_argument("--topology", choices=TOPOLOGY_NAMES, default="paper_fig1_reconstructed")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs") / "runs")


def config_from_args(args: argparse.Namespace) -> SimulationConfig:
    return SimulationConfig(
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
        output_dir=args.output_dir,
    )


def run_command(args: argparse.Namespace) -> int:
    config = config_from_args(args)
    result = run_simulation(config)
    run_id = args.run_id or _default_run_id(config)
    output_dir = config.output_dir / run_id
    save_run(result, output_dir, report=args.report, title="Single Run Validation")
    _print_summary(result.summary(), output_dir)
    return 0


def validate_command(args: argparse.Namespace) -> int:
    config = config_from_args(args)
    result = run_simulation(config)
    output_dir = config.output_dir / _default_run_id(config, prefix="validate")
    save_run(result, output_dir, report=True, title="Bounded-Noise Theorem Validation")
    summary = result.summary()
    _print_summary(summary, output_dir)
    validation = summary["validation"]
    if not validation["gain_condition_passed"]:
        return 2
    if validation["inside_bound"] is False:
        return 3
    return 0


def validate_paper_command(args: argparse.Namespace) -> int:
    base = config_from_args(args)
    paper_dir = base.output_dir / "paper_validation"
    gaussian = replace(base, noise_model="gaussian")
    bounded = replace(base, noise_model="bounded")
    gaussian_result = run_simulation(gaussian)
    bounded_result = run_simulation(bounded)
    save_run(gaussian_result, paper_dir / "gaussian_similarity", report=True, title="Paper-Style Gaussian Similarity Run")
    save_run(bounded_result, paper_dir / "bounded_theorem_check", report=True, title="Bounded-Noise Theorem Check")
    combined = {
        "gaussian_similarity": gaussian_result.summary(),
        "bounded_theorem_check": bounded_result.summary(),
        "interpretation": {
            "gaussian": "Matches the paper noise model for visual similarity, but the theorem bound is not strictly applicable.",
            "bounded": "Matches the theorem assumption and should be used for inside-bound validation.",
        },
    }
    (paper_dir / "paper_validation_summary.json").write_text(json.dumps(combined, indent=2), encoding="utf-8")
    (paper_dir / "README.md").write_text(_paper_validation_readme(combined), encoding="utf-8")
    print(json.dumps(combined, indent=2))
    print(f"Wrote paper validation artifacts to {paper_dir}")
    return 0 if bounded_result.summary()["validation"]["inside_bound"] is not False else 3


def sweep_command(args: argparse.Namespace) -> int:
    base = config_from_args(args)
    values = args.values or _default_sweep_values(args.param)
    rows: list[dict[str, object]] = []
    sweep_dir = base.output_dir / f"sweep_{args.param.lower()}"
    sweep_dir.mkdir(parents=True, exist_ok=True)
    for raw_value in values:
        config = _config_for_sweep_value(base, args.param, raw_value)
        result = run_simulation(config)
        rows.append(_sweep_row(result, args.param, raw_value))
    _write_sweep_summary(rows, sweep_dir)
    return 0


def gain_sweep_command(args: argparse.Namespace) -> int:
    base = config_from_args(args)
    rows: list[dict[str, object]] = []
    sweep_dir = base.output_dir / "gain_sweep"
    sweep_dir.mkdir(parents=True, exist_ok=True)
    for beta in args.betas:
        config = replace(base, beta=beta, alpha=args.ratio * beta)
        result = run_simulation(config)
        rows.append(_sweep_row(result, "beta", beta))
    _write_sweep_summary(rows, sweep_dir)
    return 0


def experiment_command(args: argparse.Namespace) -> int:
    config = config_from_args(args)
    suite_dir = config.output_dir / "experiments"
    rows = run_experiment_suite(args.suite, config, suite_dir, seeds=args.seeds)
    print(json.dumps(rows, indent=2))
    print(f"Wrote experiment reports to {suite_dir / args.suite}")
    return 0


def run_config_command(args: argparse.Namespace) -> int:
    try:
        shared = load_shared_config(args.config)
    except NotImplementedError as exc:
        print(str(exc))
        return 4
    except ValueError as exc:
        print(f"Invalid config: {exc}")
        return 2
    config = shared.simulation
    if args.output_dir is not None:
        config = replace(config, output_dir=args.output_dir)
    result = run_simulation(config)
    run_id = args.run_id or shared.run_id
    output_dir = config.output_dir / run_id
    save_run(result, output_dir, report=shared.save_report, title=f"Shared Config Run: {run_id}")
    (output_dir / "resolved_config.json").write_text(json.dumps(shared.raw, indent=2), encoding="utf-8")
    _print_summary(result.summary(), output_dir)
    if shared.save_animation:
        print("Animation export is requested in config but is planned for a later phase.")
    return 0


def save_run(result: SimulationResult, output_dir: Path, report: bool = False, title: str = "Validation Report") -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    save_plots(result, output_dir)
    (output_dir / "summary.json").write_text(json.dumps(result.summary(), indent=2), encoding="utf-8")
    if report:
        write_validation_report(result, output_dir, title)


def _sweep_row(result: SimulationResult, param: str, value: object) -> dict[str, object]:
    summary = result.summary()
    metrics = summary["metrics"]
    validation = summary["validation"]
    return {
        "param": param,
        "value": value,
        "topology": result.config.topology,
        "alpha": result.config.alpha,
        "beta": result.config.beta,
        "gain_ratio": validation["gain_ratio"],
        "final_formation_error": metrics["final_formation_error"],
        "final_localization_error": metrics["final_localization_error"],
        "formation_entry_time": metrics["formation_entry_time"],
        "localization_entry_time": metrics["localization_entry_time"],
        "tail_formation_error_span": metrics["tail_formation_error_span"],
        "tail_localization_error_span": metrics["tail_localization_error_span"],
        "epsilon": validation["epsilon"],
        "inside_bound": validation["inside_bound"],
        "gain_condition_passed": validation["gain_condition_passed"],
    }


def _write_sweep_summary(rows: list[dict[str, object]], sweep_dir: Path) -> None:
    summary_path = sweep_dir / "summary.json"
    summary_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    report_lines = [
        "# Sweep Summary",
        "",
        "| value | final localization | final formation | loc entry | loc tail span | gain pass | inside bound |",
        "|---|---:|---:|---:|---:|---|---|",
    ]
    for row in rows:
        report_lines.append(
            f"| {row['value']} | {row['final_localization_error']} | {row['final_formation_error']} | {row['localization_entry_time']} | {row['tail_localization_error_span']} | {row['gain_condition_passed']} | {row['inside_bound']} |"
        )
    (sweep_dir / "summary.md").write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    print(json.dumps(rows, indent=2))
    print(f"Wrote sweep summary to {summary_path}")


def _print_summary(summary: dict[str, object], output_dir: Path) -> None:
    print(json.dumps(summary, indent=2))
    print(f"Wrote artifacts to {output_dir}")


def _default_run_id(config: SimulationConfig, prefix: str = "run") -> str:
    return f"{prefix}_{config.topology}_{config.noise_model}_seed{config.seed}_dt{config.dt:g}"


def _default_sweep_values(param: str) -> list[object]:
    if param == "R":
        return [1.0, 1.5, 2.0, 2.5, 3.0]
    if param == "delta":
        return [0.05, 0.1, 0.2, 0.4]
    if param == "beta":
        return [0.025, 0.05, 0.1]
    if param == "topology":
        return list(TOPOLOGY_NAMES)
    raise ValueError(param)


def _config_for_sweep_value(base: SimulationConfig, param: str, value: object) -> SimulationConfig:
    if param == "R":
        return replace(base, radius=float(value))
    if param == "delta":
        return replace(base, noise_bound=float(value))
    if param == "beta":
        return replace(base, beta=float(value))
    if param == "topology":
        return replace(base, topology=str(value))
    raise ValueError(param)


def _paper_validation_readme(combined: dict[str, object]) -> str:
    bounded = combined["bounded_theorem_check"]
    gaussian = combined["gaussian_similarity"]
    return "\n".join(
        [
            "# Paper Validation Outputs",
            "",
            "This folder contains the Phase 1.1 paper-matching validation preset.",
            "",
            "## Runs",
            "",
            "- `gaussian_similarity`: uses the paper's Gaussian noise model for visual comparison.",
            "- `bounded_theorem_check`: uses bounded noise so the theorem bound is applicable.",
            "",
            "## Key Results",
            "",
            f"- Gaussian final localization error: {gaussian['metrics']['final_localization_error']}",
            f"- Bounded final localization error: {bounded['metrics']['final_localization_error']}",
            f"- Bounded epsilon: {bounded['validation']['epsilon']}",
            f"- Bounded inside bound: {bounded['validation']['inside_bound']}",
            "",
        ]
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.command == "run":
        return run_command(args)
    if args.command == "validate":
        return validate_command(args)
    if args.command == "validate-paper":
        return validate_paper_command(args)
    if args.command == "sweep":
        return sweep_command(args)
    if args.command == "gain-sweep":
        return gain_sweep_command(args)
    if args.command == "experiment":
        return experiment_command(args)
    if args.command == "run-config":
        return run_config_command(args)
    if args.command == "unicycle":
        from .unicycle import unicycle_command

        return unicycle_command(args)
    parser.error(f"unknown command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
