"""Experiment-suite runners for Phase 1.2 reporting."""

from __future__ import annotations

import csv
import json
from dataclasses import replace
from pathlib import Path
from typing import Iterable

from .config import SimulationConfig
from .simulation import SimulationResult, run_simulation
from .topology import TOPOLOGY_NAMES


Row = dict[str, object]


def run_experiment_suite(name: str, base: SimulationConfig, output_dir: Path, seeds: list[int] | None = None) -> list[Row]:
    """Run a named experiment suite and write aggregate reports."""

    suite_dir = output_dir / name
    suite_dir.mkdir(parents=True, exist_ok=True)
    if name == "paper-suite":
        rows = _paper_suite(base)
    elif name == "gain-ratio-sweep":
        rows = _gain_ratio_sweep(base)
    elif name == "radius-delta-sweep":
        rows = _radius_delta_sweep(base)
    elif name == "seed-sweep":
        rows = _seed_sweep(base, seeds or [1, 2, 3, 4, 5])
    else:
        raise ValueError(f"unknown experiment suite: {name}")
    write_aggregate_reports(rows, suite_dir, title=f"Experiment Suite: {name}")
    return rows


def write_aggregate_reports(rows: list[Row], output_dir: Path, title: str) -> None:
    """Write JSON, CSV, and Markdown aggregate reports."""

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "summary.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    if rows:
        keys = _ordered_keys(rows)
        with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=keys)
            writer.writeheader()
            for row in rows:
                writer.writerow({key: row.get(key) for key in keys})
    (output_dir / "report.md").write_text(_markdown_report(rows, title), encoding="utf-8")


def row_from_result(result: SimulationResult, suite: str, variant: str) -> Row:
    summary = result.summary()
    params = summary["parameters"]
    validation = summary["validation"]
    metrics = summary["metrics"]
    return {
        "suite": suite,
        "variant": variant,
        "mode": "single_integrator",
        "seed": params["seed"],
        "noise_model": params["noise_model"],
        "topology": params["topology"],
        "radius": params["radius"],
        "noise_bound": params["noise_bound"],
        "alpha": params["alpha"],
        "beta": params["beta"],
        "gain_ratio": validation["gain_ratio"],
        "gain_threshold": validation["gain_threshold"],
        "gain_condition_passed": validation["gain_condition_passed"],
        "epsilon": validation["epsilon"],
        "bound_applicable": validation["bound_applicable"],
        "inside_bound": validation["inside_bound"],
        "final_formation_error": metrics["final_formation_error"],
        "final_localization_error": metrics["final_localization_error"],
        "formation_entry_time": metrics["formation_entry_time"],
        "localization_entry_time": metrics["localization_entry_time"],
        "time_inside_localization_threshold_after_entry": metrics["time_inside_localization_threshold_after_entry"],
        "tail_formation_error_span": metrics["tail_formation_error_span"],
        "tail_localization_error_span": metrics["tail_localization_error_span"],
    }


def _paper_suite(base: SimulationConfig) -> list[Row]:
    rows: list[Row] = []
    variants = [
        ("gaussian_similarity", replace(base, noise_model="gaussian")),
        ("bounded_theorem", replace(base, noise_model="bounded")),
    ]
    for variant, config in variants:
        rows.append(row_from_result(run_simulation(config), "paper-suite", variant))
    return rows


def _gain_ratio_sweep(base: SimulationConfig) -> list[Row]:
    rows: list[Row] = []
    ratios = [1000.0, 1500.0, 1730.4, 2000.0, 2500.0]
    beta = base.beta
    for ratio in ratios:
        config = replace(base, alpha=ratio * beta, beta=beta, noise_model="bounded")
        rows.append(row_from_result(run_simulation(config), "gain-ratio-sweep", f"ratio_{ratio:g}"))
    return rows


def _radius_delta_sweep(base: SimulationConfig) -> list[Row]:
    rows: list[Row] = []
    for radius in [1.0, 1.5, 2.0, 2.5, 3.0]:
        config = replace(base, radius=radius, noise_model="bounded")
        rows.append(row_from_result(run_simulation(config), "radius-delta-sweep", f"R_{radius:g}"))
    for delta in [0.05, 0.1, 0.2, 0.4]:
        config = replace(base, noise_bound=delta, noise_model="bounded")
        rows.append(row_from_result(run_simulation(config), "radius-delta-sweep", f"delta_{delta:g}"))
    return rows


def _seed_sweep(base: SimulationConfig, seeds: Iterable[int]) -> list[Row]:
    rows: list[Row] = []
    for seed in seeds:
        config = replace(base, seed=int(seed))
        rows.append(row_from_result(run_simulation(config), "seed-sweep", f"seed_{seed}"))
    return rows


def _ordered_keys(rows: list[Row]) -> list[str]:
    preferred = [
        "suite",
        "variant",
        "mode",
        "seed",
        "noise_model",
        "topology",
        "radius",
        "noise_bound",
        "alpha",
        "beta",
        "gain_ratio",
        "gain_threshold",
        "gain_condition_passed",
        "epsilon",
        "bound_applicable",
        "inside_bound",
        "final_formation_error",
        "final_localization_error",
        "formation_entry_time",
        "localization_entry_time",
        "time_inside_localization_threshold_after_entry",
        "tail_formation_error_span",
        "tail_localization_error_span",
    ]
    extras = sorted({key for row in rows for key in row if key not in preferred})
    return preferred + extras


def _markdown_report(rows: list[Row], title: str) -> str:
    lines = [
        f"# {title}",
        "",
        "## Summary",
        "",
        f"Total runs: {len(rows)}",
        "",
        "| variant | noise | topology | final localization | epsilon | inside bound | localization entry | tail loc span |",
        "|---|---|---|---:|---:|---|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {variant} | {noise_model} | {topology} | {final_localization_error} | {epsilon} | {inside_bound} | {localization_entry_time} | {tail_localization_error_span} |".format(
                **row
            )
        )
    lines.extend(
        [
            "",
            "## Interpretation Notes",
            "",
            "- Bounded-noise rows are theorem-valid when `bound_applicable` is true.",
            "- Gaussian rows are paper-similarity rows because Gaussian noise is not strictly bounded.",
            "- `localization_entry_time` is the first time the localization error enters the reported threshold.",
            "- `time_inside_localization_threshold_after_entry` measures how persistently the run stays within that threshold after first entry.",
            "",
        ]
    )
    return "\n".join(lines)


__all__ = ["row_from_result", "run_experiment_suite", "write_aggregate_reports"]
