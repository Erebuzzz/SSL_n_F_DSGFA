"""Markdown report generation for validation runs."""

from __future__ import annotations

from pathlib import Path

from .simulation import SimulationResult
from .topology import edge_list


def write_validation_report(result: SimulationResult, output_dir: Path, title: str) -> Path:
    """Write a collaborator-readable Markdown report for one run."""

    output_dir.mkdir(parents=True, exist_ok=True)
    summary = result.summary()
    validation = summary["validation"]
    metrics = summary["metrics"]
    params = summary["parameters"]
    adjacency = result.config.resolved_adjacency()
    report = output_dir / "validation_report.md"
    report.write_text(
        "\n".join(
            [
                f"# {title}",
                "",
                "## Purpose",
                "",
                "This report validates one numerical run against the paper's Section IV setup and Theorem 1 calculations.",
                "The Gaussian run is for paper-style similarity. The bounded-noise run is for theorem-bound validation.",
                "",
                "## Parameters",
                "",
                "| Field | Value |",
                "|---|---:|",
                f"| robots | {params['n']} |",
                f"| topology | {result.config.topology} |",
                f"| source | {params['source']} |",
                f"| radius | {params['radius']} |",
                f"| Dmax | {params['dmax']} |",
                f"| alpha | {params['alpha']} |",
                f"| beta | {params['beta']} |",
                f"| alpha / beta | {validation['gain_ratio']} |",
                f"| dt | {params['dt']} |",
                f"| duration | {params['duration']} |",
                f"| noise model | {params['noise_model']} |",
                f"| seed | {params['seed']} |",
                "",
                "## Topology",
                "",
                f"Edges use zero-based robot indices: `{edge_list(adjacency)}`.",
                "",
                "## Theorem Checks",
                "",
                "| Check | Value |",
                "|---|---:|",
                f"| gain threshold | {validation['gain_threshold']} |",
                f"| gain condition passed | {validation['gain_condition_passed']} |",
                f"| min informed robots | {validation['min_n_informed']} |",
                f"| epsilon bound | {validation['epsilon']} |",
                f"| bound applicable | {validation['bound_applicable']} |",
                f"| inside bound | {validation['inside_bound']} |",
                "",
                "## Error Metrics",
                "",
                "| Metric | Value |",
                "|---|---:|",
                f"| initial formation error | {metrics['initial_formation_error']} |",
                f"| final formation error | {metrics['final_formation_error']} |",
                f"| initial localization error | {metrics['initial_localization_error']} |",
                f"| final localization error | {metrics['final_localization_error']} |",
                f"| tail formation error span | {metrics['tail_formation_error_span']} |",
                f"| tail localization error span | {metrics['tail_localization_error_span']} |",
                "",
                "## Artifacts",
                "",
                "- `trajectory.png`",
                "- `formation_error.png`",
                "- `localization_error.png`",
                "- `summary.json`",
                "",
                "## Interpretation",
                "",
                "Small late-stage ripples are expected because the sign formation controller is discontinuous and the simulator uses fixed-step integration.",
                "Use the smoothed plot line for visual comparison to the paper and the raw metrics for numerical review.",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return report
