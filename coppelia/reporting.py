"""Markdown validation report for a Phase 3 run."""

from __future__ import annotations

from pathlib import Path

from .metrics import CoppeliaResult
from .topology import edge_list


def write_report(result: CoppeliaResult, output_dir: Path, title: str, animation_path: Path | None) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = result.summary()
    validation = summary["validation"]
    metrics = summary["metrics"]
    params = summary["parameters"]
    adjacency = result.config.resolved_adjacency()
    anim_line = f"- `{animation_path.name}`" if animation_path is not None else "- (animation not generated)"

    report = output_dir / "validation_report.md"
    report.write_text(
        "\n".join(
            [
                f"# {title}",
                "",
                "## Purpose",
                "",
                "This report documents one Phase 3 CoppeliaSim-style run of the Du et al. (2024)",
                "sign gradient-free source-localization and formation control law. Metrics are",
                "computed on the feedback-linearization control points s_i, so they line up with",
                "the Phase 1/2 numerical baselines.",
                "",
                f"Backend: **{summary['backend']}**",
                "",
                "## Parameters",
                "",
                "| Field | Value |",
                "|---|---:|",
                f"| robots | {params['n']} |",
                f"| topology | {params['topology']} |",
                f"| source | {params['source']} |",
                f"| radius R | {params['radius']} |",
                f"| Dmax | {params['dmax']} |",
                f"| alpha | {params['alpha']} |",
                f"| beta | {params['beta']} |",
                f"| alpha / beta | {validation['gain_ratio']} |",
                f"| control-point offset r | {params['control_point_offset']} |",
                f"| max linear velocity | {params['max_linear_velocity']} |",
                f"| max angular velocity | {params['max_angular_velocity']} |",
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
                anim_line,
                "",
                "## Interpretation",
                "",
                "The single-integrator control law is applied to unicycle robots via point-offset",
                "feedback linearization (r > 0). On the mock kinematic backend the behavior should",
                "closely track the numerical single-integrator baseline; on the CoppeliaSim backend,",
                "physics (wheel slip, inertia, actuator limits) will introduce additional deviation",
                "that this report is meant to quantify against the numerical run.",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return report
