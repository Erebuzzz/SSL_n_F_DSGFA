# Phase 3 CoppeliaSim Run

## Purpose

This report documents one Phase 3 CoppeliaSim-style run of the Du et al. (2024)
sign gradient-free source-localization and formation control law. Metrics are
computed on the feedback-linearization control points s_i, so they line up with
the Phase 1/2 numerical baselines.

Backend: **mock**

## Parameters

| Field | Value |
|---|---:|
| robots | 6 |
| topology | paper_fig1_reconstructed |
| source | [5.5, 5.5] |
| radius R | 2.0 |
| Dmax | 12.0 |
| alpha | 100.0 |
| beta | 0.05 |
| alpha / beta | 2000.0 |
| control-point offset r | 2.0 |
| max linear velocity | None |
| max angular velocity | None |
| dt | 0.004 |
| duration | 60.0 |
| noise model | bounded |
| seed | 1 |

## Topology

Edges use zero-based robot indices: `[(0, 1), (0, 2), (0, 5), (1, 2), (1, 4), (2, 3), (3, 4), (4, 5)]`.

## Theorem Checks

| Check | Value |
|---|---:|
| gain threshold | 1730.3999999999999 |
| gain condition passed | True |
| min informed robots | 6 |
| epsilon bound | 0.1 |
| bound applicable | True |
| inside bound | False |

## Error Metrics

| Metric | Value |
|---|---:|
| initial formation error | 9.909041096234613 |
| final formation error | 2.1145631971634984 |
| initial localization error | 3.906013881752655 |
| final localization error | 0.3774886714258889 |
| tail formation error span | 0.2515629242249542 |
| tail localization error span | 0.156875692048987 |

## Artifacts

- `trajectory.png`
- `formation_error.png`
- `localization_error.png`
- `summary.json`
- `animation.gif`

## Interpretation

The single-integrator control law is applied to unicycle robots via point-offset
feedback linearization (r > 0). On the mock kinematic backend the behavior should
closely track the numerical single-integrator baseline; on the CoppeliaSim backend,
physics (wheel slip, inertia, actuator limits) will introduce additional deviation
that this report is meant to quantify against the numerical run.
