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
| alpha | 10.0 |
| beta | 0.05 |
| alpha / beta | 200.0 |
| control-point offset r | 0.5 |
| max linear velocity | None |
| max angular velocity | None |
| dt | 0.004 |
| duration | 50.0 |
| noise model | bounded |
| seed | 1 |

## Topology

Edges use zero-based robot indices: `[(0, 1), (0, 2), (0, 5), (1, 2), (1, 4), (2, 3), (3, 4), (4, 5)]`.

## Theorem Checks

| Check | Value |
|---|---:|
| gain threshold | 1730.3999999999999 |
| gain condition passed | False |
| min informed robots | 6 |
| epsilon bound | 0.1 |
| bound applicable | True |
| inside bound | False |

## Error Metrics

| Metric | Value |
|---|---:|
| initial formation error | 9.909041096234613 |
| final formation error | 0.15226909004195277 |
| initial localization error | 4.444878451031529 |
| final localization error | 1.2221060905275596 |
| tail formation error span | 0.1651900509752473 |
| tail localization error span | 0.36532021193716546 |

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
