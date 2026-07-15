# Shared Config Run: smoke_config_bounded

## Purpose

This report validates one numerical run against the paper's Section IV setup and Theorem 1 calculations.
The Gaussian run is for paper-style similarity. The bounded-noise run is for theorem-bound validation.

## Parameters

| Field | Value |
|---|---:|
| robots | 6 |
| topology | paper_fig1_reconstructed |
| source | [5.5, 5.5] |
| radius | 2.0 |
| Dmax | 12.0 |
| alpha | 100.0 |
| beta | 0.05 |
| alpha / beta | 2000.0 |
| dt | 0.0005 |
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
| inside bound | True |

## Error Metrics

| Metric | Value |
|---|---:|
| initial formation error | 9.909041096234613 |
| final formation error | 0.275381529151567 |
| initial localization error | 4.717726618239387 |
| final localization error | 0.02819648421304502 |
| tail formation error span | 0.18950966676165631 |
| tail localization error span | 0.03737937151409838 |

## Artifacts

- `trajectory.png`
- `formation_error.png`
- `localization_error.png`
- `summary.json`

## Interpretation

Small late-stage ripples are expected because the sign formation controller is discontinuous and the simulator uses fixed-step integration.
Use the smoothed plot line for visual comparison to the paper and the raw metrics for numerical review.
