# Shared Config Run: paper_fig2_reproduction

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
| alpha | 5.0 |
| beta | 0.07 |
| alpha / beta | 71.42857142857142 |
| dt | 0.0005 |
| duration | 60.0 |
| noise model | gaussian |
| seed | 1 |

## Topology

Edges use zero-based robot indices: `[(0, 1), (0, 2), (0, 5), (1, 2), (1, 4), (2, 3), (3, 4), (4, 5)]`.

## Theorem Checks

| Check | Value |
|---|---:|
| gain threshold | 1730.3999999999999 |
| gain condition passed | False |
| min informed robots | 4 |
| epsilon bound | 0.18909509650773187 |
| bound applicable | False |
| inside bound | None |

## Error Metrics

| Metric | Value |
|---|---:|
| initial formation error | 16.9877786077924 |
| final formation error | 0.015639757950715667 |
| initial localization error | 10.600052410771898 |
| final localization error | 0.004053832401443856 |
| tail formation error span | 0.01260832678447664 |
| tail localization error span | 0.026024758501913204 |

## Artifacts

- `trajectory.png`
- `formation_error.png`
- `localization_error.png`
- `summary.json`

## Interpretation

Small late-stage ripples are expected because the sign formation controller is discontinuous and the simulator uses fixed-step integration.
Use the smoothed plot line for visual comparison to the paper and the raw metrics for numerical review.
