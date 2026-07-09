# MATLAB Parity Layer

This folder contains the Phase 1.3 MATLAB implementation path for the Du et al. sign gradient-free source-localization and formation simulator.

The MATLAB layer is intentionally parallel to the Python simulator. It uses the same shared JSON config files, the same single-integrator dynamics, the same Eq. 4 sign gradient-free controller, and the same theorem metadata. It does not replace the Python implementation.

## Requirements

- MATLAB R2021b or newer is recommended.
- No toolbox-specific functions are required for the core simulation.
- The scripts use `jsondecode` and `jsonencode`, which are available in modern MATLAB releases.

## Commands

From the repository root:

```matlab
addpath('matlab')
run_from_config('configs/paper_default.json')
run_from_config('configs/paper_bounded_validation.json')
```

Convenience scripts:

```matlab
addpath('matlab')
run_single_simulation
run_paper_validation
```

## Outputs

Each run writes to the configured output folder under a MATLAB-specific run directory:

- `trajectory.png`
- `formation_error.png`
- `localization_error.png`
- `motion.gif` when animation is enabled
- `summary.json`
- `result.mat`

The summary contains the same high-level sections as Python:

- `parameters`
- `validation`
- `metrics`

## Parity Notes

Python and MATLAB can use the same config file, but exact sample-by-sample equality is not expected for noisy runs because NumPy (PCG64) and MATLAB (Mersenne Twister) use different random-number generators from the same seed. Use `noise.model = "none"` for deterministic controller and integrator comparisons.

The topology edge list in the shared JSON config is zero-based to match Python. MATLAB converts those edge indices to one-based indices internally.

## Verifying parity against Python

A deterministic (noise-free) parity harness lets you confirm the MATLAB and
Python simulators integrate the identical trajectory.

1. Generate the golden reference from the runnable Python side (repo root):

   ```powershell
   python matlab/parity/generate_golden.py
   ```

   This runs `sgf_sim` on `matlab/parity/parity_none.json` and writes
   `matlab/parity/golden_parity_none.json`. It only imports Phase 1; it does not
   modify it.

2. Run the MATLAB check:

   ```matlab
   addpath('matlab')
   verify_parity
   ```

   `verify_parity.m` runs the MATLAB simulator on the same config and compares
   the full formation- and localization-error time-series, the final robot
   positions, the final centroid, and the key summary metrics against the golden
   reference, printing the maximum deviation per quantity and `PASS`/`FAIL`
   (tolerance `1e-8`).

### Golden reference values (config `parity_none.json`, seed 1, `dt=0.01`, 5 s, noise `none`)

Produced by `python matlab/parity/generate_golden.py`; the committed
`golden_parity_none.json` is the source of truth and both simulators must match it:

| Quantity | Value |
|---|---|
| steps | 501 |
| final formation error | 5.26848914008 |
| final localization error | 3.35300448674 |
| gain ratio `alpha/beta` | 2000 |
| epsilon bound | 0.1 |
| inside bound | false (5 s is too short to localize; parity checks the *trajectory*, not convergence) |
| final centroid | `[3.409797, 2.878228]` |

Regenerate the golden file whenever the shared control math changes on either
side so the two implementations stay locked together.

## File Map

```text
run_from_config.m         Shared JSON config entry point (+ core sim loop)
run_single_simulation.m   Convenience single paper-like run
run_paper_validation.m    Gaussian and bounded validation preset
verify_parity.m           Deterministic Python<->MATLAB parity check
sgf_config.m              Config parsing and defaults
sgf_control.m             Eq. 4 controller
sgf_measurement.m         Source-field measurement model
sgf_theory_bounds.m       Gain threshold and epsilon bound
sgf_topology.m            Named graph and edge-list topology support
sgf_plot_results.m        Plot and synchronized-animation export
parity/parity_none.json   Deterministic shared config for the parity check
parity/generate_golden.py Python golden-reference generator
parity/golden_parity_none.json  Committed Python reference (regenerate on math changes)
```
