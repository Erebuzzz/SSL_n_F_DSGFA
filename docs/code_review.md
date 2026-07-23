# Code Review

## Review Summary

This repository now contains a Phase 1 numerical implementation of the Du et al. sign gradient-free source-localization and formation algorithm. The implementation is intentionally small and auditable: theory formulas, control primitives, simulation integration, plotting, CLI behavior, and tests are separated into different modules.

The code is suitable for first-pass validation against the paper's calculations and qualitative simulation behavior. It is not yet a robotics or CoppeliaSim build.

## Theory-to-Code Mapping

| Paper concept | Code location | Review note |
|---|---|---|
| Single-integrator dynamics | `sgf_sim/simulation.py` | Euler step applies `p_next = p + dt * u`. |
| Circular slots `phi(theta_i)` | `sgf_sim/control.py` | Uses fixed equally spaced slots. |
| Shifted state `z_i = p_i - R phi_i` | `sgf_sim/control.py` | Formation control is computed on shifted states. |
| Component-wise sign communication | `sgf_sim/control.py` | Uses `np.sign` per coordinate, not vector normalization. |
| Measurement model | `sgf_sim/control.py` | Supports source-field measurement inside `Dmax` and saturation outside. |
| Gain condition | `sgf_sim/theory.py` | Implements `alpha / beta > 4 n f_Dmax / R`. |
| Error bound | `sgf_sim/theory.py` | Implements the theorem epsilon formula and Remark 4 simplification. |
| Validation output | `sgf_sim/simulation.py` | Summary includes gain pass, informed counts, epsilon, and final errors. |

## Important Review Points

1. The implementation correctly uses component-wise sign values in `{-1, 0, 1}`. This is a critical paper detail because replacing it with a normalized direction vector would change the algorithm.
2. The default gain ratio is theorem-compliant: `alpha / beta = 2000`, while the default threshold is approximately `1730.4`.
3. The code distinguishes Gaussian paper-like runs from bounded theorem-valid runs. This is necessary because Gaussian noise does not satisfy the theorem's hard bounded-noise assumption.
4. The default graph is a documented connected undirected graph. It is not claimed to be the exact paper Fig. 1 graph because the paper does not publish the edge list numerically.
5. The simulation uses fixed-step Euler integration. Because sign control is discontinuous, the default `dt = 0.0005` is intentionally small to reduce discrete-time chatter.

## Validation Evidence

Verified commands:

```powershell
python -m pytest -p no:hypothesispytest -q
```

Result:

```text
7 passed
```

The bounded validation command was also verified:

```powershell
python -m sgf_sim validate --duration 60 --dt 0.0005 --alpha 100 --beta 0.05 --seed 1
```

Observed summary:

```text
gain_condition_passed: true
epsilon: 0.1
inside_bound: true
final_localization_error: about 0.035
```

The Gaussian paper-like command was verified:

```powershell
python -m sgf_sim run --duration 60 --dt 0.0005 --alpha 100 --beta 0.05 --seed 1 --noise gaussian --run-id paper_like_gaussian_seed1
```

Observed behavior:

```text
gain_condition_passed: true
final_localization_error: about 0.037
bound_applicable: false
```

The bound is marked not applicable for Gaussian noise because the theorem assumes bounded perturbations.

## Known Limitations

- The paper does not provide exact values for `alpha`, `beta`, or a numeric adjacency matrix, so exact reproduction is not possible from the published text alone.
- Formation error is reduced strongly but may retain small discrete-time chatter because of the discontinuous sign controller.
- The current simulator does not include the Section V unicycle feedback-linearization layer.
- No CoppeliaSim assets are included yet. That should remain a later phase after the numerical model is accepted.
- The tests validate core formulas and run behavior, but they do not yet assert visual similarity to paper figures.

## Recommended Next Review Items

1. Reconstruct the Fig. 1 graph from the paper image and add it as an optional named topology.
2. Add convergence-time metrics for formation and localization.
3. Add a sweep report for gain ratio sensitivity around the theorem threshold.
4. Add a comparison notebook or script that places generated plots beside screenshots of Figs. 2-4 for human review.

## Jitter Review Note

The localization jitter seen in generated plots is primarily discrete-time sign-control chatter, not a failure of source localization. This was checked by comparing Gaussian, bounded, and no-noise runs. The no-noise run still has a late-stage ripple, which points to the sign term and Euler step as the main mechanism.

The plotting layer now renders raw data lightly and overlays a smoothed curve for paper-style visual comparison. This keeps the simulation honest while making the exported figures closer to the paper's visual style.

## Phase 1.1 Review Addendum

Phase 1.1 adds named topology support and a paper-validation preset. The default topology is now `paper_fig1_reconstructed`, which was inferred from the rendered Fig. 1 image because the paper does not provide a numeric adjacency matrix.

New commands reviewed:

```powershell
python -m sgf_sim validate-paper --seed 1
python -m sgf_sim sweep --param topology --noise bounded
python -m sgf_sim gain-sweep --ratio 2000 --noise gaussian
```

The full `validate-paper` run passed with the reconstructed topology. The bounded theorem check reported `inside_bound: true`, with final localization error about `0.0282` against epsilon `0.1`.

## Phase 1.2 Review Addendum

Phase 1.2 adds aggregate experiment reporting. Single-run summaries now include convergence metrics:

- `formation_entry_time`
- `localization_entry_time`
- `time_inside_localization_threshold_after_entry`

The new `experiment` command writes JSON, CSV, and Markdown reports for paper-suite, gain-ratio, radius-delta, and seed sweeps.

## Phase 1.25 Review Addendum

Phase 1.25 adds shared JSON config files and Python `run-config` support. The config format is intentionally MATLAB-compatible through `jsondecode` and documented in `CONFIG_GUIDE.md`.

Current Python support is limited to `single_integrator`. Planned modes such as `unicycle` and `turtlebot_simulink` raise a clear `NotImplementedError` until those phases are implemented.
## Phase 1.3 Review Addendum

Phase 1.3 adds a MATLAB parity layer in `matlab/`. The implementation reads the same shared JSON config files as Python through `jsondecode`, resolves the same topology and default initial positions, runs the Section IV single-integrator model, and exports comparable plots plus `summary.json` and `result.mat`.

Review notes:

- `matlab/run_from_config.m` is the main MATLAB entry point and matches the Phase 1.25 acceptance command.
- `matlab/sgf_control.m` keeps the component-wise sign controller aligned with the Python implementation.
- `matlab/sgf_measurement.m` preserves the Gaussian, bounded, and no-noise measurement modes.
- `matlab/sgf_theory_bounds.m` computes the same gain threshold and epsilon metadata as `sgf_sim/theory.py`.
- `matlab/sgf_topology.m` converts zero-based shared-config edge lists into MATLAB one-based adjacency matrices.

Verification status:

```powershell
python -m pytest -p no:hypothesispytest -q
```

A short MATLAB smoke run was also verified outside the sandbox after the sandboxed MATLAB startup failed with a filesystem inconsistency:

```powershell
matlab -batch "addpath('matlab'); run_from_config('tmp/matlab_smoke_config.json');"
```

Observed result:

```text
MATLAB parity run complete: matlab_smoke_matlab
Final localization error: 4.72403
Inside theorem bound: 0
```

The smoke config used `duration = 0.02`, `dt = 0.01`, and disabled plots and animation, so it validates startup, config parsing, simulation, and summary export rather than convergence quality. Noisy Python and MATLAB runs are expected to differ at the sample level because their random-number generators are different. Deterministic parity should be checked with `noise.model = "none"` for deeper numerical comparison.

## Standalone script consolidation (2026-07-20)

Three self-contained scripts were added under `sims/` to simplify distribution and review:

| Script | Source of truth inlined from | Default behavior |
|---|---|---|
| `sims/SingleIntegrator.m` | `matlab/` (`sgf_*`, `run_from_config`) | paper bounded validation (`alpha=100`, `dt=5e-4`, exact `sgn`) |
| `sims/Unicycle.m` | Python Phase 2 + TurtleBot numeric loop | working unicycle (`alpha=10`, `r=2`, `eps_bl=0.2`) |
| `sims/TurtleBot.m` | `matlab_turtlebot/` numeric + Simulink builder | numeric working preset; `runMode="simulink"` for `.slx` |

Run guide: [`sims/RUN_GUIDE.md`](../sims/RUN_GUIDE.md).


Structural refactor only: Eq. 4 control, measurement model, topology, theory bounds, and integrators match the modular packages. Alternative presets (Gaussian SI, pure `sgn`, hardware TurtleBot limits) are kept as commented blocks inside each script.

### Verification

- **SingleIntegrator.m**: final localization `≈ 0.030`, `inside_bound=1` (epsilon `0.1`), matching paper bounded-validation expectations.
- **Unicycle.m**: final formation `0.00523`, localization `0.00089`, `inside_bound=1` (matches `turtlebot_working` numeric reference).
- **TurtleBot.m** (numeric): identical metrics to Unicycle under the same defaults.
- **TurtleBot.m** (Simulink, 5 s smoke): builds `sgf_turtlebot_swarm.slx`, runs `ode4`, writes summary (full 90 s noise-free reference remains available via `runMode="simulink"`).
- **SingleIntegrator vs `matlab/`**: 2 s noise-free kernel comparison gave `max |pos| diff = 0` (bit-identical).

## Research gap validation harness (2026-07-21)

New files for three open extensions relative to Du et al. (2024):

| Artifact | Purpose |
|---|---|
| `docs/RESEARCH_GAPS_THEORY.md` | Mathematical framing, proof-gap analysis, literature anchors |
| `docs/research/research_gaps.tex` | LaTeX skeleton for formal step-by-step proofs |
| `docs/WORKFLOW_CONTEXT_RESEARCH_GAPS.md` | Agent handoff for continuing research |
| `sims/research.m` | Self-contained MATLAB validation for all three gaps |

### Gap summary

| Gap | Model change | What validation checks |
|---|---|---|
| 1 Multi-source | Unified cluster start, split at $t_{\mathrm{split}}$, balanced assignment; configs `n8_N2`, `n9_N3`, `n12_N3` | Per-team centroid error vs Du et al. $\varepsilon_k$; phase timeline and deployment map |
| 2 Moving source | `p_s(t)` linear drift; `single_integrator` or `unicycle` | Tracking error vs ISS predicted offset |
| 3 Global maximum | Multi-Gaussian peaks; flipped localization sign (ascent) | Local trap vs global peak |

### Run command

```matlab
cd sims
research                    % all gaps
research gap1                 % all Gap 1 configs
research gap1 n9_N3           % single config
research gap2 unicycle        % unicycle dynamics
```

Gap 1 outputs: `sims/outputs/research/gap1/<config>/` with `trajectory.png`, `team_localization.png`, `phase_timeline.png`, `deployment_map.png`, `motion.gif`, `summary.json`, `result.mat`.

Gap 2/3 outputs: `sims/outputs/research/gap2|gap3/` (standard plot set + `motion.gif`).

### Review notes

- Gap 1 decomposes the swarm into independent teams; cross-team edges are removed from the formation graph. This matches the draft decomposition proposition but is **outside** the original theorem until formally proved.
- Gap 2 uses the baseline Eq. 4 unchanged; tracking is empirical validation of the ISS sketch in the theory doc.
- Gap 3 intentionally starts near a suboptimal peak to document local trapping. Escape mechanisms (dither, multi-swarm reassignment) are listed as future work, not implemented yet.
- All three gaps violate at least one proof pillar (P4-P7); runs are labeled as extension experiments, not theorem checks for the original paper.

### Verification status

MATLAB run completed 2026-07-21 (`research gap1`, `gap2`, `gap3`):

| Gap | Key result | Interpretation |
|---|---|---|
| 1 | Team errors `[0.0042, 0.0072]` (legacy 6-robot), `[0.0195, 0.0249]` (`n8_N2` unified-split) | Per-team decomposition works; clustered start then split |
| 2 | Tail mean tracking error `0.381` vs ISS offset `0.461` | Centroid tracks linearly moving source with bounded lag |
| 3 | Trapped at peak 2, `f(p*)=2.08` vs global max `5.0` | Local trap confirmed; escape not yet implemented |

Static Code Analyzer: no errors (two info/warning items on `find` vs logical indexing and unused argument).

