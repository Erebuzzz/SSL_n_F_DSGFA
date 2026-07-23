# Workflow Context — Standalone MATLAB Consolidation

Last updated: 2026-07-20

## Goal completed

Refactor the MATLAB/Simulink stack into three self-contained scripts under `sims/`:

- `sims/SingleIntegrator.m`
- `sims/Unicycle.m`
- `sims/TurtleBot.m`

No external helpers, JSON configs, or `addpath` required at runtime. Modular packages (`matlab/`, `matlab_turtlebot/`) were left in place for JSON/parity workflows.

**Run guide:** [`sims/RUN_GUIDE.md`](../sims/RUN_GUIDE.md)

## Architecture snapshot (pre-refactor)

| Mode | Location | Notes |
|---|---|---|
| Single integrator | `matlab/` | JSON via `sgf_config`, helpers `sgf_*` |
| Unicycle | Python `sgf_sim/unicycle.py` only | MATLAB had no dedicated unicycle script; TurtleBot numeric was the closest |
| TurtleBot | `matlab_turtlebot/` | Reuses `matlab/` helpers; numeric Euler + programmatic Simulink |

## What each standalone script contains

Shared sections (`%%`): Configuration, Parameters, System Matrices, Initialization, Simulation, Visualization, Helper Functions.

- **SingleIntegrator**: Euler on `p`, metrics on positions, exact `sgn` by default, SI initial layout from `sgf_config`.
- **Unicycle**: control points `s = p + r[cos;sin]`, FL → `(v,ω)`, Euler kinematics, metrics on `s`, boundary layer `0.2`, TurtleBot-style initial layout from `tb_config`.
- **TurtleBot**: same as Unicycle + wheel-speed reporting; `runMode` switches numeric vs Simulink (`ode4` + baked `swarm_ode` MATLAB Function). Simulink APIs use char vectors (required by `add_block` / `strjoin`).

## Verified results

| Script | Result |
|---|---|
| Unicycle | form 0.00523, loc 0.00089, inside bound |
| TurtleBot numeric | same as Unicycle |
| TurtleBot Simulink | smoke OK (5 s); full 90 s when `runMode="simulink"` |
| SI vs modular | bit-identical 2 s noise-free positions |

## Output artifacts (all three scripts)

Each run writes under `sims/outputs/<Name>/`:

- `trajectory.png`
- `formation_error.png`
- `formation_error_per_robot.png` (paper Fig. 3 style)
- `localization_error.png`
- `motion.gif` (enabled by default via `saveAnimation = true`)
- `telemetry.json` (per-step time series; enabled via `saveTelemetry = true`)
- `summary.json`, `result.mat`

Unicycle/TurtleBot per-robot formation error is computed on control points `s_i`.


1. Optional: run full `SingleIntegrator` (60 s, `dt=5e-4`) and confirm `inside_bound` vs existing `outputs/runs/config_paper_bounded_validation_*`.
2. Optional: set `TurtleBot` `runMode="simulink"` for a full 90 s noise-free reference and compare to `outputs/turtlebot/turtlebot_simulink_default_turtlebot/`.
3. Docs updated: `README.md`, `docs/RUNNING_MODES.md`, `docs/code_review.md`, `sims/RUN_GUIDE.md`.
4. Do not delete `matlab/` or `matlab_turtlebot/` unless the user asks; they still serve config-driven and parity workflows.
5. If consolidating further, consider pointing `docs/ROADMAP.md` at the standalone scripts as the preferred MATLAB entry points.

## Pitfalls fixed during this work

- `strjoin` on a cell of string scalars fails → use char vectors in `generateOdeCode`.
- `add_block` destination must be char → avoid `[modelName "/block"]` string concat; use `'/'`.
- Initial positions differ between SI (`sgf_config`) and TurtleBot/Unicycle (`tb_config`); preserve both.
- Unicycle/TurtleBot need `sign_boundary_layer > 0` for localization inside epsilon.
