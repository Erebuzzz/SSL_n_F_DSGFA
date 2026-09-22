# Standalone Simulation Run Guide

This folder contains three self-contained MATLAB scripts for the Du et al. (2024) sign gradient-free source localization and formation controller. Each script is a single file: no `addpath`, no JSON configs, no helper folders.

| Script | Robot model | Default case |
|---|---|---|
| `SingleIntegrator.m` | Point robots `p_dot = u` | Theorem-faithful bounded noise |
| `Unicycle.m` | Unicycle + feedback linearization | Working gains + boundary layer |
| `TurtleBot.m` | Differential drive (+ optional Simulink) | Numeric TurtleBot working preset |

Full parameter reference and mode switching are below.

---

## Quick start

From the repository root in MATLAB:

```matlab
cd sims
SingleIntegrator
Unicycle
TurtleBot
```

Or run by path:

```matlab
run('sims/SingleIntegrator.m')
run('sims/Unicycle.m')
run('sims/TurtleBot.m')
```

**Requirements:** MATLAB R2021b+ (jsonencode not needed here). `TurtleBot.m` with `runMode = "simulink"` also needs Simulink and Stateflow (MATLAB Function blocks).

---

## Output artifacts

Every run writes to `sims/outputs/<ScriptName>/`:

| File | Description |
|---|---|
| `trajectory.png` | Robot / control-point trails, formation circle, centroid, source |
| `formation_error.png` | Aggregate formation error vs time |
| `formation_error_per_robot.png` | Per-robot `||z_i - z*||` (paper Fig. 3 style) |
| `localization_error.png` | Centroid distance to source (epsilon bound line when applicable) |
| `motion.gif` | Synchronized animation (~120 frames) |
| `summary.json` | Parameters, theorem validation, final metrics |
| `telemetry.json` | Per-step time series (decimated if > `telemetryMaxSamples`) |
| `result.mat` | Full `result` and `cfg` structs (full resolution) |

TurtleBot Simulink runs also write `models/sgf_turtlebot_swarm.slx` under the same output folder.

Toggle plots, GIF, or telemetry in each script's `%% Configuration` section:

```matlab
savePlots = true;          % false skips all PNGs (summary.mat still saved)
saveAnimation = true;      % false skips motion.gif
saveTelemetry = true;      % false skips telemetry.json
telemetryMaxSamples = 10000;  % decimate JSON when longer; full data in result.mat
animationFps = 20;         % GIF frame rate
```

### telemetry.json

Every run writes a JSON time-series log when `saveTelemetry = true`. Structure:

| Section | Contents |
|---|---|
| `meta` | model, timestamp, `dt`, `telemetry_stride`, `n_samples` |
| `parameters`, `validation`, `metrics` | Same snapshot as `summary.json` |
| `times` | Time vector [s] |
| `centroid`, `formation_error`, `localization_error`, `n_informed` | Global per-step signals |
| `formation_error_per_robot` | `(n_robots x n_samples)` per-robot formation error |
| `positions` | `(n_samples x n_robots x 2)` body positions |
| `control_points`, `headings`, `commanded_v`, `commanded_omega` | Unicycle / TurtleBot only |

Long runs (e.g. SingleIntegrator at `dt=0.0005`) are **decimated** to at most `telemetryMaxSamples` points so the JSON stays manageable. The final timestep is always included. Full-resolution arrays remain in `result.mat`.

To export every step in JSON, raise the cap:

```matlab
telemetryMaxSamples = inf;   % or 120001 for a 60 s / dt=0.0005 run
```

Note: very large JSON files (100+ MB) may be slow to write and load.

---

## Where to edit each script

All scripts use the same section layout:

```text
%% Configuration     % outputs, runMode (TurtleBot only)
%% Parameters        % paper gains, noise, duration, robot-specific fields
%% System Matrices   % adjacency, formation slots phi
%% Initialization
%% Simulation        % (or Controller / Simulation for Unicycle/TurtleBot)
%% Visualization
%% Helper Functions  % do not edit unless extending behavior
```

Open the script, edit `%% Configuration` and `%% Parameters`, save, and re-run.

---

## Mode 1: SingleIntegrator.m

### What it simulates

- Dynamics: `p_dot_i = u_i` (explicit Euler).
- Control and measurements on **robot positions** `p_i`.
- Default: exact paper `sgn` (`signBoundaryLayer = 0`).

### Key parameters (`%% Parameters`)

| Parameter | Default | Effect |
|---|---|---|
| `alpha`, `beta` | 100, 0.05 | Formation vs source-seeking gains; ratio must exceed theorem threshold (~1730 for defaults) |
| `noiseModel` | `"bounded"` | `"none"`, `"gaussian"`, or `"bounded"` |
| `noiseBound` | 0.2 | Bounded noise amplitude; drives epsilon bound |
| `duration`, `dt` | 60, 0.0005 | Small `dt` reduces sign-control chatter |
| `signBoundaryLayer` | 0.0 | 0 = exact paper `sgn`; >0 smooths sign (rarely needed for SI) |
| `R`, `Dmax`, `source` | 2, 12, [5.5 5.5] | Formation radius, sensing range, source location |

### Common cases

**Theorem validation (default)**: already set:

```matlab
noiseModel = "bounded";
noiseBound = 0.2;
alpha = 100.0;
beta = 0.05;
signBoundaryLayer = 0.0;
```

Expected: `inside_bound = 1`, final localization ~0.03 vs epsilon = 0.1.

**Paper-like Gaussian noise**: uncomment in script or set:

```matlab
noiseModel = "gaussian";
noiseStd = 0.2;
noiseBound = 0.2;
```

Theorem bound is not strictly applicable (`bound_applicable = false`).

**Deterministic / parity check:**

```matlab
noiseModel = "none";
noiseStd = 0;
noiseBound = 0;
```

**Paper Fig. 3 timescale** (slow formation, visible localization transient):

```matlab
alpha = 1.0;
beta = 0.03;
duration = 80.0;
dt = 0.001;
```

Gain ratio is below the conservative sufficient bound but reproduces paper-like dynamics.

**Faster smoke test:**

```matlab
duration = 5.0;
dt = 0.001;
saveAnimation = false;
```

---

## Mode 2: Unicycle.m

### What it simulates

- Control points: `s_i = p_i + r [cos theta_i; sin theta_i]`.
- Eq. 4 applied on **control points**, then feedback linearization to `(v, omega)`.
- Unicycle kinematics (explicit Euler).
- Metrics on control points (not raw body positions).

### Key parameters (`%% Parameters`)

| Parameter | Default | Effect |
|---|---|---|
| `alpha`, `beta` | 10, 0.05 | Lower `alpha` than SI avoids huge angular rates |
| `offset` | 2.0 | Feedback-linearization shift `r`; must be > 0 |
| `signBoundaryLayer` | 0.2 | **Critical** for localization on unicycle |
| `commandPeriod` | 0.0 | 0 = continuous; >0 holds commands between updates |
| `maxLinearVelocity`, `maxAngularVelocity` | `[]` | `[]` = no limit; set scalars to saturate |
| `initialHeadings` | zeros | Starting headings (radians) |

### Common cases

**Working demo (default)**: localizes inside epsilon:

```matlab
alpha = 10.0;
offset = 2.0;
signBoundaryLayer = 0.2;
noiseModel = "bounded";
```

Expected: final formation ~0.005, localization ~0.001, `inside_bound = 1`.

**Pure paper `sgn` (chattering failure mode):**

```matlab
signBoundaryLayer = 0.0;
```

Formation converges; localization stalls ~0.7–0.8 m from source (outside bound).

**Sampled communication:**

```matlab
commandPeriod = 0.1;   % hold (v, omega) for 0.1 s between recomputes
```

**Hardware-style limits** (TurtleBot3 Burger caps; localization often poor):

```matlab
alpha = 100.0;
offset = 0.025;
maxLinearVelocity = 0.22;
maxAngularVelocity = 2.84;
signBoundaryLayer = 0.0;
duration = 60.0;
dt = 0.001;
```

**Noise-free reference:**

```matlab
noiseModel = "none";
noiseStd = 0;
noiseBound = 0;
```

---

## Mode 3: TurtleBot.m

### What it simulates

Same control law as Unicycle, plus:

- Wheel-speed reporting via TurtleBot3 Burger parameters.
- **Numeric path:** explicit Euler (default).
- **Simulink path:** programmatic `.slx`, fixed-step `ode4`, continuous ODE.

### Switching execution mode (`%% Configuration`)

```matlab
runMode = "numeric";    % explicit Euler, bounded noise by default
runMode = "simulink";   % builds + sim() sgf_turtlebot_swarm.slx
```

| `runMode` | Integrator | Default noise | Notes |
|---|---|---|---|
| `"numeric"` | Euler | bounded | Matches `turtlebot_working.json` |
| `"simulink"` | ode4 (RK4) | none (forced) | Deterministic reference; needs Simulink |

When `runMode = "simulink"`, the script sets `noiseModel = "none"` automatically (stochastic measurement in a continuous ODE is ill-posed).

Simulink-only options (`%% Parameters`):

```matlab
simulinkSolver = "ode4";
simulinkModelName = "sgf_turtlebot_swarm";
```

### Key parameters (shared with Unicycle)

Same as Unicycle for `alpha`, `beta`, `offset`, `signBoundaryLayer`, `commandPeriod`, actuator limits, plus:

| Parameter | Default | Effect |
|---|---|---|
| `wheelRadius` | 0.033 | TurtleBot3 Burger wheel radius [m] |
| `wheelSeparation` | 0.16 | Axle width [m] |

`commandPeriod` affects **numeric only**; Simulink is always continuous.

### Common cases

**Numeric working demo (default):**

```matlab
runMode = "numeric";
alpha = 10.0;
offset = 2.0;
signBoundaryLayer = 0.2;
noiseModel = "bounded";
```

**Simulink noise-free reference:**

```matlab
runMode = "simulink";
signBoundaryLayer = 0.2;
duration = 90.0;
dt = 0.004;
```

`inside_bound` may read 0 because epsilon = 0 when `noiseBound = 0`; final localization should still be ~0.001 (at the source numerically).

**Hardware-faithful numeric run:**

```matlab
runMode = "numeric";
maxLinearVelocity = 0.22;
maxAngularVelocity = 2.84;
commandPeriod = 0.1;
```

**Compare numeric vs Simulink** (both noise-free, same gains):

1. Run with `runMode = "simulink"`, note final errors in `summary.json`.
2. Set `runMode = "numeric"`, `noiseModel = "none"`, re-run.
3. Compare `final_formation_error` and `final_localization_error` (should agree to ~1e-6 with boundary layer).

---

## Cross-cutting options (all scripts)

### Sign boundary layer

The formation term uses component-wise `sgn`. On unicycle/TurtleBot, exact `sgn` causes heading chatter and corrupts localization.

| Value | Behavior |
|---|---|
| `0` | Exact paper controller; SI works; unicycle/TurtleBot localization fails |
| `0.2` | Recommended for unicycle/TurtleBot; smooth near equilibrium |

Set in `%% Parameters`:

```matlab
signBoundaryLayer = 0.2;   % Unicycle / TurtleBot
signBoundaryLayer = 0.0;   % SingleIntegrator (default)
```

### Noise models

| `noiseModel` | Measurement noise | Theorem bound applicable? |
|---|---|---|
| `"none"` | 0 | Yes (epsilon may be 0 if bound is 0) |
| `"bounded"` | Uniform in `[-noiseBound, noiseBound]` | Yes |
| `"gaussian"` | `N(0, noiseStd^2)` | No (not strictly bounded) |

### Topology

All scripts embed the **paper Fig. 1 reconstructed** graph (6 nodes) in `%% System Matrices`. The edge list is fixed for `n = 6`. Changing `n` requires editing both `n` and the `edges` matrix plus `initialPositions`.

### Initial conditions

- **SingleIntegrator:** layout from `matlab/sgf_config.m` (shifted relative to `source`).
- **Unicycle / TurtleBot:** layout from `matlab_turtlebot/tb_config.m`.

Both shift by `source - [5.5, 5.5]` so the source at `[5.5, 5.5]` reproduces the paper geometry.

Custom starts: replace `initialPositions` (and `initialHeadings` for unicycle/TurtleBot).

### Random seed

```matlab
seed = 1;   % MATLAB twister; affects noisy runs only
```

Deterministic runs: set `noiseModel = "none"`.

---

## Interpreting summary.json

Key fields after each run:

```json
"validation": {
  "gain_ratio": 200,
  "gain_threshold": 1730.4,
  "gain_condition_passed": false,
  "epsilon": 0.1,
  "bound_applicable": true,
  "inside_bound": true
},
"metrics": {
  "final_formation_error": 0.005,
  "final_localization_error": 0.001
}
```

- **`inside_bound`:** final localization error vs theorem epsilon (when applicable).
- **`gain_condition_passed`:** sufficient gain condition from the paper; unicycle working preset intentionally uses a lower ratio that still works empirically.
- **Unicycle/TurtleBot** summaries also report `max_commanded_linear_velocity` and `max_commanded_angular_velocity`.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Localization stays large on Unicycle/TurtleBot | Pure `sgn` chattering | Set `signBoundaryLayer = 0.2` |
| `inside_bound = 0` on Simulink noise-free | Epsilon = 0 when `noiseBound = 0` | Expected; check absolute localization error |
| Simulink build fails | Missing Simulink/Stateflow | Use `runMode = "numeric"` or install toolboxes |
| GIF export very slow | Fine `dt` + long `duration` | Set `saveAnimation = false` or shorten run |
| Warning: 0 informed robots | Source too far / `Dmax` too small | Move `source`, raise `Dmax`, or fix initial positions |
| Huge `max |omega|` | `alpha` too large or `offset` too small | Use working preset: `alpha=10`, `offset=2` |

---

## Relationship to other project entry points

These scripts consolidate logic from:

- `matlab/`: single-integrator JSON/config workflow
- `matlab_turtlebot/`: TurtleBot numeric + Simulink JSON workflow
- Python `sgf_sim unicycle`: CLI unicycle (Phase 2)

The modular packages remain for batch configs, parity checks, and CoppeliaSim integration. For distribution and quick experiments, use this `sims/` folder.

See also: [docs/RUNNING_MODES.md](../docs/RUNNING_MODES.md), [README.md](../README.md).
