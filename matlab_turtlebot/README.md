# Phase 2.5: MATLAB / Simulink TurtleBot Build

A TurtleBot-oriented differential-drive implementation of the paper's sign
gradient-free source-localization-and-formation controller (Du et al., 2024,
Eq. 4). It bridges the Phase 2 numerical unicycle model to a block-diagram
Simulink workflow, using the **same shared JSON config** as the Python and MATLAB
numerical runs and **reusing the Phase 1.3 controller math** (`../matlab/`) so the
control law is single-sourced across every implementation path.

Two execution paths are provided:

| Path | Entry point | Integrator | Config mode |
|------|-------------|-----------|-------------|
| **Numeric** (explicit Euler, sampled comms, actuator saturation) | `run_turtlebot_simulation.m` | hand-rolled Euler | `turtlebot_numeric` |
| **Simulink** (block diagram, fixed-step solver) | `run_turtlebot_simulink.m` | `ode4` (RK4) | `turtlebot_simulink` |

Both produce the identical result/summary struct and share the plotting,
animation, and metrics code.

## Layout

```
matlab_turtlebot/
  README.md
  run_turtlebot_from_config.m      dispatch on cfg.mode -> numeric | simulink, then plots/animation/summary
  run_turtlebot_simulation.m       numeric differential-drive swarm sim (Euler, sampled comms, saturation)
  run_turtlebot_simulink.m         build + sim() the Simulink model, reconstruct the result
  build_turtlebot_simulink_model.m programmatically build models/sgf_turtlebot_swarm.slx
  export_turtlebot_animation.m     synchronized 4-panel motion GIF
  tb_config.m                      parse shared JSON (accepts turtlebot_* / unicycle modes)
  models/
    sgf_turtlebot_swarm.slx        canonical generated model (rebuilt per run into the output dir)
  helpers/
    turtlebot_params.m                  TurtleBot3 Burger hardware constants
    unicycle_feedback_linearization.m   control-point map inverse -> (v, omega)
    differential_drive_step.m           explicit-Euler unicycle kinematics + wheel speeds
    turtlebot_result_from_trajectory.m  shared metrics/summary builder (used by the Simulink path)
  configs/
    turtlebot_working.json          numeric preset (noise=bounded, sane gains)
    turtlebot_simulink_default.json Simulink preset (noise=none deterministic reference)
```

## Running

From the repository root, in MATLAB (the MATLAB MCP session works too):

```matlab
addpath('matlab_turtlebot')                                    % run_turtlebot_from_config adds the rest

% Numeric differential-drive TurtleBot swarm
run_turtlebot_from_config('matlab_turtlebot/configs/turtlebot_working.json')

% Simulink block-diagram model (builds the .slx, then sim())
run_turtlebot_from_config('matlab_turtlebot/configs/turtlebot_simulink_default.json')
```

Each run writes, under `outputs/turtlebot/<run_id>_turtlebot/`:

- `trajectory.png`: robot trails, final circular formation target, centroid path, source
- `formation_error.png`, `localization_error.png`: error series (with the epsilon bound)
- `motion.gif`: synchronized 4-panel animation (formation scene + live formation and
  localization error curves + time cursor + gain-ratio/topology metadata)
- `result.mat`: full `result` and `cfg` structs
- `summary.json`: parameters, theorem validation, and metrics (same schema as Python)
- `models/sgf_turtlebot_swarm.slx`: the generated Simulink model (Simulink path only)

## The Simulink model

`build_turtlebot_simulink_model.m` constructs the model programmatically as a
genuine block diagram:

```
   ,--------------------------------------------------------------.
   v                                                              |
[ MATLAB Function: swarm_ode ] --dx--> [ Integrator (3n states) ]-'--> [ To Workspace: xout ]
                                                                       [ Clock ] -> [ To Workspace: tout ]
```

- **State vector** `x = [x_1..x_n, y_1..y_n, theta_1..theta_n]` (3n = 18 for n = 6),
  held by a single vector `Integrator` whose initial condition is the flattened
  start pose.
- **`swarm_ode` MATLAB Function block** computes the state derivative and embodies
  the full controller with all parameters baked in as numeric literals (so the
  `.slx` is self-contained and codegen-safe for the chosen config):
  1. control points `s_i = p_i + r[cos theta_i, sin theta_i]`;
  2. saturated quadratic measurement `sigma_i` (identical to `sgf_measurement`, noise-free);
  3. Eq. 4 sign controller `f_i = alpha * sum_j sgn(z_j - z_i) - (2 beta / R) sigma_i phi_i`
     with **component-wise** `sgn` and `z_i = s_i - R phi_i`;
  4. feedback linearization `v_i = f_i . [cos, sin]`, `omega_i = f_i . [-sin, cos]/r`;
  5. differential-drive derivative `dx = v cos theta`, `dy = v sin theta`, `dtheta = omega`.
- **Solver:** fixed-step `ode4` (RK4), `FixedStep = dt`, `StopTime = duration`.
- **Logging:** `To Workspace` blocks capture the state (`xout`, stacked as
  `[3n, 1, T]`) and time (`tout`).

The model is **continuous** (`command_period = 0`) and **deterministic**
(`noise = none`). A stochastic measurement in a continuous ODE is ill-posed, so
the Simulink path is the noise-free reference; the numeric path carries the
sampled-comms and bounded-noise behaviour.

## Reused Phase 1.3 math

`run_turtlebot_from_config` adds `../matlab/` to the path and reuses
`sgf_control`, `sgf_measurement`, `sgf_topology`, and `sgf_theory_bounds`
unchanged. The Simulink `swarm_ode` block reproduces the *same* arithmetic
inline (it cannot call those functions from generated code), and the shared
`turtlebot_result_from_trajectory` helper recomputes every metric through the
real `sgf_*` helpers: so numeric, Simulink, and single-integrator paths agree by
construction, not by copy.

## The chattering problem and the boundary-layer fix

The paper's controller `f_i = alpha * sum_j sgn(z_j - z_i) - (2 beta / R) sigma_i phi_i`
is exact for a **single integrator**: because point-offset feedback linearization
makes `d/dt s_i = f_i` hold *in continuous time*, the control point of a unicycle
should follow the *same* trajectory as the single-integrator model. It does not,
and the reason is **chattering**, not the gains or the integrator:

- Near the formation equilibrium the differences `z_j - z_i` are ~0, so each
  `sgn(...)` flips sign every step. The `alpha * sgn` term never settles: it
  produces a permanent high-frequency `f_i` with `|f_i|` of tens of m/s.
- In the *linear* single integrator this chatter is a benign symmetric jitter and
  the slow `beta` source-seeking drift accumulates cleanly (localization -> the
  source neighbourhood).
- Fed through the unicycle's **nonlinear heading coupling**
  (`v = f . [cos, sin]`, `omega = f . [-sin, cos] / r`), the chatter spins the
  heading at ~20 rad/s and scrambles that weak localization signal. The formation
  (a strong signal) still converges, but the centroid stalls ~0.5-0.8 m from the
  source: **outside** the theorem's epsilon-neighbourhood.

Diagnostics confirmed this: RK4 instead of Euler, and 4-8x finer `dt`, barely
moved localization (0.83 -> 0.55): because the problem is the discontinuous
right-hand side, not integration order.

**Fix: a boundary layer.** Replace the discontinuous `sgn(x)` with the saturated
approximation `sat(x / eps) = clip(x / eps, -1, 1)` (standard boundary-layer
sliding-mode control). When `|x| < eps` the term becomes smooth and vanishes at
equilibrium, so `omega -> 0`, the heading settles, and the control point tracks
the ideal single-integrator flow again. This is exposed as
`controller.sign_boundary_layer` (config) / `cfg.sign_boundary_layer` in the
shared `sgf_control`:

- `0` (default / absent) = the exact paper `sgn` controller. **Phase 1.3
  MATLAB<->Python parity is bit-identical in this mode** (max dev ~1e-15).
- `> 0` = boundary-layer width. The presets use **`0.2`**.

Boundary-layer sweep (unicycle, `alpha=10, beta=0.05, r=2, dt=0.004`, noise-free):

| `eps` | formation | localization | max `omega` |
|---|---|---|---|
| 0 (pure `sgn`) | 0.230 | **0.832** | 21.5 |
| 0.1 | 0.122 | 0.064 | 16.7 |
| **0.2** | **0.005** | **0.075** | 16.2 |

## Verification: now inside the theorem bound

### Numeric path (`turtlebot_working.json`, noise = bounded, `eps_bl = 0.2`)

```
initial formation    9.909 -> final 0.00523
initial localization 3.906 -> final 0.00089      epsilon bound = 0.100
Inside theorem bound: 1        max |v| = 29.7 m/s   max |omega| = 16.2 rad/s
```
Localization drops from 0.654 (pure `sgn`) to **0.0009**, well inside the
`epsilon = 0.1` bound, robustly across seeds (verified for seeds 1-3). Peak
velocities also fall (43.5 -> 29.7 m/s, 21.5 -> 16.2 rad/s) once the chatter is
gone.

### Simulink vs numeric (both noise = none, `eps_bl = 0.2`, `r = 2.0`, `dt = 0.004`)

| metric | Simulink `ode4` | Numeric Euler | \|diff\| |
|---|---|---|---|
| final formation error | 0.005236 | 0.005236 | 8e-10 |
| final localization error | 0.000781 | 0.000782 | 8e-07 |
| max \|v\| [m/s] | 29.79 | 29.74 | 5e-02 |
| max \|omega\| [rad/s] | 16.28 | 16.24 | 4e-02 |

Peak pointwise trajectory divergence: `|loc_err(t)| ~ 0.01`, `|form_err(t)| ~ 0.04`.

With the smooth boundary-layer control field, the RK4 (`ode4`) and explicit-Euler
paths now agree to **~1e-7** on the final metrics (before the fix they differed by
~0.13, because they took different paths across the `sgn` switching surface). The
solver choice is essentially irrelevant once chattering is removed. The Simulink
run's `inside_bound = 0` is only the degenerate noise-free case: with `delta = 0`
the theoretical `epsilon = delta / (kappa R) = 0`, so any nonzero residual is
"outside" a zero-width bound: `0.00078` is at the source to numerical precision.

## TurtleBot3 hardware note

`helpers/turtlebot_params.m` holds the real TurtleBot3 Burger constants
(`wheel_radius = 0.033 m`, `wheel_separation = 0.16 m`, `v <= 0.22 m/s`,
`omega <= 2.84 rad/s`, `command_period = 0.1 s`). The presets use **sane gains**
(`alpha = 10`, `beta = 0.05`), a **larger offset** `r = 2.0`, continuous comms,
the **`0.2` boundary layer**, and relaxed velocity limits (`null`) for a working
demonstration. The paper's own gains (`alpha = 100`, ratio 2000) command
control-point speeds of hundreds of m/s; with the true actuator limits the slow
localization term is starved. Set `max_linear_velocity` /
`max_angular_velocity` / `command_period` in a config to explore the
hardware-faithful regime (numeric path only).
