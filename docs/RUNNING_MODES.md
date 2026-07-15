# Running Guide — Every Mode, Exact Commands

This is the canonical "how do I run it" reference for the whole reproduction of
Du et al. (2024), *Simultaneous Source Localization and Formation via a Distributed
Sign Gradient-Free Algorithm*. Each mode below lists its **prerequisites**, the
**exact commands**, the **configs** it uses, and the **expected result**.

All commands are run **from the repository root** unless noted. Shell snippets use
PowerShell; MATLAB snippets use the MATLAB command window.

## Modes at a glance

| # | Mode | Runtime | Entry point | Phase |
|---|------|---------|-------------|-------|
| 1 | Single-integrator (point robots) | Python | `python -m sgf_sim` | 1 / 1.1 / 1.2 / 1.25 |
| 2 | Single-integrator parity | MATLAB | `matlab/` scripts | 1.3 |
| 3 | Numerical unicycle | Python | `python -m sgf_sim unicycle` | 2 |
| 4 | TurtleBot differential-drive (numeric) | MATLAB | `matlab_turtlebot/` | 2.5 |
| 5 | TurtleBot swarm (Simulink block diagram) | MATLAB/Simulink | `matlab_turtlebot/` | 2.5 |
| 6 | Multi-robot simulator | Python + CoppeliaSim | `python -m coppelia` | 3 |

Modes 1–3 and 6 (mock) run offline with only Python + NumPy + Matplotlib. Modes
2/4/5 need MATLAB (+ Simulink for mode 5). Mode 6 physics needs CoppeliaSim.

## Prerequisites

```powershell
# Python (3.11+, verified on 3.13)
python -m pip install numpy matplotlib pytest
```

- **MATLAB** (verified on R2026a) for modes 2, 4, 5. Mode 5 also needs **Simulink**.
- **CoppeliaSim 4.x** + `coppeliasim-zmqremoteapi-client` for mode 6 physics only
  (mode 6 `mock` backend needs neither).

> **pytest note.** All test commands add `-p no:hypothesispytest`. This works
> around a local environment issue where the installed Hypothesis pytest plugin
> imports `trio` → a missing `attrs` during teardown. The project tests do not use
> Hypothesis.

## Cross-cutting option: the signum controller (`sign_boundary_layer`)

The formation term uses the paper's component-wise `sgn`. Both Python and MATLAB
expose a **boundary-layer** option that swaps `sgn(x)` for the saturated
`sat(x/eps) = clip(x/eps, -1, 1)`:

| Value | Behaviour |
|---|---|
| `0` (default) | Exact paper `sgn`. MATLAB↔Python parity is bit-identical. On unicycle/TurtleBot robots the `sgn` chattering corrupts localization (settles **outside** the bound). |
| `0.2` (presets) | Boundary layer — removes chattering, so the control point tracks the ideal flow and localization lands **inside** the bound. |

- **Python:** CLI flag `--sign-boundary-layer 0.2`, or JSON `"controller": {"sign_boundary_layer": 0.2}`.
- **MATLAB:** JSON `"controller": {"sign_boundary_layer": 0.2}` (read by `sgf_control`).

See [docs/CONFIG_GUIDE.md](CONFIG_GUIDE.md) and
[matlab_turtlebot/README.md](../matlab_turtlebot/README.md) for the full analysis.

---

## Mode 1 — Single-integrator (Python)

The Phase 1 point-robot simulator: `p_dot_i = u_i` with the Eq. 4 control law.

```powershell
# paper-like Gaussian-noise run
python -m sgf_sim run --noise gaussian --seed 1 --run-id paper_like_gaussian_seed1

# theorem-faithful bounded-noise validation (reports inside_bound)
python -m sgf_sim validate --seed 1

# full paper preset (Gaussian similarity + bounded theorem check)
python -m sgf_sim validate-paper --seed 1

# parameter sweeps and experiment suites
python -m sgf_sim sweep --param R --noise bounded
python -m sgf_sim sweep --param topology --noise bounded
python -m sgf_sim gain-sweep --ratio 2000 --noise gaussian
python -m sgf_sim experiment paper-suite
python -m sgf_sim experiment seed-sweep --seeds 1 2 3 4 5

# run from a shared JSON config
python -m sgf_sim run-config configs/paper_default.json
python -m sgf_sim run-config configs/paper_bounded_validation.json

# paper Fig. 3 timescale (formation completes ~5 s, then localizes)
python -m sgf_sim run-config configs/paper_timescale.json
```

- **Configs:** `configs/paper_default.json` (Gaussian), `configs/paper_bounded_validation.json` (bounded),
  `configs/paper_timescale.json` (bounded, paper-faithful *timescale*).
- **Output:** `outputs/runs/<run_id>/` — `trajectory.png`, `formation_error.png`,
  `formation_error_per_robot.png` (paper Fig. 3), `localization_error.png`, `summary.json`.
- **Expected:** bounded validation reports `"inside_bound": true` (loc ≈ 0.03 vs ε = 0.1).
  `paper_timescale` reports `formation_entry_time` ≈ 4.9 s and localization decaying over
  ~40–60 s (final loc ≈ 0.17) — see the timescale note below.

> **Formation timescale (why the default finishes instantly).** The formation term is a
> **finite-time sliding-mode** controller, so its convergence time scales as `≈ 3 / alpha`.
> The default presets use `alpha = 100–2000` (chosen to satisfy the conservative *sufficient*
> gain ratio `alpha/beta > 4 n f_Dmax / R ≈ 1730` and land inside the tight ε = 0.1 bound), so
> formation completes in milliseconds. The paper's Fig. 3 shows ~5 s because it uses a modest
> `alpha ≈ 1`. That gain ratio is **sufficient, not necessary**: taken literally with a small
> `alpha` it forces `beta ≈ alpha/1730`, making localization (rate `2 beta kappa`) take
> ~1400 s. `configs/paper_timescale.json` therefore uses `alpha = 1.0, beta = 0.03` (ratio 33 —
> below the sufficient bound but well inside the empirically stable region) to reproduce the
> paper's ~5 s formation **and** a visible localization transient, at the cost of the tight
> ε = 0.1 guarantee. Use the `paper_*` bounded/Gaussian presets when you want the theorem-faithful
> bound; use `paper_timescale` when you want the paper's Fig. 3 / Fig. 4 *dynamics*.

```powershell
python -m pytest tests -p no:hypothesispytest -q
```

---

## Mode 2 — Single-integrator parity (MATLAB)

The MATLAB re-implementation of Mode 1, reading the *same* JSON configs, for
control-theory review and author-workflow parity.

```matlab
addpath('matlab')

% run from the shared configs
run_from_config('configs/paper_default.json')
run_from_config('configs/paper_bounded_validation.json')

% convenience presets
run_single_simulation
run_paper_validation

% deterministic Python<->MATLAB parity check (noise = none)
verify_parity
```

- **Configs:** same `configs/paper_default.json`, `configs/paper_bounded_validation.json`.
- **Output:** `outputs/runs/<run_id>_matlab/` — plots, `summary.json`, `result.mat`.
- **Expected:** `verify_parity` prints `RESULT: PASS` with max deviation ~1e-15.
  Exact equality with Python holds only for `noise.model = "none"` (NumPy and
  MATLAB use different RNGs).

---

## Mode 3 — Numerical unicycle (Python)

Phase 2: point-offset feedback linearization of the sign controller onto unicycle
robots, with sampled communication and optional actuator saturation. **CLI-driven**
(the shared-config loader intentionally blocks `unicycle` mode in Python).

```powershell
# original paper controller (exact sgn) -- localization stalls OUTSIDE the bound
python -m sgf_sim unicycle validate --duration 90 --dt 0.004

# chattering-free boundary layer -- localizes INSIDE the bound
python -m sgf_sim unicycle validate --duration 90 --dt 0.004 --sign-boundary-layer 0.2

# a single run, or a head-to-head vs the single integrator
python -m sgf_sim unicycle run --noise bounded --sign-boundary-layer 0.2
python -m sgf_sim unicycle compare-single-integrator --noise none --sign-boundary-layer 0.2

# actuator limits / sampled comms
python -m sgf_sim unicycle run --max-v 5 --max-omega 10 --comms-period 0.1 --sign-boundary-layer 0.2
```

Key flags: `--alpha` (10), `--beta` (0.05), `--offset` r (2.0), `--comms-period` T
(0 = continuous), `--max-v` / `--max-omega` (off by default), `--sign-boundary-layer`
(0 = exact sgn).

- **Configs:** none — parameters come from flags. `configs/unicycle_default.json` is
  the shared-schema example (consumed by the MATLAB TurtleBot layer, not Python).
- **Output:** `outputs/unicycle/<action>_.../` — plots + `summary.json`.
- **Expected:** `validate` returns exit 0 and `"inside_bound": true` with
  `--sign-boundary-layer 0.2` (loc ≈ 0.0006); exit 3 and `false` with pure `sgn`
  (loc ≈ 0.74).

```powershell
python -m pytest tests/test_unicycle.py -p no:hypothesispytest -q
```

---

## Mode 4 — TurtleBot differential-drive, numeric (MATLAB)

Phase 2.5: the sign controller on TurtleBot-style differential-drive robots via
feedback linearization, reusing the Phase 1.3 `matlab/` control helpers. Explicit
Euler integration, sampled comms, optional actuator saturation.

```matlab
addpath('matlab_turtlebot')
run_turtlebot_from_config('matlab_turtlebot/configs/turtlebot_working.json')
```

- **Config:** `matlab_turtlebot/configs/turtlebot_working.json` (bounded noise,
  `alpha=10, beta=0.05, r=2.0`, continuous comms, `sign_boundary_layer = 0.2`).
- **Output:** `outputs/turtlebot/turtlebot_working_turtlebot/` — `trajectory.png`,
  `formation_error.png`, `localization_error.png`, `motion.gif`, `summary.json`,
  `result.mat`.
- **Expected:** `Inside theorem bound: 1`, final formation ≈ 0.005, localization
  ≈ 0.0009 (vs ε = 0.1). Set `sign_boundary_layer` to `0` in the config to see the
  original pure-`sgn` behaviour (localization ≈ 0.65, outside the bound).

---

## Mode 5 — TurtleBot swarm, Simulink (MATLAB/Simulink)

Phase 2.5 centerpiece: a genuine Simulink block diagram (vector `Integrator` +
`MATLAB Function` ODE block for the sign controller / measurement / feedback
linearization / diff-drive dynamics), built programmatically and simulated with a
fixed-step solver.

```matlab
addpath('matlab_turtlebot')

% build the .slx from the config, sim() it, and write all artifacts
run_turtlebot_from_config('matlab_turtlebot/configs/turtlebot_simulink_default.json')

% (optional) build the model only, without running / writing artifacts
cfg = tb_config('matlab_turtlebot/configs/turtlebot_simulink_default.json');
addpath('matlab_turtlebot/helpers'); addpath('matlab');
build_turtlebot_simulink_model(cfg, 'matlab_turtlebot/models');
```

- **Config:** `matlab_turtlebot/configs/turtlebot_simulink_default.json`
  (`mode = "turtlebot_simulink"`, `noise = none` — a deterministic continuous
  reference, `sign_boundary_layer = 0.2`, solver `ode4`).
- **Model:** `matlab_turtlebot/models/sgf_turtlebot_swarm.slx` (also regenerated per
  run into the output folder's `models/`).
- **Output:** `outputs/turtlebot/turtlebot_simulink_default_turtlebot/` — same
  artifact set as Mode 4, plus the generated `.slx`.
- **Expected:** final formation ≈ 0.005, localization ≈ 0.0008 (at the source to
  numerical precision). With `sign_boundary_layer = 0.2` the Simulink (`ode4`) and
  numeric (Euler) paths agree to ~1e-7. `Inside theorem bound: 0` here is only the
  degenerate noise-free case (ε = δ/(κR) = 0 when δ = 0).

---

## Mode 6 — CoppeliaSim multi-robot (Python)

Phase 3: the same control law driving a `RobotBackend` — either an offline
kinematic **mock** backend or a physics-in-the-loop **CoppeliaSim** backend. The
`coppelia/` package is self-contained (it does not import `sgf_sim`).

```powershell
# offline mock backend (no CoppeliaSim needed)
python -m coppelia run --backend mock --noise bounded --seed 1 --run-id mock_demo

# from a shared JSON config
python -m coppelia run-config coppelia/configs/mock_bounded.json
python -m coppelia run-config coppelia/configs/coppelia_default.json --backend mock
```

For physics: install `coppeliasim-zmqremoteapi-client`, launch CoppeliaSim 4.x with
an empty scene (ZMQ remote API on port 23000), then:

```powershell
python -m coppelia run --backend coppelia --noise bounded --run-id physics_demo
```

> **Full step-by-step CoppeliaSim walkthrough:** see
> [docs/COPPELIASIM_SETUP_GUIDE.md](COPPELIASIM_SETUP_GUIDE.md) — install, scene builder,
> wheel-geometry caveats, the inside-the-bound tuning recipe, and troubleshooting.

- **Configs:** `coppelia/configs/mock_bounded.json`, `coppelia/configs/coppelia_default.json`.
- **Output:** `outputs/coppelia/<run_id>/` — `trajectory.png`, error PNGs,
  `animation.gif`, `summary.json`, `validation_report.md`.

```powershell
python -m pytest coppelia/tests -p no:hypothesispytest -q
```

---

## Unified launcher (all modes, one window)

A tkinter GUI that configures and runs any mode, showing a live theory readout (gain
condition, epsilon bound, informed-robot diagnostics) before the run:

```powershell
python -m launcher
```

Pick a Platform (Python / MATLAB / CoppeliaSim) and Mode (auto-filtered), set the source,
`R`, `Dmax`, gains, noise, `sgn` controller, `dt`, and each robot's initial position +
`informed` flag, then Run. Python and CoppeliaSim-mock run in-process; MATLAB is dispatched
via `matlab -batch`; CoppeliaSim physics drives a live simulator. See
[launcher/README.md](../launcher/README.md) for details. The headless logic lives in
`launcher.core` (covered by `tests/test_launcher.py`).

## Config inventory

| Config | Mode | Used by |
|---|---|---|
| `configs/paper_default.json` | single_integrator (Gaussian) | Mode 1, Mode 2 |
| `configs/paper_bounded_validation.json` | single_integrator (bounded) | Mode 1, Mode 2 |
| `configs/paper_timescale.json` | single_integrator (bounded, paper Fig. 3 timescale) | Mode 1, Mode 2 |
| `configs/unicycle_default.json` | unicycle | shared-schema example (MATLAB TurtleBot layer) |
| `configs/turtlebot_simulink_default.json` | turtlebot_simulink | shared-schema example |
| `matlab_turtlebot/configs/turtlebot_working.json` | turtlebot_numeric | Mode 4 |
| `matlab_turtlebot/configs/turtlebot_simulink_default.json` | turtlebot_simulink | Mode 5 |
| `coppelia/configs/mock_bounded.json` | single_integrator (mock backend) | Mode 6 |
| `coppelia/configs/coppelia_default.json` | coppelia | Mode 6 |

## Expected bound status (quick reference)

| Mode | Controller | Final localization | Inside ε = 0.1? |
|---|---|---|---|
| 1 single-integrator (bounded) | exact `sgn`, `alpha/beta = 2000` | ≈ 0.03 | ✅ |
| 3 unicycle (bounded) | exact `sgn` | ≈ 0.74 | ❌ |
| 3 unicycle (bounded) | boundary layer 0.2 | ≈ 0.0006 | ✅ |
| 4 TurtleBot numeric (bounded) | boundary layer 0.2 | ≈ 0.0009 | ✅ |
| 5 TurtleBot Simulink (noise-free) | boundary layer 0.2 | ≈ 0.0008 | at source (ε = 0) |

## Outputs and cleanup

All run artifacts are written under `outputs/` (git-ignored). Simulink build
artifacts (`slprj/`, `*.slxc`) are also git-ignored. Delete `outputs/` any time to
reclaim space; every run regenerates its own folder.
