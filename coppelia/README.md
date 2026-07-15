# Phase 3: CoppeliaSim Multi-Robot Build

This folder is the **Phase 3** build from `docs/ROADMAP.md`: it moves the Du et al.
(2024) sign gradient-free source-localization and formation control law off pure
numerical integration (Phase 1) and the numerical unicycle model (Phase 2) and
into a physics-capable multi-robot simulator, **CoppeliaSim**.

It is fully self-contained: nothing here imports `sgf_sim`, so it can be developed
and run while Phases 1/2 are still changing. The control math (Eq. 4), the
measurement/saturation model, the topologies, and the Theorem-1 formulas are
mirrored locally from the Phase 1 code and kept numerically identical.

## Two backends behind one interface

The control loop only ever sees the `RobotBackend` protocol, so the exact same
algorithm drives either backend by changing one config field (`backend`):

| Backend | Requires | Purpose |
|---|---|---|
| `mock` | nothing (pure Python + numpy) | Offline kinematic differential-drive stand-in. Runs everywhere, powers the test-suite, produces all artifacts. |
| `coppelia` | a running CoppeliaSim 4.x + the ZMQ remote-API client | Physics-in-the-loop run: wheel dynamics, inertia, actuator limits. |

The `mock` backend integrates the unicycle kinematics
`x_dot = v cos θ, y_dot = v sin θ, θ_dot = ω` with a fixed Euler step. It does not
model slip/inertia/collisions — that realism is exactly what the CoppeliaSim
backend adds. Because the interface is identical, a scene validated in mock mode
switches to physics by flipping `--backend coppelia`.

## Quick start (offline, no CoppeliaSim needed)

From the repository root:

```powershell
# one run against the mock kinematic backend
python -m coppelia run --backend mock --noise bounded --seed 1 --run-id mock_demo

# run the Phase 3 tests (kept separate from the root test-suite)
python -m pytest coppelia/tests -p no:hypothesispytest -q
```

Artifacts land in `outputs/coppelia/<run_id>/`:

- `trajectory.png` — robot trails, target circle, source, centroid path, over a
  filled contour (heatmap) of the scalar source field `f(z) = kappa*||z - p_s||^2`
  with a dashed `Dmax` sensing-radius circle
- `formation_error.png`, `localization_error.png` — raw + smoothed curves
- `animation.gif` (or `.mp4`) — 4 synchronized panels (formation+source over the
  field heatmap, trails, live formation error, live localization error)
- `summary.json` — same shape as the numerical phases
- `telemetry.npz` / `telemetry.csv` — full per-step signal history (poses, control
  points, command `f_i`, commanded `v`/`omega`, field samples `sigma_i`, per-robot
  informed flags, error metrics) for offline analysis; on by default, disable with
  `--no-telemetry`
- `validation_report.md` — collaborator-readable report

## Running against CoppeliaSim

1. Install [CoppeliaSim](https://www.coppeliarobotics.com/) 4.x (Edu is free).
2. Install the Python ZeroMQ remote-API client:

   ```powershell
   python -m pip install coppeliasim-zmqremoteapi-client
   ```

3. Launch CoppeliaSim. The ZMQ remote API add-on is enabled by default on port
   `23000`; leave an **empty scene open** (the scene is built programmatically).
4. Run:

   ```powershell
   python -m coppelia run --backend coppelia --noise bounded --run-id coppelia_demo
   ```

The `coppelia` backend builds the scene through the remote API
(`coppelia/scene/build_scene.py`): it loads `n` differential-drive robot models
(default: the built-in **Pioneer p3dx**), arranges them at the configured initial
positions, drops a red source marker, draws concentric field contour rings on the
floor around the source, then drives the wheels each control period. See
`coppelia/scene/README.md` for scene details and the manual-setup alternative.

> The `coppelia` backend cannot be exercised in this offline repo checkout — it is
> validated by running it against a live CoppeliaSim instance. The import of the
> client is guarded, so everything else (including the whole mock pipeline and the
> test-suite) works without CoppeliaSim installed. Instantiating the backend
> without the client raises a clear install hint.

## Config files

Runs can be driven from a JSON config in the Phase 1.25 shared-schema shape
(tolerant to missing sections):

```powershell
python -m coppelia run-config coppelia/configs/coppelia_default.json
python -m coppelia run-config coppelia/configs/mock_bounded.json --backend mock
```

## Architecture

```text
coppelia/
  config.py                  # CoppeliaConfig dataclass (+ JSON loader)
  control.py                 # Eq. 4 control law, measurement/saturation (self-contained)
  unicycle.py                # feedback linearization + diff-drive wheel conversion
  theory.py                  # Theorem-1 gain threshold + epsilon bound
  topology.py                # named graphs (paper_fig1_reconstructed, ring, ...)
  metrics.py                 # error metrics + CoppeliaResult.summary()
  experiment.py              # backend-agnostic control loop
  plotting.py                # static plots + 4-panel animation
  reporting.py               # markdown validation report
  run_coppelia_experiment.py # CLI (python -m coppelia)
  backends/
    base.py                  # RobotBackend protocol + RobotState
    mock.py                  # pure-Python differential-drive kinematics
    zmq_backend.py           # CoppeliaSim ZeroMQ remote-API backend
  scene/
    build_scene.py           # programmatic scene construction via remote API
  configs/                   # JSON presets
  tests/                     # offline pytest suite (mock backend)
```

Per control period the loop: reads ground-truth poses → forms the control points
`s_i = p_i + r[cosθ, sinθ]` → measures the (saturated, noisy) field at each `s_i` →
evaluates Eq. 4 to get the single-integrator command `f_i` → inverts the unicycle
map to `(v_i, ω_i)` → clips to limits → commands wheels → steps the backend →
records metrics. Metrics are computed on the **control points** `s_i` (the states
the single-integrator algorithm governs), so they line up with Phases 1/2.

## Theory vs. the physical unicycle (important)

This is the central Phase-3 finding and it is worth understanding before reading
the numbers.

- **The control law is exact.** Integrating the control points as a pure single
  integrator reproduces the Phase 1 result (final localization error ≈ 0.02–0.03).
- **Theorem 1's gain condition is brutally conservative.** For the default paper
  parameters it demands `α/β > ~1730`. The Phase 1 numerical run uses `α=100,
  β=0.05` (ratio 2000), which commands control-point speeds of *hundreds of m/s*.
  That is fine for a point integrator but physically impossible for a robot.
- **The unicycle inversion amplifies this.** The commanded angular velocity scales
  as `ω ≈ |f|/r`. With huge `f` and a small offset `r`, `ω` explodes and no
  reasonable Euler step (or real actuator) can track it — the feedback
  linearization breaks and the run diverges.
- **So Phase 3 defaults to physically-sane gains** (`α=10, β=0.05`, ratio 200,
  offset `r=0.5`, velocity limits disabled). Consequences:
  - Formation error still converges quickly (to ≈ 0.1–0.2).
  - The centroid **localizes toward the source** but **plateaus above the tight
    numerical bound** (typically final localization error ≈ 0.3–1.2 over 50–90 s,
    seed-dependent), because a small residual formation error biases the symmetric
    gradient estimate. Driving it to ≈ 0.03 would require the ≈2000 ratio, i.e.
    unphysical speeds.
  - `gain_condition_passed` is reported as **false** on purpose — the sufficient
    (not necessary) condition is knowingly not met, and the algorithm converges
    anyway. This is exactly the "theory is conservative; test below the bound"
    experiment called out in the extraction notes.

`summary.json` still reports `epsilon` and `inside_bound` so the gap between the
theorem's ideal bound and the achievable physical behavior is quantified rather
than hidden. Reproducing the tight numerical bound is a single-integrator
property; the CoppeliaSim/unicycle build is expected to differ, and this build
measures that difference (a Phase-3 acceptance criterion in the roadmap).

### Knobs to explore the trade-off

| Flag | Effect |
|---|---|
| `--alpha`, `--beta` | Raise the ratio toward the theorem (tighter localization, faster/less-physical motion). |
| `--offset` | Larger `r` tames `ω` and stabilizes the inversion; smaller `r` is closer to the paper's 2.5 cm hardware value but needs a smaller `--dt`. |
| `--max-v`, `--max-omega` | Enable actuator saturation. Realistic, but starves the slow localization term of velocity budget. |
| `--dt` | Smaller step keeps the linearization valid under larger gains. |

## Known limitations / next steps

- The `coppelia` backend is written against the CoppeliaSim 4.x ZMQ remote API but
  is validated only on a live simulator, not in offline CI.
- Actuator saturation, wheel slip, and inertia are modelled by CoppeliaSim, not by
  the mock backend.
- Sampled-data communication (`T = 0.1 s`, Section V) is not yet modelled — the
  loop uses `dt` as the control period. Adding a separate comms period is a natural
  follow-up (and overlaps with Phase 4).
- Feeding the same shared JSON config used by the Python/MATLAB numerical phases
  (Phase 1.25) will unify parameters once that schema lands; the loader here
  already accepts that shape.
