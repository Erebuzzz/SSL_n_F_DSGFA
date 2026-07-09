# CoppeliaSim Setup Guide — Step by Step

How to run the sign gradient-free source-localization + formation controller (Du et al.
2024, Phase 3) against a **live CoppeliaSim** physics simulation, end to end.

The `coppelia/` package drives a swarm of differential-drive robots through the CoppeliaSim
**ZeroMQ remote API**. It has two interchangeable backends behind one control loop:

- **`mock`** — a pure-Python kinematic backend. No CoppeliaSim, no extra install. Use it to
  verify the pipeline and produce artifacts offline. **Start here.**
- **`coppelia`** — the physics-in-the-loop backend. Connects to a running CoppeliaSim,
  builds the scene programmatically, drives the wheels, and reads back ground-truth poses.

> The `coppelia` backend is validated only against a live simulator (it is not exercised in
> the repo's offline tests). Treat first-run issues as expected and work through the
> troubleshooting section. The `mock` backend is your reference for "what correct output
> looks like."

---

## 0. Prerequisites

| Requirement | Notes |
|---|---|
| **Python 3.11+** (verified on 3.13) | `python -m pip install numpy matplotlib` |
| **CoppeliaSim 4.x** | The free **Edu** edition works. Download from coppeliarobotics.com. |
| **`coppeliasim-zmqremeteapi-client`** | Python client for the ZMQ remote API — **not** in `pyproject.toml`; install it manually (next step). |

The ZMQ remote API server is **enabled by default** in CoppeliaSim 4.x on TCP port **23000**
— you do not need to start an add-on or edit any config to enable it.

---

## 1. Install the Python remote-API client

```powershell
python -m pip install coppeliasim-zmqremoteapi-client
```

Import name is `coppeliasim_zmqremoteapi_client` (underscores). The `coppelia` package guards
this import, so the repo imports fine without it — the error only appears when you actually
connect with the `coppelia` backend. CoppeliaSim 4.x also bundles the same client under
`programming/zmqRemoteApi/clients/python`, but the pip install is the documented route.

Sanity check:

```powershell
python -c "import coppeliasim_zmqremoteapi_client; print('client OK')"
```

---

## 2. Smoke-test with the mock backend first (no simulator)

Confirm the control loop, metrics, plots, and animation all work before involving physics:

```powershell
python -m coppelia run --backend mock --noise bounded --seed 1 --run-id mock_smoke
```

Expected: a printed `summary.json`, then `Wrote artifacts to outputs/coppelia/mock_smoke`.
Look in `outputs/coppelia/mock_smoke/` for `trajectory.png`, `formation_error.png`,
`localization_error.png`, `formation_error_per_robot.png`, `animation.gif`, `summary.json`,
and `validation_report.md`. If this works, your Python side is correct and any later problem
is on the CoppeliaSim connection, not the algorithm.

---

## 3. Launch CoppeliaSim

1. Start **CoppeliaSim 4.x**.
2. Leave it on a **new / empty scene** (`File → New Scene`). You do **not** build anything by
   hand — the Python client constructs the whole scene for you over the remote API.
3. Do **not** press Play. The Python side calls `startSimulation()` itself.
4. Confirm the remote API server is listening on port 23000 (default). If you changed it, pass
   `--coppelia-port` to match.

That is the entire manual setup. Everything else is programmatic.

---

## 4. What the scene builder creates (so you know what to expect)

When you run the `coppelia` backend, `coppelia/scene/build_scene.py` does the following over
the remote API, for `n` robots (default 6):

1. **Loads a robot model** with `sim.loadModel(...)` — the **Pioneer p3dx** differential-drive
   model by default (`robots/mobile/pioneer p3dx.ttm`, resolved relative to your CoppeliaSim
   install). The `dr12` model is also selectable.
2. **Renames** each robot's base to the alias `robot[i]` (`i` zero-based).
3. **Places** each robot at its `initial_conditions.positions[i]` (or the default layout),
   keeping the model's own base height for z, and sets its heading from `initial_headings[i]`.
4. **Finds the two drive joints** by walking the model subtree and matching joint aliases
   containing `left` / `right` (case-insensitive). It raises `RuntimeError` if it can't find
   both — so a custom model must have identifiably named wheel motors.
5. **Creates a red source marker** — a static, non-collidable sphere (diameter 0.25 m) at
   `paper_parameters.source`, aliased `source_marker`.

Poses are read in the **world frame**; heading is the Z-Euler (yaw) angle. Units are SI
(meters, radians, m/s, rad/s).

> **Wheel-geometry caveat.** The default `wheel_radius = 0.033` and `wheel_base = 0.16` are
> TurtleBot3 Burger values, but the default loaded model is the Pioneer p3dx, whose geometry
> differs. For accurate physics, set `robot_model.wheel_radius` and `robot_model.wheel_base`
> in your JSON to the actual model's values (there is no CLI flag for these). A wrong wheel
> radius/base scales the commanded wheel speeds and will bias the motion.

---

## 5. Run the physics backend

```powershell
# direct flags
python -m coppelia run --backend coppelia --noise bounded --run-id physics_demo

# or from a JSON config (set "coppelia": {"backend": "coppelia"} inside it)
python -m coppelia run-config coppelia/configs/coppelia_default.json --backend coppelia
```

While it runs you should see the robots spawn, the red source appear, and the swarm drive
toward a circular formation around the source. Artifacts are written to
`outputs/coppelia/<run_id>/` — the same set as the mock backend, so you can diff mock vs
physics directly.

**Exit code:** `0` if the final localization error is inside the epsilon bound, `3` if not
(useful for scripting). With the physically-sane default gains (`alpha=10, beta=0.05`,
ratio 200) the gain condition and the bound are **not** met by design — see §7.

---

## 6. CLI reference (the `run` subcommand)

All flags with their defaults. Run from the repository root.

| Flag | Default | Meaning |
|---|---|---|
| `--backend {mock,coppelia}` | `mock` | Kinematic mock vs live CoppeliaSim. |
| `--duration` | `90.0` | Simulation seconds. |
| `--dt` | `0.004` | Control/integration period (also the comms period — see note). |
| `--seed` | `1` | RNG seed (noise + mock actuator noise). |
| `--noise {none,gaussian,bounded}` | `gaussian` | Measurement noise model. Use `bounded` for theorem checks. |
| `--alpha` / `--beta` | `10.0` / `0.05` | Formation / localization gains (ratio 200 by default). |
| `--radius` | `2.0` | Formation radius `R`. |
| `--dmax` | `12.0` | Sensing saturation distance. |
| `--noise-std` / `--noise-bound` | `0.2` / `0.2` | Gaussian std / bounded-noise bound. |
| `--topology` | `paper_fig1_reconstructed` | Graph preset (see CONFIG_GUIDE). |
| `--offset` | `0.5` | Feedback-linearization control-point offset `r`. |
| `--sign-boundary-layer` | `0.0` | `0` = exact paper `sgn`; `>0` = chatter-free `sat(x/eps)`. |
| `--max-v` / `--max-omega` | off | Optional actuator saturation. |
| `--coppelia-host` / `--coppelia-port` | `localhost` / `23000` | Remote API endpoint. |
| `--no-animation` | off | Skip GIF/MP4 export. |
| `--animation-format {gif,mp4}` | `gif` | Animation container. |
| `--output-dir` | `outputs/coppelia` | Base output folder. |
| `--run-id` | derived | Output subfolder name. |
| `--no-report` | off | Skip `validation_report.md`. |

> `run-config` accepts only `config_path`, plus `--backend` and `--run-id`; all other tuning
> comes from the JSON. Change gains/topology/positions in the file, not on the command line,
> when using `run-config`.

> **Comms period.** The loop uses `dt` as the control period; the paper's Section V sampled
> comms period `T = 0.1 s` is not separately modelled in the coppelia loop (a documented
> follow-up). Numeric unicycle / MATLAB TurtleBot paths do expose a comms period.

### Using a hand-built scene instead of the auto-builder

The auto-builder is on by default. To drive a scene you built yourself, name the robot bases
`/robot[i]` with `/robot[i]/leftMotor` and `/robot[i]/rightMotor` joints, and construct the
backend in code with `ZmqBackend(config, build_scene=False)` — there is no CLI switch for
this path.

---

## 7. Getting inside the theorem bound (tuning recipe)

The Phase 3 defaults (`alpha=10, beta=0.05`, ratio 200, offset 0.5, exact `sgn`) are chosen
for **physical realizability on differential-drive robots**, not to satisfy Theorem 1's
conservative sufficient condition (threshold `4·n·f_Dmax/R ≈ 1730` for the defaults). So a
default run sits outside the bound *by design*. To reproduce a theorem-satisfying, chatter-free
run — the same recipe demonstrated in `outputs/coppelia/mock_gainpass_tuned/` — use:

```powershell
python -m coppelia run --backend coppelia --noise bounded ^
  --alpha 100 --beta 0.05 ^
  --sign-boundary-layer 0.2 ^
  --dt 0.001 ^
  --offset 5.0 ^
  --run-id physics_tuned
```

Why each knob (all strictly within the paper's framework):

- **`--alpha 100`** → ratio 2000 > 1730 satisfies the sufficient gain condition.
- **`--sign-boundary-layer 0.2`** → replaces the relay `sgn` with the sliding-mode boundary
  layer `sat(x/eps)`, removing the chattering that otherwise corrupts localization on
  differential-drive robots (and that can cause transient sensing dropouts that *widen* the
  bound).
- **`--dt 0.001`** → a smaller step keeps the discretized high-gain discontinuous controller
  numerically stable (large `alpha` + coarse `dt` is unstable under explicit integration).
- **`--offset 5.0`** → a larger control-point offset keeps the feedback-linearized angular
  velocity physically reasonable at the higher gain.

See [docs/OUTPUT_ANALYSIS.md](OUTPUT_ANALYSIS.md) §3.1 for the full analysis behind this recipe.

---

## 8. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ImportError` / install hint on connect | `coppeliasim-zmqremoteapi-client` not installed (§1). |
| Connection refused / timeout | CoppeliaSim not running, or remote API not on port 23000. Start it, or set `--coppelia-port`. |
| `RuntimeError` finding wheel motors | Robot model lacks joints with `left`/`right` in their aliases. Use Pioneer p3dx, or rename your model's wheel joints. |
| Model fails to load | `sim.loadModel` path not found — the model is resolved under the CoppeliaSim install's `models/` dir; confirm the Pioneer model exists in your install. |
| Robots drift wrong distance / spin oddly | Wheel geometry mismatch — set `robot_model.wheel_radius` / `wheel_base` to your model's real values (§4 caveat). |
| Heavy chattering, localization stalls outside bound | Expected with exact `sgn`; add `--sign-boundary-layer 0.2` (§7). |
| Runs but `inside_bound: false`, exit code 3 | Default gains don't meet the sufficient condition — use the §7 recipe if you need the bound. |

---

## 9. Outputs

Written to `outputs/coppelia/<run_id>/` (git-ignored):

- `trajectory.png` — robot trails, target circle, source, centroid path
- `formation_error.png` — aggregate consensus error vs time
- `formation_error_per_robot.png` — per-robot formation error (paper Fig. 3)
- `localization_error.png` — centroid-to-source distance with the epsilon bound
- `animation.gif` / `.mp4` — 4 synchronized panels (unless `--no-animation`)
- `summary.json` — parameters, theory validation, and metrics
- `validation_report.md` — human-readable report (unless `--no-report`)

The default `run-id` when omitted is `<backend>_<topology>_<noise>_seed<seed>`.

---

## 10. Quick reference

```powershell
# 1. install client
python -m pip install coppeliasim-zmqremoteapi-client
# 2. offline sanity check
python -m coppelia run --backend mock --noise bounded --run-id mock_smoke
# 3. launch CoppeliaSim 4.x on an empty scene (don't press Play)
# 4. physics run (default physically-sane gains)
python -m coppelia run --backend coppelia --noise bounded --run-id physics_demo
# 5. physics run inside the theorem bound
python -m coppelia run --backend coppelia --noise bounded --alpha 100 --beta 0.05 --sign-boundary-layer 0.2 --dt 0.001 --offset 5.0 --run-id physics_tuned
```
