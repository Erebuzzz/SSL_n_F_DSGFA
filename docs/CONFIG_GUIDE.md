# Shared Config Guide

The shared config files in `configs/` are the source of truth for reproducible experiments across Python and future MATLAB or Simulink implementations.

The format is JSON because Python can read it with the standard `json` module and MATLAB can read it with `jsondecode`.

## Files

| File | Purpose |
|---|---|
| `configs/paper_default.json` | Paper-like Gaussian single-integrator run. |
| `configs/paper_bounded_validation.json` | Theorem-valid bounded-noise single-integrator run. |
| `configs/paper_timescale.json` | Paper Fig. 3 *timescale* — modest gains (`alpha=1.0, beta=0.03`) so formation completes in ~5 s then localizes. Trades the tight ε=0.1 bound for paper-faithful dynamics. |
| `configs/unicycle_default.json` | Planned numerical unicycle run. |
| `configs/turtlebot_simulink_default.json` | Planned MATLAB/Simulink TurtleBot run. |

## Sections

### `experiment`

Controls run identity and integration timing.

| Field | Meaning |
|---|---|
| `name` | Human-readable experiment name and default output folder name. |
| `mode` | Runtime mode: `single_integrator`, `unicycle`, `turtlebot_numeric`, `turtlebot_simulink`, or `coppelia`. |
| `seed` | Random seed for noise generation. |
| `duration` | Total simulation time in seconds. |
| `dt` | Numerical integration step in seconds. |

Implemented paths: `single_integrator` (Python `sgf_sim` + MATLAB `matlab/`), `coppelia`
(Python `coppelia` package, mock + physics backends), `turtlebot_numeric` and
`turtlebot_simulink` (MATLAB `matlab_turtlebot/`). The Python `sgf_sim` unicycle model
is CLI-driven (`python -m sgf_sim unicycle …`) rather than config-driven. All modes share
this one config shape so parameters stay comparable across paths.

### `paper_parameters`

Maps directly to the paper model.

| Field | Meaning |
|---|---|
| `n` | Number of robots. Section IV uses 6. |
| `source` | Source position `p_s`. Section IV uses `[5.5, 5.5]`. |
| `kappa` | Source-field strength in `f(z) = kappa * ||z - p_s||^2`. |
| `R` | Desired formation radius. Section IV uses 2. |
| `Dmax` | Maximum sensing distance. Section IV uses 12. |
| `alpha` | Formation gain. |
| `beta` | Localization gain. |

The gain ratio `alpha / beta` should be compared against `4 * n * f_Dmax / R` (the paper's
**sufficient**, not necessary, stability condition).

**Absolute gains set the timescale.** The formation term is finite-time, so formation completes
in `≈ 3 / alpha` seconds; localization decays exponentially at rate `2 * beta * kappa`. The
`paper_default` / `paper_bounded_validation` presets use large `alpha` (100–2000) to satisfy the
conservative ratio and land inside the tight ε bound — formation then finishes almost instantly.
To reproduce the paper's ~5 s formation (Fig. 3) use `alpha ≈ 1`; because the ratio is sufficient
(not necessary), `configs/paper_timescale.json` drops to `alpha=1.0, beta=0.03` (ratio 33) to get
the paper timescale for both formation and localization. See `docs/RUNNING_MODES.md` (Mode 1) for
the full explanation.

### `noise`

Controls the scalar measurement perturbation.

| Field | Meaning |
|---|---|
| `model` | `gaussian`, `bounded`, or `none`. |
| `std` | Standard deviation for Gaussian noise. |
| `bound` | Absolute bound for theorem-valid bounded noise. |

Use `gaussian` for paper-like visual similarity. Use `bounded` for strict theorem validation.

### `topology`

Controls graph communication.

| Field | Meaning |
|---|---|
| `name` | Named topology, such as `paper_fig1_reconstructed`. |
| `adjacency` | Optional full adjacency matrix. If present, it overrides `edges`. |
| `edges` | Optional undirected edge list using zero-based robot indices. |

If both `adjacency` and `edges` are null, Python uses the named topology.

> **Loader divergence (important).** The three runtimes resolve `topology` differently.
> Python `sgf_sim` and MATLAB use `adjacency` → `edges` → `name` (so a present `edges`
> list wins over `name`). The `coppelia` package uses `adjacency` → `name`, and only reads
> `edges` when `name` is null or `"custom"`. The shipped configs carry **both** a
> `name: "paper_fig1_reconstructed"` and an explicit 6-node `edges` list, so the same file
> can build its graph from different sources across runtimes. When you change the topology
> or `n`, set the field you actually want and null out the other, or set `name: "custom"`
> and supply `edges`/`adjacency` for the new size.

### `initial_conditions`

Optional. Sets each robot's starting pose explicitly. **Omit the whole section** to keep the
built-in 6-robot layout (fully back-compatible — this is what every shipped config does).

> **Default layout follows the source.** When `positions` is omitted, the built-in layout is
> defined *relative to* `paper_parameters.source`: it is the paper geometry translated by
> `source − [5.5, 5.5]`. So changing the source moves the whole starting formation with it,
> keeping every robot within `Dmax` at `t = 0` (a source far from a fixed origin layout would
> otherwise leave every robot **uninformed** — no source signal, no localization). At the paper
> source `[5.5, 5.5]` the layout is bit-identical to the original. Explicit `positions` opt out
> of this and are used verbatim.

| Field | Meaning |
|---|---|
| `positions` | List of `n` `[x, y]` coordinates (meters). Must have exactly `n` rows. |
| `headings` | List of `n` initial headings (radians). Honoured by paths with robot orientation (`coppelia`, MATLAB TurtleBot); ignored by the point-robot single-integrator paths. |
| `informed` | List of `n` flags, `1` = sensing-capable, `0` = forced blind. Optional; absent ⇒ all `1`. See "Informed vs. uninformed robots" below. |

```json
"initial_conditions": {
  "positions": [[0, 0], [2.5, -0.5], [5, 0], [0.5, 3.5], [3, 4], [5.5, 3]],
  "headings":  [0, 0, 0, 0, 0, 0],
  "informed":  [1, 1, 1, 1, 1, 1]
}
```

#### Informed vs. uninformed robots

The paper splits robots into **informed** (within sensing range `Dmax` of the source — they
measure the true field `f(p_i) + noise`) and **blind/uninformed** (outside range — they get
only the constant saturation value `f_Dmax = kappa*Dmax^2 + delta`). This is computed
**automatically every step from each robot's distance to the source**, so a robot that leaves
the sensing radius becomes uninformed on its own.

The optional `informed` mask adds a **user override on top of that rule**: a robot flagged `0`
is treated as blind even while inside `Dmax` (e.g. a failed/disabled sensor). The effective
status is `informed[i] == 1 AND ||p_i - source|| < Dmax`. Honoured by all paths
(Python single-integrator/unicycle, `coppelia`, MATLAB `matlab/` and `matlab_turtlebot/`);
the generated Simulink model uses all-informed only.

Theory notes surfaced in `summary.json` (`validation` block): `min_n_informed`,
`min_informed_for_valid_bound` (always 1 — the bound is valid for any `n_informed ≥ 1`),
`epsilon_all_informed` (the Remark-4 best case `delta/(kappa*R)`), and
`epsilon_inflation_factor`. The practical rule of thumb: **at least one informed robot is
required** for localization (with zero, there is no source signal and the centroid cannot
converge); and as the informed *fraction* shrinks, `epsilon` inflates (valid but loose).
Formation (the circle shape) still forms from the consensus term regardless of informed count.

Honoured by Python `sgf_sim` (`run-config`), the `coppelia` package (mock + physics), and
both MATLAB paths (`matlab/`, `matlab_turtlebot/`). The shape is validated against `n`; a
mismatched row count raises a clear error. **This section is also the mechanism for running
`n ≠ 6`** — see "Changing the number of robots" below.

### `robot_model`

Holds robot-model parameters for later phases.

| Field | Meaning |
|---|---|
| `type` | Robot model type for the selected mode. |
| `unicycle_shift_r` | Feedback-linearization offset distance. Must be positive for unicycle/TurtleBot modes. |
| `max_linear_velocity` | Optional command saturation. |
| `max_angular_velocity` | Optional command saturation. |
| `wheel_radius` | Wheel radius for differential-drive conversion (meters). |
| `wheel_base` | Wheel separation / track width (meters). The `coppelia` loader reads the key `wheel_base` (not `wheel_separation`). Defaults are TurtleBot3 Burger values (`0.033`, `0.16`); set them to match your actual robot model — e.g. the Pioneer p3dx used by the default CoppeliaSim scene has different geometry. |
| `command_period` | Sampled command period, matching the paper's Section V value when set to `0.1`. |

### `controller`

Optional. Controls the formation-term signum shaping. Omit the whole section for
the paper's exact behaviour.

| Field | Meaning |
|---|---|
| `sign_boundary_layer` | Boundary-layer width `eps` for the formation term. `0` (default / absent) uses the paper's exact component-wise `sgn`. `> 0` replaces `sgn(x)` with `sat(x/eps) = clip(x/eps, -1, 1)`, a sliding-mode boundary layer that removes the chattering which otherwise corrupts localization on unicycle / TurtleBot robots. The TurtleBot presets use `0.2`; `0` keeps MATLAB↔Python parity bit-identical. |

Honoured by both the Python (`SimulationConfig.sign_boundary_layer`, shared with
the unicycle) and MATLAB (`cfg.sign_boundary_layer`, read by `sgf_control`)
control laws.

### `outputs`

Controls artifact generation.

| Field | Meaning |
|---|---|
| `folder` | Base output folder. |
| `run_id` | Output subfolder name. |
| `save_plots` | Save static plots. Currently Python always saves plots for `run-config`. |
| `save_report` | Save `validation_report.md`. |
| `save_animation` | Request GIF or MP4 export. Planned for later phases. |
| `animation_format` | `gif` or `mp4`. |
| `animation_fps` | Frames per second for animation export. |
| `show_error_panels` | Include formation and localization error panels in animation. |
| `save_telemetry` | (`coppelia` path) Save full per-step telemetry to `telemetry.npz` + `telemetry.csv` for offline analysis. Defaults to `true`; disable on the CLI with `--no-telemetry`. |

### `coppelia`

Backend-specific settings read only by the `coppelia` package (all optional):

| Field | Meaning |
|---|---|
| `backend` | `mock` (offline kinematic stand-in) or `coppelia` (live physics via the ZeroMQ remote API). CLI `--backend` overrides. |
| `host` / `port` | CoppeliaSim ZeroMQ remote-API endpoint (defaults `localhost` / `23000`). |
| `stepped` | Use CoppeliaSim stepped (synchronous) mode. |
| `robot_model` | Scene-builder model key (`pioneer`, `dr12`). |
| `floor_scale` | Isometric (uniform x/y/z) scale applied to the scene floor when building the CoppeliaSim scene. Defaults to `7.0` (a 5 m floor → 35 m) so the paper layout and localization drift stay on the floor. CLI `--floor-scale`. Also accepted under a top-level `scene.floor_scale`. |

## Python Usage

```powershell
python -m sgf_sim run-config configs/paper_default.json
python -m sgf_sim run-config configs/paper_bounded_validation.json
```

Override output folder:

```powershell
python -m sgf_sim run-config configs/paper_default.json --output-dir outputs/config_runs
```

Phase 2 unicycle model — both signum modes are available:

```powershell
# original paper controller (exact sgn) -- localization stalls outside the bound
python -m sgf_sim unicycle validate --duration 90 --dt 0.004

# chattering-free boundary layer -- localizes inside the epsilon bound
python -m sgf_sim unicycle validate --duration 90 --dt 0.004 --sign-boundary-layer 0.2
```

## MATLAB Usage Planned

```matlab
raw = jsondecode(fileread('configs/paper_default.json'));
run_from_config('configs/paper_default.json')
```

MATLAB implementation should preserve the same section names and field meanings.

## Changing the number of robots `n`

The default is `n = 6` (paper Section IV). Valid range is `n ≥ 3` (`validate()` requires
`n > 2`). Most of the machinery scales automatically, but four things are pinned to 6 and
must be handled when you change `n`. Here is exactly what to do.

**Scales automatically — no action needed:**

- **Formation target circle** — slots are `θ_i = 2π i / n`, `φ_i = (cos θ_i, sin θ_i)`,
  generated from `n` in every path.
- **`ring` and `complete` topologies** — rebuilt from `n` for any size.
- **Theory bounds** — `gain_threshold = 4·n·f_Dmax/R` and the `epsilon` bound both take `n`
  and `n_informed` directly; the all-informed case reduces to Remark 4 `δ/(κR)` for any `n`.
- **Runtime arrays, centroid, error series, `n_informed`** — all sized from `n`.
- **Plots / animation** — per-robot loops are dynamic. (Cosmetic only: with `n > 10`, trail
  colors from the default cycle repeat and the legend gets crowded — not an error.)
- **CoppeliaSim scene** — the builder loads `n` robot models automatically.

**Manual edits required:**

1. **Set `paper_parameters.n`** in every config you run.
2. **Do not use `topology.name = "paper_fig1_reconstructed"` for `n ≠ 6`.** That preset is a
   fixed 6-node, 8-edge graph and **raises an error** for any other `n` (in Python, MATLAB,
   and coppelia). Switch to `ring`, `complete`, or `default`, or supply your own
   `adjacency` / `edges` for the new size.
3. **Fix the `topology.edges` list.** The shipped configs also carry an explicit 6-node
   `edges` array. Because Python/MATLAB prefer `edges` over `name` (see the loader-divergence
   note above), leaving the old list in place builds a wrong/broken graph for `n ≠ 6`
   (it throws for `n < 6` and leaves nodes ≥ 6 disconnected for `n > 6`). Replace it with an
   edge list for the new `n`, or null it out and rely on a scalable `name`.
4. **Provide `initial_conditions.positions` for the new `n`** (and `headings` for
   orientation-aware paths). The built-in default layout is defined only for `n = 6` and the
   MATLAB paths hard-error otherwise. This is the whole reason the `initial_conditions`
   section exists — with it, `n ≠ 6` works from the config file alone, no code edit.
5. **CLI note:** the Python `sgf_sim` CLI has no `--n` flag, so change `n` via a JSON config
   and `run-config` (not the bare `run`/`validate` subcommands).
6. **Tests:** a few fixtures assume the 6-robot default (`tests/test_theory.py`,
   `tests/test_unicycle.py`, `coppelia/tests/…`) — update them only if you change the
   *default* `n` in code, not for a one-off config run.

**Minimal example — an 8-robot ring:**

```json
{
  "paper_parameters": { "n": 8, "source": [5.5, 5.5], "kappa": 1.0, "R": 2.0, "Dmax": 12.0, "alpha": 100.0, "beta": 0.05 },
  "topology": { "name": "ring", "adjacency": null, "edges": null },
  "initial_conditions": {
    "positions": [[0,0],[1,0],[2,0],[3,0],[0,3],[1,3],[2,3],[3,3]]
  }
}
```

(Include the other sections — `experiment`, `noise`, `outputs` — as usual.) With a
scalable topology (`ring`), custom positions, and `edges` nulled out, this runs unchanged
through `python -m sgf_sim run-config`, the MATLAB `run_from_config`, and the `coppelia`
package.

## Editing Rules

- Change parameters in config files instead of hardcoding experiment values.
- Keep units in SI units unless documented otherwise.
- Keep graph indices zero-based in JSON so Python and MATLAB outputs stay comparable.
- When MATLAB uses one-based arrays internally, convert indices at the parser boundary.
- Keep mode-specific fields present even if the current runner ignores them, so future phases do not need a new config format.
