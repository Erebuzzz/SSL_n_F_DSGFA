# Shared Config Guide

The shared config files in `configs/` are the source of truth for reproducible experiments across Python and future MATLAB or Simulink implementations.

The format is JSON because Python can read it with the standard `json` module and MATLAB can read it with `jsondecode`.

## Files

| File | Purpose |
|---|---|
| `configs/paper_default.json` | Paper-like Gaussian single-integrator run. |
| `configs/paper_bounded_validation.json` | Theorem-valid bounded-noise single-integrator run. |
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

Only `single_integrator` is implemented in Python right now. Other modes are present so future MATLAB, Simulink, unicycle, TurtleBot, and CoppeliaSim phases use the same config shape.

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

The gain ratio `alpha / beta` should be compared against `4 * n * f_Dmax / R`.

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

### `robot_model`

Holds robot-model parameters for later phases.

| Field | Meaning |
|---|---|
| `type` | Robot model type for the selected mode. |
| `unicycle_shift_r` | Feedback-linearization offset distance. Must be positive for unicycle/TurtleBot modes. |
| `max_linear_velocity` | Optional command saturation. |
| `max_angular_velocity` | Optional command saturation. |
| `wheel_radius` | TurtleBot wheel radius for differential-drive conversion. |
| `wheel_separation` | TurtleBot wheel separation. |
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

## Editing Rules

- Change parameters in config files instead of hardcoding experiment values.
- Keep units in SI units unless documented otherwise.
- Keep graph indices zero-based in JSON so Python and MATLAB outputs stay comparable.
- When MATLAB uses one-based arrays internally, convert indices at the parser boundary.
- Keep mode-specific fields present even if the current runner ignores them, so future phases do not need a new config format.
