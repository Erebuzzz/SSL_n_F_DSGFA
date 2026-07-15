"""Generate the 24-case batch config matrix for the MATLAB report.

This report deliberately exercises the paper's **uninformed-robot** machinery: a
fraction of the swarm starts OUTSIDE the sensing radius (``||p_i - source|| >
Dmax``) and therefore measures only the saturation value ``f_Dmax = kappa*Dmax^2
+ delta`` (Eq. 2), while the rest measure the true noisy field. The blind robots
are created purely by initial *distance* (not the ``informed`` mask) so the split
is identical in the single-integrator path and the Simulink ODE, which honours
only the ``dist < Dmax`` rule.

Matrix (24 distinct cases):
  2 modes x 3 sizes x 2 noise x 2 sources
    modes  : single_integrator, turtlebot_simulink
    sizes  : n = 4 (1 uninformed / 3 informed),
             n = 6 (2 / 4),
             n = 8 (3 / 5)
    noise  : none, gaussian (N(0, 0.2), paper-faithful)
    sources: A = [5.5, 5.5] (paper), B = [30.0, -20.0] (far-shifted)

turtlebot_simulink now injects real seeded Gaussian measurement noise (see
matlab_turtlebot/build_turtlebot_simulink_model.m), so its noisy arm is a
genuinely distinct run, not a duplicate of the noise-free twin.
"""

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
CFG_DIR = HERE / "configs"

SOURCES = {"srcA": [5.5, 5.5], "srcB": [30.0, -20.0]}

# Runs that also export a motion.gif (>= one-third of the 24-case batch = 8 runs).
# Chosen for coverage: both modes x all three sizes at source A with gaussian
# noise, plus one noise-free n=6 per mode. Keep this set in sync with whatever
# patterns are re-run through run_batch so every animated config gets its gif.
ANIMATE_RUNS = {
    "si_n4_srcA_gaussian", "si_n6_srcA_gaussian", "si_n8_srcA_gaussian",
    "ts_n4_srcA_gaussian", "ts_n6_srcA_gaussian", "ts_n8_srcA_gaussian",
    "si_n6_srcA_none", "ts_n6_srcA_none",
}

# Number of uninformed (out-of-range) robots per swarm size. Informed = n - k.
N_UNINFORMED = {4: 1, 6: 2, 8: 3}

# Radial placement (metres from the source), spread over a full ring of angles.
# Informed robots sit well inside Dmax = 12; uninformed robots sit outside it.
INFORMED_RADII = (5.0, 10.0)     # linspace start/stop
UNINFORMED_RADII = (14.0, 17.0)  # all > Dmax = 12, so blind at t = 0


def _linspace(a, b, count):
    if count <= 1:
        return [a]
    step = (b - a) / (count - 1)
    return [a + step * i for i in range(count)]


def positions_for(n, source):
    """n starting positions on a ring: first (n-k) informed (inside Dmax), then
    k uninformed (outside Dmax). Layout is source-relative, so both sources give
    the same informed/uninformed geometry and the same initial errors."""
    k = N_UNINFORMED[n]
    informed = n - k
    ri = _linspace(*INFORMED_RADII, informed)
    ru = _linspace(*UNINFORMED_RADII, k)
    angles = [2.0 * math.pi * i / n for i in range(n)]
    pos = []
    for i in range(informed):
        r = ri[i]
        pos.append([round(source[0] + r * math.cos(angles[i]), 4),
                    round(source[1] + r * math.sin(angles[i]), 4)])
    for j in range(k):
        idx = informed + j
        r = ru[j]
        pos.append([round(source[0] + r * math.cos(angles[idx]), 4),
                    round(source[1] + r * math.sin(angles[idx]), 4)])
    return pos


def _noise_block(noise):
    # "gaussian" -> paper N(0, 0.2); "none" -> deterministic. bound stays at the
    # theorem's delta = 0.2 so f_Dmax and the epsilon reference are unchanged.
    std = 0.2 if noise == "gaussian" else 0.0
    return {"model": noise, "std": std, "bound": 0.2}


def si_config(n, src_key, source, noise):
    run_id = f"si_n{n}_{src_key}_{noise}"
    return run_id, {
        "experiment": {"name": run_id, "mode": "single_integrator",
                       "seed": 1, "duration": 60.0, "dt": 0.0005},
        "paper_parameters": {"n": n, "source": source, "kappa": 1.0,
                             "R": 2.0, "Dmax": 12.0, "alpha": 100.0, "beta": 0.05},
        "noise": _noise_block(noise),
        "topology": {"name": "ring", "adjacency": None, "edges": None},
        "robot_model": {"type": "single_integrator", "unicycle_shift_r": 0.025,
                        "max_linear_velocity": None, "max_angular_velocity": None},
        "initial_conditions": {"positions": positions_for(n, source),
                               "informed": [1] * n},
        "outputs": {"folder": "outputs/batch_report/runs", "run_id": run_id,
                    "save_plots": True, "save_report": True,
                    "save_animation": run_id in ANIMATE_RUNS,
                    "animation_format": "gif",
                    "animation_fps": 20, "show_error_panels": True},
    }


# Formation-gain sweep: n=6, source A, noise-free, beta fixed at 0.05, only alpha
# varies. Isolates the effect of the formation gain on the formation timescale
# (finite-time term) while everything else is held identical.
FORMATION_GAIN_SWEEP = [1.0, 5.0, 20.0, 50.0, 100.0]

# Extreme localization-gain probe (Simulink): beta = 10000, alpha = 10^0/10^1/10^2.
BETA_PROBE_ALPHAS = [1.0, 10.0, 100.0]


def fg_config(alpha):
    run_id = f"fg_n6_a{int(alpha)}_srcA_none"
    source = SOURCES["srcA"]
    return run_id, {
        "experiment": {"name": run_id, "mode": "single_integrator",
                       "seed": 1, "duration": 60.0, "dt": 0.0005},
        "paper_parameters": {"n": 6, "source": source, "kappa": 1.0,
                             "R": 2.0, "Dmax": 12.0, "alpha": alpha, "beta": 0.05},
        "noise": _noise_block("none"),
        "topology": {"name": "ring", "adjacency": None, "edges": None},
        "robot_model": {"type": "single_integrator", "unicycle_shift_r": 0.025,
                        "max_linear_velocity": None, "max_angular_velocity": None},
        "initial_conditions": {"positions": positions_for(6, source),
                               "informed": [1] * 6},
        "outputs": {"folder": "outputs/batch_report/runs", "run_id": run_id,
                    "save_plots": True, "save_report": True,
                    "save_animation": True, "animation_format": "gif",
                    "animation_fps": 20, "show_error_panels": True},
    }


# TurtleBot-Simulink formation-gain sweep: same 5 alpha values as the single-
# integrator sweep, run through the differential-drive Simulink model (beta fixed
# at 0.05, n=6, source A, noise-free, 2-uninformed layout). Plots + gif saved.
def fgt_config(alpha):
    run_id = f"fgt_n6_a{int(alpha)}_srcA_none"
    source = SOURCES["srcA"]
    return run_id, {
        "experiment": {"name": run_id, "mode": "turtlebot_simulink",
                       "seed": 1, "duration": 90.0, "dt": 0.004},
        "paper_parameters": {"n": 6, "source": source, "kappa": 1.0,
                             "R": 2.0, "Dmax": 12.0, "alpha": alpha, "beta": 0.05},
        "noise": _noise_block("none"),
        "topology": {"name": "ring", "adjacency": None, "edges": None},
        "robot_model": {"type": "turtlebot_simulink", "unicycle_shift_r": 2.0,
                        "wheel_radius": 0.033, "wheel_separation": 0.16,
                        "max_linear_velocity": None, "max_angular_velocity": None,
                        "command_period": 0.0},
        "controller": {"sign_boundary_layer": 0.2},
        "simulink": {"solver": "ode4", "model_name": "sgf_turtlebot_swarm"},
        "initial_conditions": {"positions": positions_for(6, source),
                               "headings": [0.0] * 6, "informed": [1] * 6},
        "outputs": {"folder": "outputs/batch_report/runs", "run_id": run_id,
                    "save_plots": True, "save_report": True,
                    "save_animation": True, "animation_format": "gif",
                    "animation_fps": 20},
    }


# Extreme localization-gain probe: beta held at 10000 (astronomically large vs
# the paper's 0.05) while alpha = 1, 10, 100. Run in Simulink to see what a
# localization-dominant gain balance does. Plots + summary saved (animation off:
# these are expected to be numerically violent, so a gif frame may be non-finite).
def fgb_config(alpha):
    run_id = f"fgb_n6_a{int(alpha)}_srcA_none"
    source = SOURCES["srcA"]
    return run_id, {
        "experiment": {"name": run_id, "mode": "turtlebot_simulink",
                       "seed": 1, "duration": 90.0, "dt": 0.004},
        "paper_parameters": {"n": 6, "source": source, "kappa": 1.0,
                             "R": 2.0, "Dmax": 12.0, "alpha": alpha, "beta": 10000.0},
        "noise": _noise_block("none"),
        "topology": {"name": "ring", "adjacency": None, "edges": None},
        "robot_model": {"type": "turtlebot_simulink", "unicycle_shift_r": 2.0,
                        "wheel_radius": 0.033, "wheel_separation": 0.16,
                        "max_linear_velocity": None, "max_angular_velocity": None,
                        "command_period": 0.0},
        "controller": {"sign_boundary_layer": 0.2},
        "simulink": {"solver": "ode4", "model_name": "sgf_turtlebot_swarm"},
        "initial_conditions": {"positions": positions_for(6, source),
                               "headings": [0.0] * 6, "informed": [1] * 6},
        "outputs": {"folder": "outputs/batch_report/runs", "run_id": run_id,
                    "save_plots": True, "save_report": True,
                    "save_animation": False, "animation_format": "gif",
                    "animation_fps": 20},
    }


def ts_config(n, src_key, source, noise):
    run_id = f"ts_n{n}_{src_key}_{noise}"
    return run_id, {
        "experiment": {"name": run_id, "mode": "turtlebot_simulink",
                       "seed": 1, "duration": 90.0, "dt": 0.004},
        "paper_parameters": {"n": n, "source": source, "kappa": 1.0,
                             "R": 2.0, "Dmax": 12.0, "alpha": 10.0, "beta": 0.05},
        "noise": _noise_block(noise),
        "topology": {"name": "ring", "adjacency": None, "edges": None},
        "robot_model": {"type": "turtlebot_simulink", "unicycle_shift_r": 2.0,
                        "wheel_radius": 0.033, "wheel_separation": 0.16,
                        "max_linear_velocity": None, "max_angular_velocity": None,
                        "command_period": 0.0},
        "controller": {"sign_boundary_layer": 0.2},
        "simulink": {"solver": "ode4", "model_name": "sgf_turtlebot_swarm"},
        "initial_conditions": {"positions": positions_for(n, source),
                               "headings": [0.0] * n, "informed": [1] * n},
        "outputs": {"folder": "outputs/batch_report/runs", "run_id": run_id,
                    "save_plots": True, "save_report": True,
                    "save_animation": run_id in ANIMATE_RUNS,
                    "animation_format": "gif",
                    "animation_fps": 20},
    }


def main():
    CFG_DIR.mkdir(parents=True, exist_ok=True)
    written = []
    for n in (4, 6, 8):
        for src_key, source in SOURCES.items():
            for noise in ("none", "gaussian"):
                run_id, cfg = si_config(n, src_key, source, noise)
                (CFG_DIR / f"{run_id}.json").write_text(json.dumps(cfg, indent=2))
                written.append(run_id)
    for n in (4, 6, 8):
        for src_key, source in SOURCES.items():
            for noise in ("none", "gaussian"):
                run_id, cfg = ts_config(n, src_key, source, noise)
                (CFG_DIR / f"{run_id}.json").write_text(json.dumps(cfg, indent=2))
                written.append(run_id)
    for alpha in FORMATION_GAIN_SWEEP:
        run_id, cfg = fg_config(alpha)
        (CFG_DIR / f"{run_id}.json").write_text(json.dumps(cfg, indent=2))
        written.append(run_id)
    for alpha in FORMATION_GAIN_SWEEP:
        run_id, cfg = fgt_config(alpha)
        (CFG_DIR / f"{run_id}.json").write_text(json.dumps(cfg, indent=2))
        written.append(run_id)
    for alpha in BETA_PROBE_ALPHAS:
        run_id, cfg = fgb_config(alpha)
        (CFG_DIR / f"{run_id}.json").write_text(json.dumps(cfg, indent=2))
        written.append(run_id)
    print(f"wrote {len(written)} configs to {CFG_DIR}")
    for w in written:
        print(" ", w)


if __name__ == "__main__":
    main()
