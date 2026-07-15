# Unified Launcher

One window to configure and run any mode of the sign gradient-free simulator, with a live
theory readout (gain condition, epsilon bound, informed-robot diagnostics) shown **before**
you run.

```powershell
python -m launcher
```

## What you can set

- **Platform** — Python, MATLAB, or CoppeliaSim.
- **Mode** — auto-filtered to what the platform supports:
  - Python → `single_integrator`, `unicycle`
  - MATLAB → `single_integrator`, `turtlebot_numeric`, `turtlebot_simulink`
  - CoppeliaSim → differential-drive (choose `mock` or `coppelia` backend)
- **Source** location, **R**, **Dmax**, **alpha/beta**, **kappa**, **dt**, **duration**, **seed**.
- **Noise** — none / gaussian / bounded (+ std / bound).
- **Sgn controller** — `exact` (paper `sgn`) or `boundary_layer` (chatter-free `sat`).
- **Per-robot table** — each robot's initial `x, y` and an `informed` checkbox. Defaults are
  filled in; edit any cell. Press **apply n** to change the robot count and regenerate the grid.
  While you haven't hand-edited a position, the default layout **follows the source**: changing
  the source slides the whole formation so every robot keeps starting within `Dmax` (editing any
  cell, or using a custom layout, opts out — then a warning fires if that leaves nobody informed).

## Live readout

As you edit, the right panel recomputes (no simulation needed):

- gain ratio vs. threshold `4 n f_Dmax / R` and whether the sufficient condition passes;
- `epsilon` for the all-informed case and for the number of robots that **start** informed
  (within `Dmax` and flagged sensing-capable), with the inflation factor between them;
- the minimum informed count for a valid bound, and warnings when zero robots start informed.

## Run / dispatch

**Run** executes on the chosen platform:

- **Python** (`single_integrator`, `unicycle`) — runs in-process; writes plots (including the
  paper Fig. 3 per-robot formation error) and `summary.json`.
- **CoppeliaSim** — `mock` backend runs offline in-process; `coppelia` backend drives a live
  CoppeliaSim (install `coppeliasim-zmqremoteapi-client` and have the simulator open on an
  empty scene — see [docs/COPPELIASIM_SETUP_GUIDE.md](../docs/COPPELIASIM_SETUP_GUIDE.md)).
- **MATLAB** — writes a shared JSON config and shells out to
  `matlab -batch "... run_from_config('cfg.json')"` (or `run_turtlebot_from_config`). If
  `matlab` is not on `PATH`, the exact command is shown so you can run it yourself.

## Architecture

- [`launcher/core.py`](core.py) — headless: config state, the theory readout, shared-JSON
  serialization, and dispatch. Depends only on numpy + the project packages, so it is fully
  unit-tested (`tests/test_launcher.py`) without a display.
- [`launcher/gui.py`](gui.py) — thin tkinter layer that reads/writes the state and calls core.
