# CoppeliaSim scene

Phase 3 builds its scene **programmatically** through the ZeroMQ remote API rather
than shipping a binary `.ttt` file, so the whole build stays text-based and
reviewable and the robot count/layout follows the config.

## What `build_scene(sim, config)` does

1. Resolves the robot model path inside the CoppeliaSim install `models` folder
   (default key `pioneer` → `models/robots/mobile/pioneer p3dx.ttm`).
2. Loads the model `config.n` times with `sim.loadModel`, renames each base to
   `robot[i]`, and places it at the configured initial position/heading.
3. Finds each robot's left/right drive joints by walking the model subtree
   (`sim.getObjectsInTree(..., sim.object_joint_type)`) and matching the alias
   substrings `left` / `right`, so it is robust to exact model naming.
4. Adds a small red, non-respondable sphere as the **source marker** at
   `config.source`.
5. Isometrically (uniform x/y/z) scales the scene **floor** by `config.floor_scale`
   (default `7.0`). A new CoppeliaSim scene ships a 5 m × 5 m floor, which the
   paper layout and the localization drift quickly overrun; scaling it to
   ~35 m × 35 m keeps every robot on the floor. Scaling is best-effort: if the
   scene has no floor object the step is skipped and the run continues.
6. Draws concentric **field contour rings** on the floor around the source
   (`sim.drawing_lines`) at radii bracketing the initial swarm spread. The scalar
   field `f(z) = kappa*||z - p_s||^2` has circular level sets, so the rings show the
   "bowl" the swarm descends toward the source. Best-effort like the floor scaling.
7. Returns the handles the backend needs:

   ```python
   {"robots": [...], "left_motors": [...], "right_motors": [...],
    "source": handle, "floor": handle_or_None, "field_contours": handle_or_None}
   ```

The backend then drives each robot by converting the body command `(v, ω)` into
left/right wheel angular velocities (`coppelia.unicycle.unicycle_to_wheel_speeds`)
using `config.wheel_radius` and `config.wheel_base`, and calling
`sim.setJointTargetVelocity`.

## Choosing a robot model

Set `config.robot_model` (or extend `ROBOT_MODEL_PATHS` in `build_scene.py`):

| Key | Model |
|---|---|
| `pioneer` | `robots/mobile/pioneer p3dx.ttm` (default, classic diff-drive) |
| `dr12` | `robots/mobile/dr12 (Dr Robot).ttm` |

A TurtleBot3 model can be added the same way if you install its `.ttm`; update
`wheel_radius` / `wheel_base` to match (TurtleBot3 Burger defaults are already the
config defaults: 0.033 m / 0.16 m).

## Manual scene alternative

If you prefer to build the scene by hand in the GUI:

1. Place `n` differential-drive robots and name their bases `robot[0]` …
   `robot[n-1]`, with child motors resolvable as `/robot[i]/leftMotor` and
   `/robot[i]/rightMotor` (matches `ZmqBackend.*_alias_template`).
2. Save the scene.
3. Run with scene-building disabled by constructing `ZmqBackend(config,
   build_scene=False)` (the backend then discovers the existing handles by alias).

## Verifying against the real simulator

This scene code needs a running CoppeliaSim and therefore is not covered by the
offline test-suite. To validate:

```powershell
python -m pip install coppeliasim-zmqremoteapi-client
# start CoppeliaSim with an empty scene, then:
python -m coppelia run --backend coppelia --run-id coppelia_check --duration 40
```

Watch the robots gather into the circular formation and the centroid drift toward
the red source marker; compare `outputs/coppelia/coppelia_check/summary.json`
against a `--backend mock` run with the same parameters.
