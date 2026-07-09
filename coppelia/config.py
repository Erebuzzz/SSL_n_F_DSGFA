"""Configuration for the Phase 3 CoppeliaSim multi-robot build.

The :class:`CoppeliaConfig` dataclass carries every parameter needed for one
CoppeliaSim (or mock) experiment. It mirrors the paper's Section IV / Section V
parameters and adds robot-model, backend, and output settings.

It can be constructed directly, or loaded from the Phase 1.25 shared JSON config
format (see ``ROADMAP.md``) via :meth:`CoppeliaConfig.from_json`. Loading tolerates
missing sections so a minimal JSON file still works.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]
IntArray = NDArray[np.int_]


def default_initial_positions() -> FloatArray:
    """Deterministic six-robot starting layout near the paper source.

    Matches the Phase 1 layout (``sgf_sim.config.default_initial_positions``) so
    CoppeliaSim runs are visually comparable to the numerical baseline.
    """

    return np.array(
        [
            [0.0, 0.0],
            [2.5, -0.5],
            [5.0, 0.0],
            [0.5, 3.5],
            [3.0, 4.0],
            [5.5, 3.0],
        ],
        dtype=float,
    )


def default_initial_headings(n: int) -> FloatArray:
    """Default heading for each robot: face roughly toward the source."""

    return np.zeros(n, dtype=float)


@dataclass(frozen=True)
class CoppeliaConfig:
    """Parameters for one Phase 3 CoppeliaSim / mock experiment."""

    # --- paper Section IV parameters (single-integrator control law) ---
    # NOTE on gains: the Phase 1 numerical baseline uses alpha=100, beta=0.05
    # (ratio 2000) to satisfy Theorem 1's conservative sufficient condition
    # (alpha/beta > ~1730). Those gains command control-point speeds of hundreds
    # of m/s, which is fine for a point integrator but breaks the unicycle
    # feedback linearization under any physical angular-velocity/step budget.
    # Phase 3 therefore defaults to physically-sane gains (ratio 200): formation
    # still converges quickly and the centroid localizes toward the source,
    # though not as tightly as the ideal single integrator. See coppelia/README.md
    # ("Theory vs. physical unicycle") for the full discussion.
    n: int = 6
    source: tuple[float, float] = (5.5, 5.5)
    kappa: float = 1.0
    radius: float = 2.0
    dmax: float = 12.0
    alpha: float = 10.0
    beta: float = 0.05

    # --- integration / horizon ---
    duration: float = 90.0
    dt: float = 0.004
    seed: int = 1

    # --- noise ---
    noise_model: str = "gaussian"  # one of: none, gaussian, bounded
    noise_std: float = 0.2
    noise_bound: float = 0.2

    # --- topology ---
    topology: str = "paper_fig1_reconstructed"
    adjacency: IntArray | None = None

    # --- unicycle / differential-drive robot model (Section V) ---
    # A larger control-point offset r keeps the feedback-linearization angular
    # velocity (~|f|/r) manageable. Velocity limits default to None (disabled):
    # enabling them models real actuator saturation but, with these gains, starves
    # the slow localization term of velocity budget (a documented Phase 3 finding).
    control_point_offset: float = 0.5  # r > 0, feedback-linearization shift [m]
    # Boundary-layer width for the formation sgn term. 0 (default) = exact paper
    # sgn; > 0 uses sat(x/eps) = clip(x/eps, -1, 1), which removes the chattering
    # that corrupts localization on these differential-drive robots (matches the
    # sgf_sim / MATLAB sign_boundary_layer option).
    sign_boundary_layer: float = 0.0
    max_linear_velocity: float | None = None  # [m/s]; None disables the limit
    max_angular_velocity: float | None = None  # [rad/s]; None disables the limit
    wheel_radius: float = 0.033  # TurtleBot3 Burger wheel radius [m]
    wheel_base: float = 0.16  # TurtleBot3 wheel separation [m]

    # --- initial conditions ---
    initial_positions: FloatArray | None = None
    initial_headings: FloatArray | None = None

    # --- backend selection ---
    backend: str = "mock"  # one of: mock, coppelia
    coppelia_host: str = "localhost"
    coppelia_port: int = 23000
    stepped: bool = True  # use CoppeliaSim stepped (synchronous) mode
    robot_model: str = "pioneer"  # scene-builder robot model key

    # --- outputs ---
    output_dir: Path = field(default_factory=lambda: Path("outputs") / "coppelia")
    save_plots: bool = True
    save_animation: bool = True
    animation_format: str = "gif"  # gif or mp4
    animation_fps: int = 20
    animation_max_frames: int = 240

    # ------------------------------------------------------------------
    # resolution helpers
    # ------------------------------------------------------------------
    def resolved_initial_positions(self) -> FloatArray:
        if self.initial_positions is None:
            positions = default_initial_positions()
        else:
            positions = np.asarray(self.initial_positions, dtype=float)
        if positions.shape != (self.n, 2):
            raise ValueError(f"initial_positions must have shape {(self.n, 2)}")
        return positions.copy()

    def resolved_initial_headings(self) -> FloatArray:
        if self.initial_headings is None:
            headings = default_initial_headings(self.n)
        else:
            headings = np.asarray(self.initial_headings, dtype=float)
        if headings.shape != (self.n,):
            raise ValueError(f"initial_headings must have shape {(self.n,)}")
        return headings.copy()

    def resolved_adjacency(self) -> IntArray:
        from .topology import adjacency_for_topology, is_connected

        if self.adjacency is None:
            adjacency = adjacency_for_topology(self.topology, self.n)
        else:
            adjacency = np.asarray(self.adjacency, dtype=int)
        if adjacency.shape != (self.n, self.n):
            raise ValueError(f"adjacency must have shape {(self.n, self.n)}")
        if np.any(np.diag(adjacency) != 0):
            raise ValueError("adjacency diagonal must be zero")
        if not np.array_equal(adjacency, adjacency.T):
            raise ValueError("phase 3 requires an undirected adjacency matrix")
        if not is_connected(adjacency):
            raise ValueError("adjacency must be connected")
        return adjacency.copy()

    def source_array(self) -> FloatArray:
        return np.asarray(self.source, dtype=float)

    def validate(self) -> None:
        if self.n <= 2:
            raise ValueError("the paper identities require n > 2")
        if self.radius <= 0:
            raise ValueError("radius must be positive")
        if self.radius > self.dmax:
            raise ValueError("radius must satisfy R <= Dmax")
        if self.dmax <= 0:
            raise ValueError("dmax must be positive")
        if self.alpha <= 0 or self.beta <= 0:
            raise ValueError("alpha and beta must be positive")
        if self.duration <= 0 or self.dt <= 0:
            raise ValueError("duration and dt must be positive")
        if self.control_point_offset <= 0:
            raise ValueError("control_point_offset r must be positive (inversion is singular at r=0)")
        if self.sign_boundary_layer < 0:
            raise ValueError("sign_boundary_layer must be non-negative (0 = exact sgn)")
        if self.noise_model not in {"none", "gaussian", "bounded"}:
            raise ValueError("noise_model must be one of: none, gaussian, bounded")
        if self.noise_std < 0 or self.noise_bound < 0:
            raise ValueError("noise magnitudes must be non-negative")
        if self.backend not in {"mock", "coppelia"}:
            raise ValueError("backend must be one of: mock, coppelia")
        if self.animation_format not in {"gif", "mp4"}:
            raise ValueError("animation_format must be gif or mp4")
        self.resolved_initial_positions()
        self.resolved_initial_headings()
        self.resolved_adjacency()

    # ------------------------------------------------------------------
    # JSON interoperability (Phase 1.25 shared config schema, best-effort)
    # ------------------------------------------------------------------
    @classmethod
    def from_json(cls, path: str | Path) -> "CoppeliaConfig":
        """Load from the Phase 1.25 shared JSON schema (tolerant to missing keys)."""

        data: dict[str, Any] = json.loads(Path(path).read_text(encoding="utf-8"))
        experiment = data.get("experiment", {})
        params = data.get("paper_parameters", {})
        noise = data.get("noise", {})
        topo = data.get("topology", {})
        robot = data.get("robot_model", {})
        outputs = data.get("outputs", {})
        coppelia = data.get("coppelia", {})

        kwargs: dict[str, Any] = {}

        def put(key: str, value: Any) -> None:
            if value is not None:
                kwargs[key] = value

        put("seed", experiment.get("seed"))
        put("duration", experiment.get("duration"))
        put("dt", experiment.get("dt"))

        put("n", params.get("n"))
        source = params.get("source")
        if source is not None:
            kwargs["source"] = tuple(source)
        put("kappa", params.get("kappa"))
        put("radius", params.get("R"))
        put("dmax", params.get("Dmax"))
        put("alpha", params.get("alpha"))
        put("beta", params.get("beta"))

        put("noise_model", noise.get("model"))
        put("noise_std", noise.get("std"))
        put("noise_bound", noise.get("bound"))

        put("topology", topo.get("name"))
        edges = topo.get("edges")
        adjacency = topo.get("adjacency")
        if adjacency is not None:
            kwargs["adjacency"] = np.asarray(adjacency, dtype=int)
        elif edges is not None and topo.get("name") in (None, "custom"):
            n = kwargs.get("n", cls.n)
            kwargs["adjacency"] = _adjacency_from_edge_list(n, edges)

        initial = data.get("initial_conditions", {})
        if isinstance(initial, dict):
            if initial.get("positions") is not None:
                kwargs["initial_positions"] = np.asarray(initial["positions"], dtype=float)
            if initial.get("headings") is not None:
                kwargs["initial_headings"] = np.asarray(initial["headings"], dtype=float)

        put("control_point_offset", robot.get("unicycle_shift_r"))
        controller = data.get("controller") if isinstance(data.get("controller"), dict) else {}
        put("sign_boundary_layer", controller.get("sign_boundary_layer"))
        put("max_linear_velocity", robot.get("max_linear_velocity"))
        put("max_angular_velocity", robot.get("max_angular_velocity"))
        put("wheel_radius", robot.get("wheel_radius"))
        put("wheel_base", robot.get("wheel_base"))

        folder = outputs.get("folder")
        if folder is not None:
            kwargs["output_dir"] = Path(folder)
        put("save_plots", outputs.get("save_plots"))
        put("save_animation", outputs.get("save_animation"))
        put("animation_format", outputs.get("animation_format"))
        put("animation_fps", outputs.get("animation_fps"))

        put("backend", coppelia.get("backend"))
        put("coppelia_host", coppelia.get("host"))
        put("coppelia_port", coppelia.get("port"))
        put("stepped", coppelia.get("stepped"))
        put("robot_model", coppelia.get("robot_model"))

        return cls(**kwargs)

    def to_dict(self) -> dict[str, Any]:
        """Serializable view of the config (arrays -> lists, Path -> str)."""

        data = asdict(self)
        data["output_dir"] = str(self.output_dir)
        for key in ("adjacency", "initial_positions", "initial_headings"):
            value = data.get(key)
            if isinstance(value, np.ndarray):
                data[key] = value.tolist()
        return data


def _adjacency_from_edge_list(n: int, edges: list[list[int]]) -> IntArray:
    adjacency = np.zeros((n, n), dtype=int)
    for edge in edges:
        i, j = int(edge[0]), int(edge[1])
        adjacency[i, j] = 1
        adjacency[j, i] = 1
    return adjacency
