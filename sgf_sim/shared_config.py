"""Shared JSON config loading for Python and future MATLAB parity."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .config import IntArray, SimulationConfig
from .topology import adjacency_from_edges

SUPPORTED_PYTHON_MODES = {"single_integrator"}
PLANNED_MODES = {"unicycle", "turtlebot_numeric", "turtlebot_simulink", "coppelia"}


@dataclass(frozen=True)
class SharedRunConfig:
    """Parsed shared config plus Python simulation config."""

    path: Path
    raw: dict[str, Any]
    simulation: SimulationConfig
    run_id: str
    save_plots: bool
    save_report: bool
    save_animation: bool
    mode: str


def load_shared_config(path: str | Path) -> SharedRunConfig:
    """Load a shared JSON config file and convert it for the Python simulator."""

    config_path = Path(path)
    raw = json.loads(config_path.read_text(encoding="utf-8"))
    experiment = _section(raw, "experiment")
    paper = _section(raw, "paper_parameters")
    noise = _section(raw, "noise")
    topology = _section(raw, "topology")
    outputs = _section(raw, "outputs")

    mode = str(experiment.get("mode", "single_integrator"))
    if mode not in SUPPORTED_PYTHON_MODES:
        if mode in PLANNED_MODES:
            raise NotImplementedError(
                f"mode '{mode}' is planned but not implemented in the Python runner yet"
            )
        raise ValueError(f"unsupported mode: {mode}")

    n = int(paper.get("n", 6))
    adjacency = _adjacency_from_topology_section(topology, n)
    controller = raw.get("controller") if isinstance(raw.get("controller"), dict) else {}
    initial_positions = _initial_positions_from_section(raw, n)
    informed = _informed_from_section(raw, n)
    simulation = SimulationConfig(
        n=n,
        source=tuple(float(x) for x in paper.get("source", [5.5, 5.5])),
        kappa=float(paper.get("kappa", 1.0)),
        radius=float(paper.get("R", paper.get("radius", 2.0))),
        dmax=float(paper.get("Dmax", paper.get("dmax", 12.0))),
        alpha=float(paper.get("alpha", 100.0)),
        beta=float(paper.get("beta", 0.05)),
        duration=float(experiment.get("duration", 60.0)),
        dt=float(experiment.get("dt", 0.0005)),
        seed=int(experiment.get("seed", 1)),
        noise_model=str(noise.get("model", "gaussian")),
        noise_std=float(noise.get("std", 0.2)),
        noise_bound=float(noise.get("bound", 0.2)),
        topology=str(topology.get("name", "paper_fig1_reconstructed")),
        sign_boundary_layer=float(controller.get("sign_boundary_layer", 0.0)),
        initial_positions=initial_positions,
        informed=informed,
        adjacency=adjacency,
        output_dir=Path(outputs.get("folder", Path("outputs") / "runs")),
    )
    run_id = str(outputs.get("run_id") or experiment.get("name") or config_path.stem)
    return SharedRunConfig(
        path=config_path,
        raw=raw,
        simulation=simulation,
        run_id=run_id,
        save_plots=bool(outputs.get("save_plots", True)),
        save_report=bool(outputs.get("save_report", True)),
        save_animation=bool(outputs.get("save_animation", False)),
        mode=mode,
    )


def _section(raw: dict[str, Any], key: str) -> dict[str, Any]:
    value = raw.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"config section '{key}' must be an object")
    return value


def _initial_positions_from_section(raw: dict[str, Any], n: int) -> np.ndarray | None:
    """Parse the optional ``initial_conditions.positions`` field.

    Absent section/field -> None, so the simulator falls back to the fixed
    ``default_initial_positions()`` layout (back-compatible). When present, it must
    be an ``(n, 2)`` list of ``[x, y]`` coordinates; the shape is re-checked by
    ``SimulationConfig.resolved_initial_positions()`` as well.
    """

    initial = raw.get("initial_conditions")
    if not isinstance(initial, dict):
        return None
    positions = initial.get("positions")
    if positions is None:
        return None
    matrix = np.asarray(positions, dtype=float)
    if matrix.shape != (n, 2):
        raise ValueError(
            f"initial_conditions.positions must have shape {(n, 2)}, got {matrix.shape}"
        )
    return matrix


def _informed_from_section(raw: dict[str, Any], n: int) -> np.ndarray | None:
    """Parse the optional ``initial_conditions.informed`` per-robot mask.

    Absent -> None (all robots sensing-capable, back-compatible). When present,
    must be a length-``n`` list of 0/1 flags.
    """

    initial = raw.get("initial_conditions")
    if not isinstance(initial, dict):
        return None
    informed = initial.get("informed")
    if informed is None:
        return None
    arr = np.asarray(informed, dtype=int)
    if arr.shape != (n,):
        raise ValueError(
            f"initial_conditions.informed must have shape {(n,)}, got {arr.shape}"
        )
    return arr


def _adjacency_from_topology_section(topology: dict[str, Any], n: int) -> IntArray | None:
    adjacency = topology.get("adjacency")
    if adjacency is not None:
        matrix = np.asarray(adjacency, dtype=int)
        if matrix.shape != (n, n):
            raise ValueError(f"topology.adjacency must have shape {(n, n)}")
        return matrix
    edges = topology.get("edges")
    if edges is None:
        return None
    parsed_edges = [(int(edge[0]), int(edge[1])) for edge in edges]
    return adjacency_from_edges(n, parsed_edges)
