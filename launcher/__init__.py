"""Unified interactive launcher for the sign gradient-free simulator.

`launcher.core` is a headless, dependency-light layer (numpy only) that holds the
run configuration, computes the pre-run theory readout (gain condition, epsilon
bound, minimum-informed condition), gates invalid platform/mode combinations, and
dispatches a run to the chosen platform (Python in-process, CoppeliaSim backend, or
MATLAB via subprocess). `launcher.gui` is a thin tkinter front-end on top of it.

The split keeps all logic testable without a display: the GUI only reads/writes the
`LauncherState` and calls `core` functions.
"""

from .core import (
    LauncherState,
    PLATFORMS,
    MODES_BY_PLATFORM,
    TheoryReadout,
    compute_readout,
    dispatch,
    valid_modes,
)

__all__ = [
    "LauncherState",
    "PLATFORMS",
    "MODES_BY_PLATFORM",
    "TheoryReadout",
    "compute_readout",
    "dispatch",
    "valid_modes",
]
