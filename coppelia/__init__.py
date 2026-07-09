"""Phase 3: CoppeliaSim multi-robot build for the sign gradient-free algorithm.

This package moves the Du et al. (2024) source-localization and formation control
law from pure numerical integration (Phase 1) and the numerical unicycle model
(Phase 2) into a physics-capable multi-robot simulator (CoppeliaSim).

The package is intentionally **self-contained**: it re-implements the small
control primitives (Eq. 4 control law, measurement/saturation model, formation
slots) and the unicycle feedback-linearization layer locally instead of importing
``sgf_sim``. This keeps Phase 3 decoupled from the Phase 1/2 package while that
package is still being actively developed, and lets the whole pipeline run against
a lightweight mock backend when CoppeliaSim is not installed.

Two backends are provided behind a common interface:

- :class:`coppelia.backends.mock.MockBackend` -- a pure-Python differential-drive
  kinematic simulator. Requires no external software and is used for offline
  validation and the test-suite.
- :class:`coppelia.backends.zmq_backend.ZmqBackend` -- a CoppeliaSim backend that
  talks to a running simulator through the ZeroMQ remote API.

The two backends produce artifacts (``summary.json``, trajectory/error plots, and
a synchronized animation) in the same shape as the numerical phases so results are
directly comparable.
"""

from __future__ import annotations

from .config import CoppeliaConfig

__all__ = ["CoppeliaConfig"]

__version__ = "0.1.0"
