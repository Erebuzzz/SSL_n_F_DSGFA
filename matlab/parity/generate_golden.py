"""Generate a golden reference for the Python<->MATLAB parity check.

This runs the *Python* Phase 1 simulator (``sgf_sim``) on a deterministic
noise-free shared config and dumps the full error time-series, final poses, and
summary metrics to ``golden_parity_none.json``. The MATLAB ``verify_parity.m``
script runs the MATLAB parity simulator on the same config and asserts agreement
within tolerance.

Noise-free is required: NumPy (PCG64) and MATLAB (Mersenne Twister) generate
different random streams from the same seed, so only a ``noise.model = "none"``
run is expected to match sample-for-sample.

Run from the repository root:

    python matlab/parity/generate_golden.py

This only *reads* / imports ``sgf_sim`` (Phase 1); it does not modify it.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sgf_sim.shared_config import load_shared_config  # noqa: E402
from sgf_sim.simulation import run_simulation  # noqa: E402

CONFIG_PATH = HERE / "parity_none.json"
GOLDEN_PATH = HERE / "golden_parity_none.json"


def main() -> None:
    shared = load_shared_config(CONFIG_PATH)
    result = run_simulation(shared.simulation)
    summary = result.summary()

    golden = {
        "source": "python:sgf_sim.run_simulation",
        "config_file": CONFIG_PATH.name,
        "n_steps": int(len(result.times)),
        "times": [float(t) for t in result.times],
        "formation_error": [float(v) for v in result.formation_error],
        "localization_error": [float(v) for v in result.localization_error],
        "final_positions": result.positions[-1].tolist(),
        "final_centroid": result.centroid[-1].tolist(),
        "summary": {
            "validation": summary["validation"],
            "metrics": summary["metrics"],
        },
    }
    GOLDEN_PATH.write_text(json.dumps(golden, indent=2), encoding="utf-8")

    # Console recap so the numbers are visible in the run log / README.
    m = summary["metrics"]
    v = summary["validation"]
    print(f"Wrote {GOLDEN_PATH}")
    print(f"  n_steps                 = {golden['n_steps']}")
    print(f"  final_formation_error   = {m['final_formation_error']:.12g}")
    print(f"  final_localization_error= {m['final_localization_error']:.12g}")
    print(f"  gain_ratio              = {v['gain_ratio']:.12g}")
    print(f"  epsilon                 = {v['epsilon']}")
    print(f"  inside_bound            = {v['inside_bound']}")
    print(f"  final_centroid          = {np.round(result.centroid[-1], 6).tolist()}")


if __name__ == "__main__":
    main()
