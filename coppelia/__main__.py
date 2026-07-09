"""Module entrypoint for ``python -m coppelia``."""

from __future__ import annotations

from .run_coppelia_experiment import main

if __name__ == "__main__":
    raise SystemExit(main())
