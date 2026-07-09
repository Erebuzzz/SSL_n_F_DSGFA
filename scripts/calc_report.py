#!/usr/bin/env python3
"""Reconciliation tool for docs/OUTPUT_ANALYSIS.md.

Recomputes the paper's gain condition, ultimate localization bound ``epsilon``, and
the inside-bound verdict for every ``summary.json`` under ``outputs/``, then compares
them against the values the runs recorded. Any disagreement is flagged inline
(GAIN-MISMATCH / BND-MISMATCH / EPS-MISMATCH); a clean run prints none.

The theory functions are imported from ``sgf_sim.theory`` -- the same module the
simulators use -- so this tool verifies *consistency*, not an independent re-derivation.

Two subtleties, both matching the simulators' own logic:
  * ``epsilon`` uses the WORST-CASE (minimum) informed count over the trajectory, not the
    final count: a transient sensing dropout inflates the bound for the whole run.
  * the bound only applies to noise models the theorem covers (``bounded`` / ``none``);
    Gaussian noise violates Assumption 2, so the bound is reported N/A there.

Usage:
    python scripts/calc_report.py            # scan ./outputs
    python scripts/calc_report.py PATH       # scan a different outputs root
"""

from __future__ import annotations

import glob
import json
import math
import os
import sys

from sgf_sim.theory import (
    all_informed_epsilon,
    epsilon_bound,
    f_dmax,
    gain_ratio_threshold,
)


def _safe_epsilon(n: int, n_informed: int, kappa: float, radius: float, delta: float) -> float:
    """epsilon_bound but returns NaN instead of raising on a degenerate denominator."""

    try:
        return epsilon_bound(n, n_informed, kappa, radius, delta)
    except ValueError:
        return float("nan")


def collect(outputs_root: str) -> list[dict]:
    rows: list[dict] = []
    pattern = os.path.join(outputs_root, "**", "summary.json")
    for sp in sorted(glob.glob(pattern, recursive=True)):
        with open(sp) as fh:
            d = json.load(fh)
        if not isinstance(d, dict):
            continue  # sweep/aggregate summaries are JSON arrays -- skip
        p = d.get("parameters", {})
        v = d.get("validation", {})
        m = d.get("metrics", {})
        if not (isinstance(p, dict) and isinstance(v, dict) and isinstance(m, dict)):
            continue
        n = p.get("n")
        kappa = p.get("kappa")
        radius = p.get("radius")
        dmax = p.get("dmax")
        delta = p.get("noise_bound")
        if None in (n, kappa, radius, dmax, delta):
            continue

        alpha = p.get("alpha")
        beta = p.get("beta")
        ratio = alpha / beta if beta else None
        ni_min = v.get("min_n_informed", n) or n

        threshold = gain_ratio_threshold(n, kappa, dmax, radius, delta)
        passed = ratio > threshold if ratio is not None else None
        bound_applicable = p.get("noise_model") in ("bounded", "none") and ni_min > 0
        eps = _safe_epsilon(n, ni_min, kappa, radius, delta) if bound_applicable else float("nan")

        floc = m.get("final_localization_error")
        inb = (floc <= eps) if (floc is not None and bound_applicable) else None
        name = (
            os.path.relpath(sp, outputs_root)
            .replace(os.sep, "/")
            .replace("/summary.json", "")
        )
        rows.append(
            dict(
                name=name,
                ratio=ratio,
                thr=threshold,
                passed=passed,
                rep_pass=v.get("gain_condition_passed"),
                ni=ni_min,
                eps=eps,
                rep_eps=v.get("epsilon"),
                floc=floc,
                inb=inb,
                rep_inb=v.get("inside_bound"),
            )
        )
    return rows


def main(argv: list[str]) -> int:
    outputs_root = argv[1] if len(argv) > 1 else "outputs"
    rows = collect(outputs_root)

    print("== THEORY RECOMPUTE vs RECORDED (all runs) ==")
    print(
        f"{'run':56} {'ratio':>6} {'thr':>7} {'gOK':>3} {'ni':>3} "
        f"{'eps':>6} {'rep_eps':>8} {'floc':>8} {'inB':>3} {'recB':>4}"
    )
    mismatches = 0
    for r in rows:
        gk = "" if r["passed"] is None else ("Y" if r["passed"] else "N")
        rgk = "" if r["rep_pass"] is None else ("Y" if r["rep_pass"] else "N")
        ib = "" if r["inb"] is None else ("Y" if r["inb"] else "N")
        rib = "" if r["rep_inb"] not in (True, False) else ("Y" if r["rep_inb"] else "N")
        flags = ""
        if gk and rgk and gk != rgk:
            flags += " GAIN-MISMATCH"
        if ib and rib and ib != rib:
            flags += " BND-MISMATCH"
        if (
            r["rep_eps"] is not None
            and not math.isnan(r["eps"])
            and abs(r["eps"] - r["rep_eps"]) > 1e-6
        ):
            flags += " EPS-MISMATCH"
        if flags:
            mismatches += 1
        ratio_s = str(round(r["ratio"])) if r["ratio"] else ""
        eps_s = "%.3f" % r["eps"] if not math.isnan(r["eps"]) else "N/A"
        rep_s = "%.3f" % r["rep_eps"] if r["rep_eps"] is not None else "N/A"
        floc_s = "%.4f" % r["floc"] if r["floc"] is not None else "NA"
        print(
            f"{r['name']:56} {ratio_s:>6} {r['thr']:7.1f} {gk:>3} {r['ni']:>3} "
            f"{eps_s:>6} {rep_s:>8} {floc_s:>8} {ib:>3} {rib:>4}{flags}"
        )

    print(f"\n{len(rows)} runs checked, {mismatches} mismatch(es).")
    print("\n== manual theory constants (defaults n=6, kappa=1, R=2, Dmax=12, delta=0.2) ==")
    print(" f_Dmax          =", f_dmax(1, 12, 0.2))
    print(" gain_threshold  =", gain_ratio_threshold(6, 1, 12, 2, 0.2))
    print(" epsilon(all)    =", epsilon_bound(6, 6, 1, 2, 0.2), " remark4 =", all_informed_epsilon(1, 2, 0.2))
    return 1 if mismatches else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
