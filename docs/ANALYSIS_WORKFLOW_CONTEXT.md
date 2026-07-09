# Output Analysis — Workflow Context

Status snapshot for the "deep output analysis + calculation report + GIF/plot coverage"
task requested against the recorded runs under `outputs/`. This file tracks *what is
already done and verified* vs *what remains*, so the work can be resumed cleanly.

_Last updated: 2026-07-09_

---

## Task (as requested)

1. Deep root-cause analysis ("actual reasons") for each pointed-to output group.
2. Determine, where possible, fixes that stay **within the theory of the paper**.
3. A written **calculation report** comparing each recorded run against manual/independent
   theory computation.
4. A **GIF** exported for every run type.
5. Static **output plots (PNGs)** for the Python runs (MATLAB/coppelia already had them).

Pointed-to outputs:
- `outputs/coppelia/mock_smoke/` — localization error + chattering; gain condition should pass.
- `outputs/parity/parity_none_matlab/` — errors high vs allowed bound.
- `outputs/runs/validate_bounded_seed1_dt{0.01,0.001,0.0005}/` + `validate_paper_fig1_reconstructed...`
  — compare dt effects (chattering, localization error).
- `outputs/unicycle/smoke_cmp/unicycle/`, `outputs/unicycle/smoke_run/` — heavy chattering + loc error.
- `outputs/unicycle/smoke_validate/` — loc error, less chattering.

---

## DONE (gathered + verified)

### Theory constants (manual, defaults n=6, kappa=1, R=2, Dmax=12, delta=0.2)
- `f_Dmax = kappa*Dmax^2 + delta = 144.2`
- `gain_threshold = 4*n*f_Dmax/R = 1730.4`
- `epsilon(n_informed=6) = 0.1` (matches Remark 4 `delta/(kappa*R) = 0.1`)
- `epsilon(5) = 0.1438`, `epsilon(4) = 0.1891`, `epsilon(3) = 0.2`
- Confirmed these match `sgf_sim/theory.py` formulas exactly.

### Reconciliation (recorded vs independently recomputed)
- `tmp/calc_report.py` recomputes `gain_condition_passed`, `epsilon`, `inside_bound` for
  **every** dict-type `summary.json` in the repo → **zero mismatches**.
- Key correction baked in: epsilon uses **min (worst-case) `n_informed`** over the trajectory,
  not the final count (a transient sensing dropout inflates the theoretical bound for the whole run).
- Gaussian-noise runs correctly gated as **bound-not-applicable** (Assumption 2 needs bounded noise).

### Precise recorded numbers pulled for each pointed-to run (verified from summary.json)
| run | alpha | beta | ratio | dt | dur | gainOK | eps | final_loc | final_form | inBound |
|---|---|---|---|---|---|---|---|---|---|---|
| coppelia/mock_smoke | 10 | 0.05 | 200 | 0.004 | 50 | N | 0.1 | 1.222 | 0.152 | N |
| coppelia/mock_gainpass_pure | 100 | 0.05 | 2000 | 0.004 | 60 | Y | 0.189 (ni=4) | 0.860 | 2.557 | N |
| coppelia/mock_gainpass_bl_v2 | 100 | 0.05 | 2000 | 0.004 | 60 | Y | 0.1 | 0.377 | 2.115 | N |
| coppelia/mock_gainpass_tuned | 100 | 0.05 | 2000 | 0.001 | 60 | Y | 0.1 | **0.0046** | 0.490 | **Y** |
| parity/parity_none_matlab | 100 | 0.05 | 2000 | 0.01 | **5** | Y | 0.1 | 3.353 | 5.268 | N |
| runs/validate_...dt0.01 | 10 | 0.005 | 2000 | 0.01 | 60 | Y | 0.1 | 2.674 | 0.677 | N |
| runs/validate_...dt0.001 | 100 | 0.05 | 2000 | 0.001 | 60 | Y | 0.1 | 0.041 | 0.661 | **Y** |
| runs/validate_...dt0.0005 | 100 | 0.05 | 2000 | 0.0005 | 60 | Y | 0.1 | 0.035 | 0.290 | **Y** |
| runs/validate_paper_fig1_reconstructed...dt0.001 | 100 | 0.05 | 2000 | 0.001 | **5** | Y | 0.1 | 2.914 | 0.533 | N |
| unicycle/smoke_cmp/unicycle | 10 | 0.05 | 200 | 0.01 | 20 | N | 0.1 | 2.238 | 0.551 | N |
| unicycle/smoke_run | 10 | 0.05 | 200 | 0.01 | 20 | N | 0.1 | 2.059 | 0.493 | N |
| unicycle/smoke_validate | 10 | 0.05 | 200 | 0.004 | 60 | N | 0.1 | 0.878 | 0.167 | N |
| unicycle/boundary_layer_validate | 10 | 0.05 | 200 | 0.004 | 60 | N | 0.1 | **0.0103** | 0.0052 | **Y** |

### Root causes established (experiments already run)
- **coppelia/mock_smoke** — gain condition fails *by design* (sane gains alpha=10/beta=0.05,
  ratio=200 < 1730.4, chosen for physical realizability). Full remedy demonstrated in
  `mock_gainpass_tuned` (alpha=100 → ratio=2000 passes; bl=0.2 kills chatter; dt=0.001 avoids
  coarse-dt instability; offset r=5.0 keeps angular velocity sane) → lands **inside** eps=0.1.
- **parity/parity_none_matlab** — high residual is a **duration artifact**: 5 s deterministic
  parity-check snapshot, not a convergence run. Traced same config to 60 s → loc 3.35 → 0.55.
  Not a bug, not a parity failure (parity is exact, verified separately by `verify_parity.m`).
- **validate_bounded_seed1_dt\*** — two separable, additive causes (2×2 experiment):
  (a) **absolute gain magnitude** sets loc convergence *rate* (linear in beta); the dt0.01 run
      used beta=0.005 (10× smaller) → 10× slower → looks unconverged at 60 s.
  (b) **coarse dt + large alpha** → genuine explicit-Euler instability of the discontinuous
      relay controller (formation *blows up*, not just chatters).
  Boundary layer does **NOT** fix (b) (large-amplitude → sat≈sgn). Correct fix: smaller dt for large alpha.
  `validate_paper_fig1_reconstructed...dt0.001` poor result = duration artifact (5 s, not 60 s).
- **unicycle smoke_\*** — chattering from discontinuous sgn feeding unicycle heading coupling;
  gain ratio 200 < 1730.4 (fails sufficient condition). `boundary_layer_validate` (bl=0.2) lands
  loc=0.0103 **inside** eps=0.1 → demonstrates gain condition is **sufficient, not necessary**.

### Artifacts (GIF + PNG coverage)
- **100% coverage** across all single-run folders: every one has 3 PNGs + 1 GIF.
  Verified via scan — only the aggregate/sweep folders lack artifacts (see below).
- Filled 3 gaps: 2 Python (`config_smoke/smoke_config_bounded`,
  `runs/paper_validation/gaussian_similarity`) via `tmp/fill_gif_gaps.py` +
  `sgf_sim/animation.py`; 1 MATLAB (`runs/config_paper_bounded_validation_matlab`) via MCP.
  All reproduced bit-identically before writing.
- `sign_boundary_layer` option confirmed wired across Python (`sgf_sim/`), MATLAB
  (`matlab/sgf_control.m`), and coppelia (`coppelia/`) packages; default 0.0 = exact paper sgn.

---

## REMAINING (to do)

1. **Write `docs/OUTPUT_ANALYSIS.md`** — the actual synthesized deliverable. All data above is
   ready; this is pure writing. Structure:
   - (a) Theory formulas + worked manual calculations (constants above).
   - (b) Master reconciliation table (recorded vs recomputed → zero mismatches).
   - (c) Deep-dive root-cause section per output group (content above).
   - (d) Within-theory remedies vs deliberate design trade-offs, clearly distinguished.
   - (e) Artifact index + explicit scope note.
2. **Final verification pass** of the written report.
3. (Optional) Clean up `tmp/` scratch scripts (`tmp/` is gitignored).

### Scope boundary (state explicitly in report)
- Aggregate/sweep folders store only final-metric tables (no raw per-step trajectories):
  `runs/experiments/{gain-ratio-sweep,paper-suite,seed-sweep}`, `runs/gain_sweep`,
  `runs/sweep_topology`. **GIF export is not possible** for these without re-running each
  sweep point individually — intentional boundary, not a gap.

### Key nuances to preserve in the writeup
- Gain ratio condition is **sufficient, not necessary** for the ultimate bound.
- Epsilon depends on **min** `n_informed` over trajectory, not final.
- Ratio condition is scale-invariant in absolute (alpha,beta) magnitude, but **settling time is not**
  (drift term is linear in beta).
- Two distinct integration failure modes: weak-gain slow-rate (not instability) vs coarse-dt
  large-alpha genuine instability. Boundary layer only smooths near-equilibrium chatter.
