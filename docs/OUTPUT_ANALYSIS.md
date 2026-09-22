# Output Analysis & Calculation Report

Deep root-cause analysis of the recorded runs under `outputs/`, with an independent
by-hand reconciliation of every run against the theory of the paper (Du et al. 2024),
and the within-theory remedies for the observed errors and chattering.

This document is organized as:

1. **Theory & manual calculations**: the formulas and worked-by-hand constants.
2. **Master reconciliation table**: recorded vs. independently recomputed, all runs.
3. **Deep-dive root causes**: one section per pointed-to output group.
4. **Within-theory remedies vs. deliberate trade-offs.**
5. **Artifact index**: GIF/plot coverage and scope boundaries.

All numeric values below are taken verbatim from the runs' `summary.json` files and
cross-checked against `sgf_sim/theory.py`.

---

## 1. Theory & manual calculations

The paper's Theorem 1 gives a **sufficient** gain condition and an **ultimate localization
error bound** `epsilon`. The three formulas (implemented in `sgf_sim/theory.py`) are:

| Quantity | Formula | Source |
|---|---|---|
| Max sensed value | `f_Dmax = kappa * Dmax^2 + delta` | saturation model |
| Gain threshold | `alpha/beta > 4*n*f_Dmax / R` | Theorem 1 (sufficient) |
| Ultimate bound | `epsilon = 2*pi*n*delta / [ kappa*R*(2*pi*n_inf - n*|sin(2*pi*n_inf/n)|) ]` | Theorem 1 |
| Remark 4 (all informed) | `epsilon = delta / (kappa*R)` | Remark 4 |

### 1.1 Worked constants (defaults: n=6, kappa=1, R=2, Dmax=12, delta=0.2)

Computed by hand and confirmed against the code:

```
f_Dmax          = kappa*Dmax^2 + delta = 1*144 + 0.2            = 144.2
gain_threshold  = 4*n*f_Dmax / R       = 4*6*144.2 / 2          = 1730.4
epsilon(n_inf=6)= 2*pi*6*0.2 / [1*2*(2*pi*6 - 6*|sin(2*pi)|)]   = 0.1
Remark 4        = delta/(kappa*R)      = 0.2 / (1*2)            = 0.1   (agrees)
```

### 1.2 Epsilon as a function of the *worst-case* informed count

The bound depends on `n_informed`, and: critically: the theorem uses the **minimum
(worst-case) informed count over the whole trajectory**, not the final count. A transient
sensing dropout (a robot momentarily beyond `Dmax`) inflates the bound for the entire run:

```
epsilon(n_inf=6) = 0.1000     (all informed: the nominal case)
epsilon(n_inf=5) = 0.1438
epsilon(n_inf=4) = 0.1891
epsilon(n_inf=3) = 0.2000
```

This is why chattering is not merely cosmetic: chattering-driven dropouts raise `min n_informed`,
which **widens the theorem's own admissible bound** (see `mock_gainpass_pure` in §2, where
chattering pushed `min n_informed` to 4 and inflated `epsilon` from 0.1 to 0.189).

### 1.3 Two nuances that drive the entire analysis

- **The gain-ratio condition is *sufficient*, not *necessary*.** Satisfying
  `alpha/beta > 1730.4` guarantees the ultimate bound. But the converse does not hold:
  several runs with ratio = 200 (< 1730.4) still land well inside `epsilon = 0.1`
  (e.g. `unicycle/boundary_layer_validate`, final loc = 0.0103). Failing the condition is
  therefore not proof of a bound violation.
- **The ratio condition is scale-invariant in absolute gain magnitude, but settling time is
  not.** `epsilon` and the threshold depend only on the *ratio* `alpha/beta`, not on the
  absolute magnitudes. But the localization-drift term `-(2*beta/R)*sigma*phi` is **linear in
  beta**, so halving both gains (same ratio) halves the absolute convergence *rate*. Over a
  fixed duration this looks like "not converging" when it is really "needs proportionally more
  simulated time."

---

## 2. Master reconciliation table (recorded vs. independently recomputed)

Every dict-form `summary.json` in the repository was re-checked by an independent script
([`scripts/calc_report.py`](../scripts/calc_report.py), run as
`PYTHONPATH=. python scripts/calc_report.py`) that recomputes the gain condition, `epsilon`,
and the inside-bound verdict from `sgf_sim.theory`: using **min `n_informed`** for `epsilon`, and gating
the bound to the noise models the theorem covers (`bounded` / `none`; Gaussian → N/A because
Assumption 2 requires bounded noise). Columns: `gOK` = gain condition passed, `ni` = min
informed count, `eps` = recomputed bound, `rep_eps` = recorded bound, `inB`/`recB` =
recomputed/recorded inside-bound.

```
run                                                     ratio    thr gOK  ni    eps  rep_eps    floc inB recB
config_smoke/smoke_config_bounded                        2000 1730.4   Y   6  0.100   0.100  0.0282   Y   Y
coppelia/mock_gainpass_bl                                2000 1730.4   Y   6  0.100   0.100  0.3775   N   N
coppelia/mock_gainpass_bl_v2                             2000 1730.4   Y   6  0.100   0.100  0.3775   N   N
coppelia/mock_gainpass_pure                              2000 1730.4   Y   4  0.189   0.189  0.8603   N   N
coppelia/mock_gainpass_tuned                             2000 1730.4   Y   6  0.100   0.100  0.0046   Y   Y
coppelia/mock_smoke                                       200 1730.4   N   6  0.100   0.100  1.2221   N   N
parity/parity_none_matlab                                2000 1730.4   Y   6  0.100   0.100  3.3530   N   N
runs/config_paper_bounded_validation                     2000 1730.4   Y   6  0.100   0.100  0.0282   Y   Y
runs/config_paper_bounded_validation_matlab              2000 1730.4   Y   6  0.100   0.100  0.0317   Y   Y
runs/config_paper_default_matlab                         2000 1730.4   Y   6    N/A   0.100  0.0296   -   -
runs/paper_like_gaussian_seed1                           2000 1730.4   Y   6    N/A   0.100  0.0366   -   -
runs/paper_validation/bounded_theorem_check              2000 1730.4   Y   6  0.100   0.100  0.0282   Y   Y
runs/paper_validation/gaussian_similarity                2000 1730.4   Y   6    N/A   0.100  0.0252   -   -
runs/validate_bounded_seed1_dt0.0005                     2000 1730.4   Y   6  0.100   0.100  0.0348   Y   Y
runs/validate_bounded_seed1_dt0.001                      2000 1730.4   Y   6  0.100   0.100  0.0412   Y   Y
runs/validate_bounded_seed1_dt0.01                       2000 1730.4   Y   6  0.100   0.100  2.6739   N   N
runs/validate_paper_fig1_reconstructed_bounded_..dt0.001 2000 1730.4   Y   6  0.100   0.100  2.9139   N   N
turtlebot/turtlebot_simulink_default_turtlebot            200 1728.0   N   6  0.000   0.000  0.0008   N   N
turtlebot/turtlebot_working_turtlebot                     200 1730.4   N   6  0.100   0.100  0.0009   Y   Y
unicycle/boundary_layer_validate                          200 1730.4   N   6  0.100   0.100  0.0103   Y   Y
unicycle/smoke_cmp/unicycle                               200 1730.4   N   6  0.100   0.100  2.2378   N   N
unicycle/smoke_run                                        200 1730.4   N   6  0.100   0.100  2.0592   N   N
unicycle/smoke_validate                                   200 1730.4   N   6  0.100   0.100  0.8783   N   N
```

**Result: zero mismatches.** For every run, the recomputed gain verdict, `epsilon`, and
inside-bound verdict equal the recorded values. This establishes that the codebase's own
theory bookkeeping is trustworthy, so the discussion below can treat the recorded metrics as
ground truth and focus on *why* certain runs sit outside the bound.

Two rows worth reading carefully:

- `coppelia/mock_gainpass_pure`: `ni = 4`, so `epsilon` correctly widens to **0.189**: this is
  the theorem responding to a chattering-induced sensing dropout, not a code discrepancy.
- `turtlebot_simulink_default`: threshold 1728.0 and `epsilon ≈ 0` reflect that run's slightly
  different `Dmax`/`delta`; the recomputation still matches the record exactly.

---

## 3. Deep-dive root causes

### 3.1 `outputs/coppelia/mock_smoke/`: gain condition fails; chattering; localization error

**Recorded:** alpha=10, beta=0.05, ratio=**200**, dt=0.004, dur=50 s, offset=0.5,
gain_condition_passed=**False** (200 < 1730.4), epsilon=0.1, final_loc=**1.222**,
final_form=0.152, inside_bound=False.

**What is actually happening.**

1. **The gain condition fails by design, not by accident.** The threshold is 1730.4, but this
   run uses ratio 200. The gains alpha=10/beta=0.05 were chosen to be *physically realizable*
   on the differential-drive kinematic backend: the mock backend feedback-linearizes a
   control point offset `r` into wheel/heading commands, and a ratio of 1730+ with a small
   offset produces angular-velocity commands far outside any real robot's envelope. So this run
   trades the theorem's sufficient condition for physical sanity. Because the condition is only
   *sufficient* (§1.3), failing it does not guarantee a violation: but here it does coincide
   with a real miss (final_loc 1.222 > epsilon 0.1).

2. **The chattering has a specific source.** The controller applies the paper's exact `sgn(·)`
   relay term. Near consensus, `z_j - z_i` flips sign every step, so the relay switches at the
   integration frequency. On a differential-drive backend that switching is fed through the
   nonlinear heading map, which converts the position-level chatter into heading oscillation -
   the visible "chattering/oscillation between formation and localization."

3. **Why the two error signals fight.** Formation converges fast (final_form 0.152) while
   localization lags at 1.222. This is the beta-drift term being slow at these absolute gains:
   with beta=0.05 the drift toward the source is gentle, and 50 s is not enough time to close
   the gap while the relay keeps nudging the centroid around.

**Within-theory remedy (fully demonstrated).** `outputs/coppelia/mock_gainpass_tuned/`:

- alpha=100, beta=0.05 → ratio=**2000 > 1730.4**: the sufficient condition now **passes**.
- `sign_boundary_layer = 0.2`: replaces `sgn(x)` with the saturation `sat(x/eps)`, which is
  the standard sliding-mode boundary-layer construction. It removes the near-equilibrium
  chattering *without leaving the theory* (the boundary layer only changes behavior inside a
  thin band around consensus, preserving the relay far from it).
- dt=0.001: avoids the coarse-dt instability discussed in §3.3.
- control_point_offset r=5.0: a larger offset keeps the feedback-linearized angular velocity
  physically reasonable at the higher gain.

**Result:** gain condition passes, epsilon=0.1, final_loc=**0.0046**, inside_bound=**True** -
i.e. the localization error lands ~20× inside the theorem's bound, chatter-free. This is the
constructive proof that `mock_smoke`'s issues are curable strictly within the paper's framework.

### 3.2 `outputs/parity/parity_none_matlab/`: errors high vs. allowed bound

**Recorded:** alpha=100, beta=0.05, ratio=2000 (passes), dt=0.01, dur=**5 s**, noise=none,
epsilon=0.1, final_loc=**3.353**, final_form=5.268, inside_bound=False.

**What is actually happening: this is a duration artifact, not a bound violation.** This run is
a **parity check**: its job is to confirm that the MATLAB and Python implementations produce
*bit-identical trajectories* over a short, deterministic window, so it deliberately runs for
only **5 seconds**. Five seconds is nowhere near long enough for the beta-drift term to pull the
centroid onto the source, so the localization error is naturally still large. The theorem's
bound is an *ultimate* (asymptotic) bound; reading it against a 5 s snapshot is a category error.

**Evidence.** Continuing the identical configuration (alpha=100, beta=0.05, dt=0.01,
noise=none) out to 60 s shows the error decaying exactly as the theory predicts: it was simply
truncated early:

```
t =  5.00 s   loc = 3.353   form = 5.269     <- where the parity run stops
t = 10.00 s   loc = 2.419   form = 6.046
t = 20.00 s   loc = 1.622   form = 5.371
t = 40.00 s   loc = 0.754   form = 3.227
t = 60.00 s   loc = 0.550   form = 4.715
```

The localization error falls monotonically (3.35 → 0.55) once given time. (It has not yet
reached 0.1 at 60 s here because dt=0.01 with alpha=100 also suffers the coarse-dt effect of
§3.3: but the parity run's *purpose* is reproducibility, not convergence.)

**Is parity itself OK?** Yes. MATLAB↔Python numerical parity is verified separately and exactly
by `verify_parity.m`; the high residual here is orthogonal to parity. **No remedy is needed** -
this run is doing its job. If a *converged* reference is wanted instead, use a validation-quality
config (long duration + fine dt), e.g. `runs/validate_bounded_seed1_dt0.0005` (§3.3).

### 3.3 `outputs/runs/validate_bounded_seed1_dt*`: dt sweep: chattering and localization error

The user's observation, restated precisely against the recorded metrics:

| run | alpha | beta | ratio | dt | dur | final_loc | final_form | inside_bound |
|---|---|---|---|---|---|---|---|---|
| validate_bounded_seed1_dt0.01 | 10 | **0.005** | 2000 | 0.01 | 60 | **2.674** | 0.677 | No |
| validate_bounded_seed1_dt0.001 | 100 | 0.05 | 2000 | 0.001 | 60 | 0.041 | 0.661 | **Yes** |
| validate_bounded_seed1_dt0.0005 | 100 | 0.05 | 2000 | 0.0005 | 60 | 0.035 | 0.290 | **Yes** |
| validate_paper_fig1_reconstructed_..dt0.001 | 100 | 0.05 | 2000 | 0.001 | **5** | 2.914 | 0.533 | No |

All four have the **same ratio (2000) and pass the gain condition**, yet behave very
differently. There are **two separate, additive causes**: isolated with a controlled 2×2
experiment that varied absolute gain magnitude and dt independently while holding the ratio
fixed.

**Cause (a): absolute gain magnitude sets the convergence *rate* (not just the ratio).**
The `dt0.01` run is not just coarser in time: it also uses **beta=0.005**, ten times smaller
than its siblings' 0.05 (alpha is 10 vs 100, so the ratio is unchanged at 2000). The
localization-drift term `-(2*beta/R)*sigma*phi` is **linear in beta**, so this run converges
~10× *slower in absolute time*. Over the same 60 s it simply hasn't arrived yet (loc 2.674).
This is a "needs proportionally more simulated time" effect, **not** an instability and **not**
a bound violation: the ratio-based `epsilon` is still 0.1 and still valid; the trajectory is
just far from its asymptote. This is the single biggest contributor to the poor `dt0.01`
localization, and it is independent of dt.

**Cause (b): coarse dt + large absolute alpha → genuine explicit-Euler instability.** When
alpha is *large* (100) and dt is *coarse*, the discontinuous relay controller integrated with
explicit Euler becomes numerically unstable: the formation error does not settle to a small
chatter floor, it **grows**: overshooting past its own initial descent. This is the classical
step-size stability limit for discretized high-gain discontinuous (sliding-mode-like) systems.
It is visible as the heaviest chattering and shows up as an elevated formation-error floor.

**How the two causes combine across the four runs:**

- `dt0.0005` (alpha=100, fine dt): neither cause active → cleanest run, lowest chatter floor
  (form 0.290), loc 0.035, well inside bound. **This is the reference-quality run.**
- `dt0.001` (alpha=100, moderate dt): cause (b) mild → moderate chatter, but loc still converges
  to 0.041, inside bound. Matches the user's "chattering moderate, localization eventually under
  bound."
- `dt0.01` (alpha=10, beta=0.005, coarse dt): cause (a) dominates (10× slow rate) → loc stuck at
  2.674. Matches the user's "chattering most and localization error too." (Note the *formation*
  floor is actually low here, 0.677, because alpha is small: the visible instability is
  amplitude-dependent; small alpha keeps it bounded, at the cost of a slow beta.)
- `validate_paper_fig1_reconstructed_..dt0.001`: same gains as the good `dt0.001` run but
  **dur=5 s**, not 60 s → loc 2.914 is again a **duration artifact** (§3.2), not a dt problem.

**Negative result: the boundary layer does *not* fix cause (b).** A natural guess is to apply
`sign_boundary_layer` to remove the coarse-dt instability. It does not: the boundary layer only
smooths behavior *near* consensus (small `|z_j - z_i|`, where `sat(x/eps)` is gentle). When the
trajectory oscillates at *large amplitude* far from consensus, `sat(x/eps) ≈ sgn(x)` almost
everywhere, so the boundary layer has essentially no effect on large-amplitude instability.
Empirically, applying bl=0.2 to the unstable large-alpha/coarse-dt case made both errors
*worse*. The correct within-theory remedy for cause (b) is simply **using a smaller dt when
alpha is large** (as `dt0.0005` demonstrates).

**Within-theory remedies, summarized:**
- For cause (a): keep the ratio, but do not shrink absolute gains below what the duration can
  afford: or extend the duration proportionally (the bound is unchanged, only time-to-arrive).
- For cause (b): reduce dt for large alpha (dt=0.0005 at alpha=100 is stable and clean).
- Duration artifacts (fig1_reconstructed): run long enough to reach the *ultimate* bound.

### 3.4 `outputs/unicycle/{smoke_cmp, smoke_run, smoke_validate}`: chattering + localization error

| run | alpha | beta | ratio | dt | dur | bl | noise | final_loc | final_form |
|---|---|---|---|---|---|---|---|---|---|
| smoke_cmp/unicycle | 10 | 0.05 | 200 | 0.01 | 20 | 0.0 | none | 2.238 | 0.551 |
| smoke_run | 10 | 0.05 | 200 | 0.01 | 20 | 0.0 | bounded | 2.059 | 0.493 |
| smoke_validate | 10 | 0.05 | 200 | 0.004 | 60 | 0.0 | bounded | 0.878 | 0.167 |

**What is actually happening.**

1. **Chattering source (smoke_cmp, smoke_run).** These use the exact `sgn(·)` relay
   (`sign_boundary_layer = 0.0`) on the *unicycle* model. The unicycle couples the commanded
   velocity into a nonlinear heading map, so the relay's per-step sign flips become heading
   oscillations: the "tons of chattering." dt=0.01 makes it worse (fewer steps to average the
   switching), and the short 20 s horizon means localization (loc ≈ 2.1–2.2) never gets near
   the bound. Both share the same cause; the only difference is noise (none vs bounded), which
   barely moves the result.

2. **Why smoke_validate is better.** Same gains (ratio 200) but **finer dt (0.004)** and a
   **longer horizon (60 s)**. Finer dt averages the relay switching more, so chattering drops
   (form 0.167, loc tail std 0.025), and the longer run lets the beta-drift make real progress
   (loc 0.878 vs ~2.1). It still misses epsilon=0.1 because (i) the gain ratio is only 200 (the
   *sufficient* condition is not met) and (ii) the pure relay still chatters somewhat. This
   matches the user's "localization error, less chattering than the others."

3. **The gain condition is failed in all three (ratio 200 < 1730.4)**: but recall this is only
   *sufficient* (§1.3), so it does not by itself prove the miss.

**Within-theory remedy (demonstrated).** `outputs/unicycle/boundary_layer_validate/`: identical
gains (ratio still **200**, still fails the sufficient condition) but with
`sign_boundary_layer = 0.2` on the same dt=0.004 / 60 s setup. Result: final_form=**0.0052**,
final_loc=**0.0103**, **inside_bound = True**, and the commanded velocities *drop*
(max linear 29.7 vs ~44, max angular 16.2 vs ~21.5) because the controller is no longer
slamming between ±relay values. The tail formation std falls to 3e-5: chatter essentially
eliminated.

**The important theoretical takeaway.** This run lands ~10× inside epsilon=0.1 *while failing
the sufficient gain condition*. That is the concrete demonstration that the gain-ratio
condition is **sufficient but not necessary**: the boundary-layer relay achieves the ultimate
bound at ratio 200 because removing the chattering keeps all robots continuously informed (min
`n_informed` stays at 6, so epsilon stays at its tightest 0.1) and lets the drift term work
cleanly.

---

## 4. Within-theory remedies vs. deliberate trade-offs

A key point for interpreting these outputs: **not every "error" is a defect.** Some runs are
deliberately configured for a purpose other than convergence, and their high residuals are
expected. The table separates the two categories.

| Observation | Category | Explanation / remedy |
|---|---|---|
| `mock_smoke` fails gain condition (ratio 200) | **Deliberate trade-off** | Sane gains for a physically realizable differential-drive backend. Remedy shown in `mock_gainpass_tuned` (ratio 2000 + bl 0.2 + dt 0.001 + offset 5 → inside bound). |
| `parity_none_matlab` high error | **Deliberate (duration artifact)** | 5 s parity check, not a convergence run. No remedy needed; parity itself is exact. |
| `validate..fig1_reconstructed` high error | **Deliberate (duration artifact)** | 5 s vs 60 s. Run longer to reach the ultimate bound. |
| `validate..dt0.01` poor localization | **Genuine (cause a)** | 10× smaller absolute beta → 10× slower rate. Remedy: don't shrink absolute gains below the duration budget, or extend duration. |
| Large-alpha + coarse-dt instability | **Genuine (cause b)** | Explicit-Euler step-size limit for the discontinuous controller. Remedy: smaller dt (dt=0.0005 at alpha=100). **Boundary layer does not fix this.** |
| Chattering near equilibrium (unicycle, mock) | **Genuine** | Pure `sgn` relay switching. Remedy: `sign_boundary_layer > 0`: the standard sliding-mode boundary layer, fully within the paper's model. |
| Runs at ratio 200 still inside bound | **Not an error** | Gain condition is *sufficient, not necessary*; the boundary-layer relay reaches the bound at ratio 200. |

**Remedies that are genuinely theorem-consistent** (not ad-hoc hacks):
1. **Boundary layer** `sat(x/eps)`: the textbook sliding-mode regularization of the relay;
   default 0.0 preserves exact paper behavior and parity.
2. **Correct dt for the chosen alpha**: respects the explicit-Euler stability limit of the
   discretized discontinuous system.
3. **Sufficient duration**: the bound is *ultimate/asymptotic*; give the drift term time.
4. **Larger control-point offset**: keeps feedback-linearized angular velocity physical when
   gains are raised to pass the sufficient condition.

None of these alter the paper's controller law or its assumptions; they are choices of
regularization width, step size, horizon, and kinematic parameterization.

---

## 5. Artifact index & scope boundary

**Calculation report:** §1–§2 above constitute the requested by-hand calculation report: the
formulas, the worked constants, and the run-by-run reconciliation showing recorded values equal
independently recomputed theory values with **zero mismatches**.

**GIF + plot coverage:** every **single-run** output folder has both static plots (PNGs) and a
motion GIF: **23 / 23 single-run folders** are fully covered. The three gaps that existed were
filled by exactly reproducing each run's trajectory (bit-identical check before writing):
- Python: `config_smoke/smoke_config_bounded`, `runs/paper_validation/gaussian_similarity`
  (via `sgf_sim/animation.py`).
- MATLAB: `runs/config_paper_bounded_validation_matlab` (via `sgf_plot_results` over the saved
  `result.mat`).

Python static output plots were generated for the Python runs as requested (the coppelia and
MATLAB pipelines already emitted their own plots).

**Scope boundary (explicit).** Five folders have no GIF/PNG **by design**: they are
aggregate/sweep folders that store only final-metric tables (`summary.json` / `.csv` / `.md` /
`report.md`), with **no raw per-step trajectory data**:
- `runs/experiments/gain-ratio-sweep`
- `runs/experiments/paper-suite`
- `runs/experiments/seed-sweep`
- `runs/gain_sweep`
- `runs/sweep_topology`

Producing GIFs for these would require re-running every sweep point individually to regenerate
trajectories. That is intentionally out of scope: these folders are summaries, not runs. This
is a boundary, not a missing artifact.

---

## Summary

- The codebase's theory bookkeeping is **exactly correct**: every recorded gain verdict,
  `epsilon`, and inside-bound flag reproduces by hand (§2).
- The pointed-to "errors" resolve into three kinds: **deliberate trade-offs** (physical gains,
  parity/short-duration snapshots), **genuine but curable-within-theory** issues (chattering →
  boundary layer; coarse-dt instability → smaller dt; slow rate → adequate duration/absolute
  gains), and **non-errors** arising from the sufficient-not-necessary nature of the gain
  condition.
- Constructive within-theory fixes are demonstrated end-to-end: `mock_gainpass_tuned`
  (loc 0.0046) and `unicycle/boundary_layer_validate` (loc 0.0103) both land well inside
  epsilon = 0.1, chatter-free.
- Artifact coverage is complete for all single-run folders; sweep folders are summaries with no
  trajectory data and are out of scope for animation.
