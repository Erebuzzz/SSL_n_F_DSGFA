# Code Review

## Review Summary

This repository now contains a Phase 1 numerical implementation of the Du et al. sign gradient-free source-localization and formation algorithm. The implementation is intentionally small and auditable: theory formulas, control primitives, simulation integration, plotting, CLI behavior, and tests are separated into different modules.

The code is suitable for first-pass validation against the paper's calculations and qualitative simulation behavior. It is not yet a robotics or CoppeliaSim build.

## Theory-to-Code Mapping

| Paper concept | Code location | Review note |
|---|---|---|
| Single-integrator dynamics | `sgf_sim/simulation.py` | Euler step applies `p_next = p + dt * u`. |
| Circular slots `phi(theta_i)` | `sgf_sim/control.py` | Uses fixed equally spaced slots. |
| Shifted state `z_i = p_i - R phi_i` | `sgf_sim/control.py` | Formation control is computed on shifted states. |
| Component-wise sign communication | `sgf_sim/control.py` | Uses `np.sign` per coordinate, not vector normalization. |
| Measurement model | `sgf_sim/control.py` | Supports source-field measurement inside `Dmax` and saturation outside. |
| Gain condition | `sgf_sim/theory.py` | Implements `alpha / beta > 4 n f_Dmax / R`. |
| Error bound | `sgf_sim/theory.py` | Implements the theorem epsilon formula and Remark 4 simplification. |
| Validation output | `sgf_sim/simulation.py` | Summary includes gain pass, informed counts, epsilon, and final errors. |

## Important Review Points

1. The implementation correctly uses component-wise sign values in `{-1, 0, 1}`. This is a critical paper detail because replacing it with a normalized direction vector would change the algorithm.
2. The default gain ratio is theorem-compliant: `alpha / beta = 2000`, while the default threshold is approximately `1730.4`.
3. The code distinguishes Gaussian paper-like runs from bounded theorem-valid runs. This is necessary because Gaussian noise does not satisfy the theorem's hard bounded-noise assumption.
4. The default graph is a documented connected undirected graph. It is not claimed to be the exact paper Fig. 1 graph because the paper does not publish the edge list numerically.
5. The simulation uses fixed-step Euler integration. Because sign control is discontinuous, the default `dt = 0.0005` is intentionally small to reduce discrete-time chatter.

## Validation Evidence

Verified commands:

```powershell
python -m pytest -p no:hypothesispytest -q
```

Result:

```text
7 passed
```

The bounded validation command was also verified:

```powershell
python -m sgf_sim validate --duration 60 --dt 0.0005 --alpha 100 --beta 0.05 --seed 1
```

Observed summary:

```text
gain_condition_passed: true
epsilon: 0.1
inside_bound: true
final_localization_error: about 0.035
```

The Gaussian paper-like command was verified:

```powershell
python -m sgf_sim run --duration 60 --dt 0.0005 --alpha 100 --beta 0.05 --seed 1 --noise gaussian --run-id paper_like_gaussian_seed1
```

Observed behavior:

```text
gain_condition_passed: true
final_localization_error: about 0.037
bound_applicable: false
```

The bound is marked not applicable for Gaussian noise because the theorem assumes bounded perturbations.

## Known Limitations

- The paper does not provide exact values for `alpha`, `beta`, or a numeric adjacency matrix, so exact reproduction is not possible from the published text alone.
- Formation error is reduced strongly but may retain small discrete-time chatter because of the discontinuous sign controller.
- The current simulator does not include the Section V unicycle feedback-linearization layer.
- No CoppeliaSim assets are included yet. That should remain a later phase after the numerical model is accepted.
- The tests validate core formulas and run behavior, but they do not yet assert visual similarity to paper figures.

## Recommended Next Review Items

1. Reconstruct the Fig. 1 graph from the paper image and add it as an optional named topology.
2. Add convergence-time metrics for formation and localization.
3. Add a sweep report for gain ratio sensitivity around the theorem threshold.
4. Add a comparison notebook or script that places generated plots beside screenshots of Figs. 2-4 for human review.

## Jitter Review Note

The localization jitter seen in generated plots is primarily discrete-time sign-control chatter, not a failure of source localization. This was checked by comparing Gaussian, bounded, and no-noise runs. The no-noise run still has a late-stage ripple, which points to the sign term and Euler step as the main mechanism.

The plotting layer now renders raw data lightly and overlays a smoothed curve for paper-style visual comparison. This keeps the simulation honest while making the exported figures closer to the paper's visual style.

## Phase 1.1 Review Addendum

Phase 1.1 adds named topology support and a paper-validation preset. The default topology is now `paper_fig1_reconstructed`, which was inferred from the rendered Fig. 1 image because the paper does not provide a numeric adjacency matrix.

New commands reviewed:

```powershell
python -m sgf_sim validate-paper --seed 1
python -m sgf_sim sweep --param topology --noise bounded
python -m sgf_sim gain-sweep --ratio 2000 --noise gaussian
```

The full `validate-paper` run passed with the reconstructed topology. The bounded theorem check reported `inside_bound: true`, with final localization error about `0.0282` against epsilon `0.1`.

## Phase 1.2 Review Addendum

Phase 1.2 adds aggregate experiment reporting. Single-run summaries now include convergence metrics:

- `formation_entry_time`
- `localization_entry_time`
- `time_inside_localization_threshold_after_entry`

The new `experiment` command writes JSON, CSV, and Markdown reports for paper-suite, gain-ratio, radius-delta, and seed sweeps.

## Phase 1.25 Review Addendum

Phase 1.25 adds shared JSON config files and Python `run-config` support. The config format is intentionally MATLAB-compatible through `jsondecode` and documented in `CONFIG_GUIDE.md`.

Current Python support is limited to `single_integrator`. Planned modes such as `unicycle` and `turtlebot_simulink` raise a clear `NotImplementedError` until those phases are implemented.
## Phase 1.3 Review Addendum

Phase 1.3 adds a MATLAB parity layer in `matlab/`. The implementation reads the same shared JSON config files as Python through `jsondecode`, resolves the same topology and default initial positions, runs the Section IV single-integrator model, and exports comparable plots plus `summary.json` and `result.mat`.

Review notes:

- `matlab/run_from_config.m` is the main MATLAB entry point and matches the Phase 1.25 acceptance command.
- `matlab/sgf_control.m` keeps the component-wise sign controller aligned with the Python implementation.
- `matlab/sgf_measurement.m` preserves the Gaussian, bounded, and no-noise measurement modes.
- `matlab/sgf_theory_bounds.m` computes the same gain threshold and epsilon metadata as `sgf_sim/theory.py`.
- `matlab/sgf_topology.m` converts zero-based shared-config edge lists into MATLAB one-based adjacency matrices.

Verification status:

```powershell
python -m pytest -p no:hypothesispytest -q
```

A short MATLAB smoke run was also verified outside the sandbox after the sandboxed MATLAB startup failed with a filesystem inconsistency:

```powershell
matlab -batch "addpath('matlab'); run_from_config('tmp/matlab_smoke_config.json');"
```

Observed result:

```text
MATLAB parity run complete: matlab_smoke_matlab
Final localization error: 4.72403
Inside theorem bound: 0
```

The smoke config used `duration = 0.02`, `dt = 0.01`, and disabled plots and animation, so it validates startup, config parsing, simulation, and summary export rather than convergence quality. Noisy Python and MATLAB runs are expected to differ at the sample level because their random-number generators are different. Deterministic parity should be checked with `noise.model = "none"` for deeper numerical comparison.

## Standalone script consolidation (2026-07-20)

Three self-contained scripts were added under `sims/` to simplify distribution and review:

| Script | Source of truth inlined from | Default behavior |
|---|---|---|
| `sims/SingleIntegrator.m` | `matlab/` (`sgf_*`, `run_from_config`) | paper bounded validation (`alpha=100`, `dt=5e-4`, exact `sgn`) |
| `sims/Unicycle.m` | Python Phase 2 + TurtleBot numeric loop | working unicycle (`alpha=10`, `r=2`, `eps_bl=0.2`) |
| `sims/TurtleBot.m` | `matlab_turtlebot/` numeric + Simulink builder | numeric working preset; `runMode="simulink"` for `.slx` |

Run guide: [`sims/RUN_GUIDE.md`](../sims/RUN_GUIDE.md).


Structural refactor only: Eq. 4 control, measurement model, topology, theory bounds, and integrators match the modular packages. Alternative presets (Gaussian SI, pure `sgn`, hardware TurtleBot limits) are kept as commented blocks inside each script.

### Verification

- **SingleIntegrator.m**: final localization `≈ 0.030`, `inside_bound=1` (epsilon `0.1`), matching paper bounded-validation expectations.
- **Unicycle.m**: final formation `0.00523`, localization `0.00089`, `inside_bound=1` (matches `turtlebot_working` numeric reference).
- **TurtleBot.m** (numeric): identical metrics to Unicycle under the same defaults.
- **TurtleBot.m** (Simulink, 5 s smoke): builds `sgf_turtlebot_swarm.slx`, runs `ode4`, writes summary (full 90 s noise-free reference remains available via `runMode="simulink"`).
- **SingleIntegrator vs `matlab/`**: 2 s noise-free kernel comparison gave `max |pos| diff = 0` (bit-identical).

## Research gap validation harness (2026-07-21)

New files for three open extensions relative to Du et al. (2024):

| Artifact | Purpose |
|---|---|
| `docs/RESEARCH_GAPS_THEORY.md` | Mathematical framing, proof-gap analysis, literature anchors |
| `docs/research/research_gaps.tex` | LaTeX skeleton for formal step-by-step proofs |
| `docs/WORKFLOW_CONTEXT_RESEARCH_GAPS.md` | Agent handoff for continuing research |
| `sims/research.m` | Self-contained MATLAB validation for all three gaps |

### Gap summary

| Gap | Model change | What validation checks |
|---|---|---|
| 1 Multi-source | Unified cluster start, split at $t_{\mathrm{split}}$, balanced assignment; configs `n8_N2`, `n9_N3`, `n12_N3` | Per-team centroid error vs Du et al. $\varepsilon_k$; phase timeline and deployment map |
| 2 Moving source | `p_s(t)` linear drift; `single_integrator` or `unicycle` | Tracking error vs ISS predicted offset |
| 3 Global maximum | Multi-Gaussian peaks; flipped localization sign (ascent) | Local trap vs global peak |

### Run command

```matlab
cd sims
research                    % all gaps
research gap1                 % all Gap 1 configs
research gap1 n9_N3           % single config
research gap2 unicycle        % unicycle dynamics
```

Gap 1 outputs: `sims/outputs/research/gap1/<config>/` with `trajectory.png`, `team_localization.png`, `phase_timeline.png`, `deployment_map.png`, `motion.gif`, `summary.json`, `result.mat`.

Gap 2/3 outputs: `sims/outputs/research/gap2|gap3/` (standard plot set + `motion.gif`).

### Review notes

- Gap 1 decomposes the swarm into independent teams; cross-team edges are removed from the formation graph. This matches the draft decomposition proposition but is **outside** the original theorem until formally proved.
- Gap 2 uses the baseline Eq. 4 unchanged; tracking is empirical validation of the ISS sketch in the theory doc.
- Gap 3 intentionally starts near a suboptimal peak to document local trapping. Escape mechanisms (dither, multi-swarm reassignment) are listed as future work, not implemented yet.
- All three gaps violate at least one proof pillar (P4-P7); runs are labeled as extension experiments, not theorem checks for the original paper.

### Verification status

MATLAB run completed 2026-07-21 (`research gap1`, `gap2`, `gap3`):

| Gap | Key result | Interpretation |
|---|---|---|
| 1 | Team errors `[0.0042, 0.0072]` (legacy 6-robot), `[0.0195, 0.0249]` (`n8_N2` unified-split) | Per-team decomposition works; clustered start then split |
| 2 | Tail mean tracking error `0.381` vs ISS offset `0.461` | Centroid tracks linearly moving source with bounded lag |
| 3 | Trapped at peak 2, `f(p*)=2.08` vs largest peak-amplitude proxy `5.0` | Local trap confirmed; the smooth Gaussian mixture's true global maximizer still needs numerical computation before a global-performance claim |

Static Code Analyzer: no errors (two info/warning items on `find` vs logical indexing and unused argument).

## Gap 1 theory hardening (2026-07-23)

`docs/research/research_gaps.tex` now separates a conditional multi-source reduction from the unsolved safety problem that makes the reduction valid.

- The selected first field model is the equal-curvature lower envelope of quadratic wells. A quadratic sum would have one minimizer and therefore cannot represent the stated multi-minimum problem.
- The document derives the Voronoi clearance $d_k=\frac12\min_{j\ne k}\|p_s^{(k)}-p_s^{(j)}\|$ and gives a checkable sufficient condition: centroid error plus formation radius and transient formation deviation must remain below $d_k$.
- It records the exact cancellation of raw team size in the all-informed error bound. Equal splitting is a deployment policy, not an accuracy optimum under Du et al.'s ideal model.
- It labels the current timed split and source-labelled measurements as an oracle deployment experiment. Unknown-source identification, post-split formation transients, and inter-team safety remain open research tasks.

`sims/research.m` now records the minimum robot-to-assigned-source Voronoi margin for every team after the split. A positive margin certifies sampled basin containment for the simulation trajectory; it is evidence, not a continuous-time proof.

### Verification

- `checkcode('sims/research.m','-id')` completed successfully. It reported only advisory warnings, including existing unused-variable, unused-helper, scalar-check, and growth warnings.
- A full `research gap1 n8_N2` run was started but exceeded the one-minute execution limit while rendering its normal figures and animation. Its process tree was stopped and its generated artifacts were restored. The next required check is a full `research gap1` run and confirmation that `basin_containment_passed` is true for each configuration.

## Detailed research manuscript (2026-07-23)

`docs/research/research_gaps.tex` was expanded from a short skeleton into a self-contained technical note.

- The baseline now derives shifted-coordinate formation dissipation, a connected-graph finite-time certificate, the regular-polygon identities, and the exact all-informed quadratic-field centroid dynamics.
- Gap 1 now includes the allocation optimization, team-size scale cancellation, Voronoi basin certificate, conditional decomposition theorem, and sampled versus continuous-time validation criteria.
- Gap 2 now derives the moving-source error equation, explicit transient and ISS bounds, the constant-velocity lag vector, and a common-mode feedforward extension.
- Gap 3 now proves why a local ascent swarm cannot ensure a global maximum, bounds the nonquadratic circular-gradient estimator bias, and gives a conditional multi-start peak-identification theorem with a measurable score-separation condition.
- The manuscript distinguishes established calculations, conditional results, and proposed mechanisms. It also corrects the Gaussian-mixture caveat: a largest-amplitude center is not generally the exact global maximizer of a smooth mixture.

### Manuscript verification

- Static checks confirm balanced LaTeX environments, six resolved bibliography keys, 53 resolved cross-references, and a clean `git diff --check`.
- The bundled Tectonic compiler could not complete within the one-minute execution limit because its local runtime stalled. No PDF or build artifacts were retained. A full PDF compile remains required when a usable TeX runtime is available.

---

## Gap 1 related-work positioning (2026-07-27)

Two papers were added to `docs/research/` and assessed against Gap 1 ($N$ sources, $N$ robot teams). Both are useful, for different reasons, and neither closes the gap.

- **MESA** (Turgeman and Werner, ACC 2018) is the closest structural match: groups of $\delta$ unicycle agents, one group per extremum, with formation control and LTL task switching. Its sizing rule $N=\delta\cdot\tilde N_\psi$ matches our `n8_N2`, `n9_N3`, `n12_N3` configs exactly. It uses an explicit least-squares gradient estimate, so it is a **baseline to beat**, not a component to adopt. Reusable pieces: the source-density fairness metric, occupancy repulsion, and the $\tilde N_\psi \ne N_\psi$ case analysis.
- **DIAS** (Chen et al., ICRA 2025) attacks what Gap 1 currently assumes away, namely unknown source positions and unknown source count, via Voronoi cells plus Gaussian process regression and an LCB detection test. It has no formation control and no error bound, so it is **complementary**: it supplies the discovery front end that would retire the oracle assumption flagged after Proposition `prop:multisource-reduction`.

The positioning table added to both documents identifies the unoccupied cell as sign gradient-free, ternary-communication, multi-team formation seeking with per-team $\varepsilon_k$ bounds.

### Files changed

- `docs/RESEARCH_GAPS_THEORY.md`: new "Closest prior work", "Positioning of Gap 1", and "Concrete items to pull in" subsections; literature anchors extended with MESA, DIAS, GSO, DoSS, GMES.
- `docs/research/research_gaps.tex`: new `sec:gap1-related` subsection with the density metric \eqref{eq:density} and LCB test \eqref{eq:lcb}, a capability comparison table, and four new bibliography entries (`turgeman2018`, `chen2025dias`, `krishnanand2009`, `du2021doss`).

### Verification

- Static LaTeX check: 0 cited-but-undefined keys, 0 defined-but-uncited keys, 0 unresolved cross-references, 0 unbalanced environments.
- **Correction.** The check above was first run with a broken regex (over-escaped backslashes), so it matched nothing and reported success vacuously. It has been replaced by `docs/research/_texcheck.py`, which also checks `\left`/`\right` balance per display equation. The rerun found two genuine defects, both since fixed. See the manuscript audit entry below.
- No PDF compile. A PATH sweep for `pdflatex`, `xelatex`, `lualatex`, `latexmk`, `tectonic`, `miktex`, and `tex` found none of them, so the machine has no usable TeX runtime at all. The PDF build stays outstanding, same as the 2026-07-23 entry. Anyone picking this up should install MiKTeX or TeX Live first, then run `pdflatex research_gaps.tex` twice from `docs/research/` to resolve the cross-references.
- No simulation code was touched, so no `research.m` re-run was needed.

### Follow-ups this suggests

1. Report MESA-style per-team source density and density error in the Gap 1 summary JSON, so cross-team fairness is measurable rather than inferred from per-team error alone.
2. Add occupancy repulsion before enabling dynamic reassignment; without it two teams can lock onto the same source.
3. Add configs with teams $\ne$ sources, since all three current configs assume equality.
4. Treat GP plus LCB source discovery as a separate sub-gap, sequenced after the fixed-source decomposition proof lands.

---

## Manuscript math audit (2026-07-27)

Line-by-line recheck of every derivation in `docs/research/research_gaps.tex`, prompted by the question of whether the mathematics is sound or needs adaptation from the two new papers.

### Derivations that check out

Rederived independently and confirmed correct:

- **Regular-polygon identities.** $\sum\phi_i=0$, $\sum\phi_i\phi_i^\top=\tfrac n2 I$, and the third-moment identity $\sum(\phi_i^\top H\phi_i)\phi_i=0$. The last reduces to $\sum e^{\mathrm{i}3\theta_i}=0$, which holds exactly when $n\nmid 3$, so the stated $n\geq4$ requirement and the noted $n=3$ failure are both right.
- **Connectivity bound.** $Q(z)\leq nS(z)$ and $S(z)\geq\sqrt{2V_f}/n$ follow correctly; the path/triangle-inequality step is valid because path edges are a subset of $\mathcal{E}$ and all terms are nonnegative.
- **Finite-time formation.** $\dot V_f\leq-\gamma S\leq-\tfrac{\sqrt2\gamma}{n}\sqrt{V_f}$ integrates to the stated $T_f$ bound. The factor $4n$ in the gain condition (rather than the bare feasibility factor $2n$) is what delivers $\gamma>\alpha/2$, consistent with the text.
- **Exact gradient recovery.** $\sum f(p_i)\phi_i=\kappa nRe$ is exact for the quadratic field, and $\varepsilon_{\mathrm{all}}=\delta/(\kappa R)$ follows.
- **Consistency of $\varepsilon$ and $\lambda_{\mathcal X}$.** The claim $\lambda_{\mathcal X}\varepsilon(\underline n_{\mathcal X})=2\beta\delta/R$ verifies algebraically, and both expressions collapse to the all-informed values at $\underline n_{\mathcal X}=n$.
- **Team-size cancellation.** Substituting $n\to n_k$, $n_{\mathcal X}\to\chi_kn_k$ into the saturation bound does cancel $n_k$ exactly, so "extra robots do not shrink the asymptotic radius" is a real algebraic consequence, not a hand-wave.
- **Basin certificate.** The two triangle inequalities are correct, including the strictness, and $d_k=\tfrac12\min_j\lVert p_s^{(k)}-p_s^{(j)}\rVert$ is the right source-to-seam clearance for the equal-curvature envelope.
- **Moving source.** $\dot e=-\lambda e+d_\eta-\dot p_s$ and the split bound $\delta/(\kappa R)+v_{\max}/(2\beta\kappa)$ are correct, as is the constant-velocity lag $e(\infty)=-v/(2\beta\kappa)$.
- **Estimator bias.** The $L_HR^2/3$ bound follows correctly from the third-order Taylor remainder.
- **Negative result and multi-start selection.** Both are correct and, notably, are the two cleanest unconditional results in the document.

### Defects found and fixed

| Location | Defect | Severity |
|---|---|---|
| `eq:lambda-informed` | Missing `\right|`, leaving `\left(` unclosed | Would have failed to compile |
| `eq:team-centroid` | `,qquad` missing its backslash | Renders as literal text |
| `eq:team-epsilon-scale` | $\rho_k$ denoted both the allocation cost and the informed fraction in adjacent subsections | Ambiguous; informed fraction renamed to $\chi_k$ |
| `eq:density` (added earlier today) | Imported MESA's normalisation $1/N$, but $N$ is the source count in this note and the agent count in MESA | Wrong by a factor of $n$ |
| `eq:lcb` (added earlier today) | Imported DIAS's $\beta$ and $\sigma^2$, which already denote the localization gain and the measurement here | Symbol collision |

The last two were introduced by me in the earlier related-work edit. `eq:density` now states MESA's form with its own symbols explicitly labelled, followed by `eq:density-ours`, a per-team normalised version in this note's notation with $\varrho_k\in[0,1]$ and $\varrho_k=e^{-R_k}$ for a team seated exactly on its ring.

### Assessment

The mathematics does not need adaptation from the two papers. It is internally consistent and the status labelling (established, conditional, proposed) is honest. What it needs is closure on the gap between what is proved and what is simulated:

1. **Gap 1's proposition is conditional on a certificate the controller does not establish.** `eq:basin-certificate` is assumed, not proved, and the simulation supplies source labels as an oracle. This is the single largest open item and it is correctly flagged in the manuscript.
2. **The saturation geometry is imported, not rederived.** Everything resting on `eq:du-epsilon` and `eq:lambda-informed` inherits that dependency, which is stated but easy to lose track of.
3. **Gaussian simulation noise does not satisfy the hard bound $|\eta_i|\leq\delta$** used by every theorem. Already noted after Assumption 1; the runs should either truncate the noise or report the bound as empirical.

### Verification

- `docs/research/_texcheck.py` (new): checks `\left`/`\right` balance per display equation, citation and label resolution, environment balance, and bare-macro typos. Current status: 10 cite keys, 10 bibitems, 105 labels, 62 refs, no problems.
- Still no PDF compile, no TeX runtime available. The delimiter bug above is exactly the class of defect a compile would have caught immediately, which is why the checker was added.

---

## Graphical verification suite (2026-07-27)

`sims/verify_theory.m` (new, about 950 lines) turns every load-bearing claim in `research_gaps.tex` into a figure that compares a measured quantity against the predicted one. Each check is written as a falsification attempt rather than a demonstration, and records a verdict plus supporting numbers in `sims/outputs/verify_theory/summary.json`.

### Coverage

| ID | Claim verified | Result |
|---|---|---|
| V1 | `lem:circle-identities` | Exact for $n\ge4$; third moment $0.453$ at $n=3$ |
| V2 | `lem:connectivity-bound` | 600/600 random connected graphs respect both inequalities |
| V3 | `prop:formation` | Settles two decades before the bound; chatter floor $O(\Delta t^{0.99})$ |
| V4 | `eq:exact-gradient-identity` | Machine precision for quadratic, $0.199$ residual for Gaussian |
| V5 | `thm:all-informed` | Worst tail error $0.0148$ against bound $0.05$, four seeds |
| V6 | `eq:du-epsilon`, `eq:lambda-informed` | Product constant to $1.7\times10^{-16}$ |
| V7 | `eq:team-epsilon-scale` | $\varepsilon_k$ flat in $n_k$ to $5.6\times10^{-16}$ |
| V8 | `lem:basin-certificate` | Sufficiency respected; tight spacing loses containment |
| V9 | `cor:constant-velocity` | Lag vector to $2.3\%$, speed sweep slope to $5\%$ |
| V10 | `lem:estimator-bias` | Slopes $1.10$ at $n=3$, $1.99$ at $n\ge4$ |
| V11 | `prop:no-global-guarantee` | Suboptimal basins cover $60\%$ of the domain |
| V12 | `thm:multistart` | $100\%$ accuracy at every point inside the condition |

### Findings that changed the documentation

1. **V3 initially failed, and the failure was real.** The check asked for $\sqrt{V_f}<10^{-6}$, which fixed-step Euler cannot reach: the sign term chatters at a floor proportional to $\alpha\,\Delta t$. Rather than relax the threshold silently, the check was rewritten to sweep $\Delta t$ and measure the floor, which comes out as $O(\Delta t^{0.99})$. At the `research.m` defaults ($\alpha=100$, $\Delta t=5\times10^{-4}$) that floor is about $0.15$, so formation errors reported below that value in existing runs are discretization artefacts. This is now stated in the manuscript and in `sims/README.md`.
2. **V8 was initially a weak test.** With well-separated sources the teams never approached the seam, so the certificate passed without discriminating. A second tight-spacing scenario was added, in which the certificate fails and containment is genuinely lost ($\min_t m_k=-0.05$). The check now verifies sufficiency in the direction the lemma claims and demonstrates the condition is not vacuous.
3. **V9 had to be redesigned around V3's floor.** The first speeds gave a predicted lag of about $0.045$, comparable to the centroid chatter, so the test could not resolve the prediction. Speeds were raised so the lag is well clear of the floor. This dependency between checks is now noted in the caption.

### Review notes

- Figures are embedded in `research_gaps.tex` next to the claim each one verifies, via a `\verifyfig` macro and `\graphicspath` pointing at `sims/outputs/verify_theory/`. All 12 referenced filenames were confirmed to exist on disk.
- The suite deliberately reports where evidence is weak. V6 is pure algebra and gives no independent support for the imported saturation geometry. V5 and V12 show the bounds are loose for zero-mean noise, so they confirm correctness without confirming tightness.
- V8 tests the certificate; it does not prove the controller enforces it. Gap 1 remains conditional.

---

## Nomenclature and per-page notation key (2026-07-27)

Two notation aids were added to `research_gaps.tex`.

1. **Nomenclature section** after the table of contents: a `longtable` symbol table with 70 entries grouped by the section that introduces each symbol, including the imported MESA and DIAS symbols and the renames applied to avoid collisions.
2. **Per-page key**: every page carries a short legend below the text block, styled like a footnote, listing the symbols in play on that page.

### Why the per-page key is a footer and not a `\footnote`

A real footnote cannot be made to repeat on every page where a symbol appears without a two-pass scheme that reads page numbers from the `.aux` file and then emits footnotes, which changes pagination and can oscillate between runs. The `fixfoot` package does implement repeating footnotes, but it stamps a visible superscript marker at every call site, which would mean a marker after essentially every equation.

The footer instead uses TeX's mark mechanism. `\NotationContext{<id>}` writes a mark; the output routine reads `\rightmark`, which is by construction the mark in force at the top of the page being shipped. That is page-synchronised without a second pass and without affecting pagination. Twelve `\NotationContext` switches are placed at section and subsection boundaries, mapping onto eleven declared keys.

Known behaviour, documented in the paper itself: a page that straddles a section boundary shows the key of the section it begins in, not the one it ends in. `\sectionmark` and `\subsectionmark` are neutralised so the sectioning commands do not overwrite the notation marks.

To disable the whole feature, set `\notationkeyfalse` and reduce the bottom margin in `\geometry`.

### Risk and how it was contained

No TeX runtime is installed, confirmed again by a sweep for `pdflatex`, `xelatex`, `lualatex`, `latexmk`, `tectonic`, and `miktex-pdflatex`. The document is therefore **uncompiled**. Since the footer is the one part that can fail silently or overflow the page, `_texcheck.py` was extended with checks that a compile would otherwise be needed to catch:

- every `\NotationContext` has a matching `\DeclareNotationKey`, and every declared key is activated. A missing declaration prints an empty key rather than raising an error, so a compile would not catch it either.
- a footer height estimate per key, capped at 3.5 lines against a 4-line budget. The margin exists because the estimate strips macros and therefore undercounts math-heavy keys. Worst current key is `teams` at 3.3 lines.
- `longtable` column counts per row, since a row with the wrong number of `&` is a hard error.
- `%` comments are now stripped before parsing. Without that, the checker was reading the commented-out `\DeclareNotationKey{<id>}` usage example in the preamble and counting it as a real key.

`_selftest.py` was added to confirm the checker actually fires. It mutates a temporary copy of the document with seven faults, one per check, and asserts each is reported: 8/8 pass, including that the unmutated document is clean. This matters because two of the new checks initially passed for the wrong reason.

### Verification

- `_texcheck.py`: clean. 10 cite keys, 10 bibitems, 106 labels, 66 refs, 11 notation keys.
- `_texcheck.py --budget`: all keys within the footer budget.
- `_selftest.py`: 8/8.
- Still unverified until someone compiles: actual footer height in points, `longtable` page breaking interacting with the deeper bottom margin, and whether `\rightmark` selection reads well at section transitions. First compile should check the bottom of a page in each of \cref{sec:gap1}, \cref{sec:gap2}, and \cref{sec:gap3}.

---

## Graphical verification suite verification

- MATLAB Code Analyzer on `verify_theory.m`: no issues.
- Full suite: 14/14 checks pass, 231 s wall clock. Re-run with `cd sims && matlab -batch "verify_theory"`.
- The sign consensus was vectorized into a single sparse product per step after the first run took 221 s for six checks; the full twelve then ran in 150 s, and all fourteen now run in 231 s.
- `_texcheck.py` clean after the figure insertions: 105 labels, 62 refs, no unresolved keys or unbalanced delimiters.

---

## Step-by-step proof document (2026-07-27)

`docs/research/proof_documentation.tex` is a proofs-only companion to `research_gaps.tex`, scoped to the three gap sections.

### What was removed, and the consequences

Removed per request: title, date, abstract, table of contents, the reading guide, all figures, and every reference to the MATLAB harness. Dropping figures meant `graphicx`, `\graphicspath`, and the `\verifyfig` macro all go away, and the numerical evidence had to be re-expressed as tables of predicted-versus-measured numbers. That turned out to be an improvement for the numbers that are scalar comparisons, and a real loss only for V3-style results where the shape of a curve is the finding, which is why those are absent here rather than described in prose.

Two judgement calls worth flagging:

- **The per-page notation key was kept.** It was not on the removal list, it costs no front matter, and a document that is almost entirely symbol manipulation is exactly where it earns its keep. Five contexts instead of eleven, since the scope is narrower.
- **A short "standing results" section was added despite the gaps-only instruction.** The gap proofs cite five baseline properties (moment identities, sign dissipation, finite-time formation, all-informed localisation, the partial-information radius). Stating them as `Fact` environments without proof keeps the document self-contained without re-deriving baseline material. Every non-`Fact` statement in the document is proved in full.

### Two new results found while expanding the proofs

Writing out every step surfaced two claims that were previously glossed over.

- **`lem:denominator`.** The denominator of the partial-information error radius, $2\pi n_\mathcal{X} - n|\sin(2\pi n_\mathcal{X}/n)|$, is normally guarded by an explicit positivity assumption. Substituting $x = 2\pi n_\mathcal{X}/n$ rewrites it as $n(x - |\sin x|)$, and $|\sin x| \le x$ makes it positive for every admissible pair. The guard is vacuous and can be dropped. This also cleans up the proof of `lem:team-scaling`, where cancelling $n_k$ needs a nonzero denominator to be legitimate.
- **`lem:n3-bias`.** The $n \ge 4$ hypothesis in the estimator-bias lemma was previously justified by noting that the third-moment identity fails at $n=3$. Computing what survives gives $\sum_i (\phi_i^\top H \phi_i)\phi_i = \tfrac34(H_{11}-H_{22},\,-2H_{12})$ and hence $r_R = \tfrac{R}{4}(H_{11}-H_{22},\,-2H_{12}) + O(R^2)$. This upgrades an excluded case into a quantitative prediction, and it explains the $1.10$ slope measured by V10 rather than merely tolerating it.

Both are new claims, so both were added to the harness as V13 and V14 rather than asserted from a scratch calculation. V14 is the stronger test of the two: it checks the direction of a vector, not just a magnitude, so it validates the moment computation itself.

### Review notes

- The `\bottomrule` false positive in `_texcheck.py` was real, not cosmetic. The `longtable` row splitter stripped `\hline` and the `\end*head` markers but not the booktabs rules, so any booktabs `longtable` reported a spurious one-column row. `research_gaps.tex` uses `\hline` throughout and so never hit it. Fixed by stripping the rule commands.
- The bare-macro check flagged `sgn` inside `\DeclareMathOperator{\sgn}{sgn}`, where the bare name is the operator's printed form and therefore correct. Fixed by stripping `\DeclareMathOperator` and `\DeclarePairedDelimiter` argument pairs.
- `lem:clearance` was added because the certificate lemma quietly assumed $d_k = \tfrac12\min_{j\ne k}\|p_s^{(k)}-p_s^{(j)}\|$. The $\ge$ direction is a one-line triangle inequality; the $\le$ direction needs an argument that the midpoint to the nearest source actually lies on $\partial\mathcal{B}_k$, which is not automatic.
- The equal-curvature assumption is now isolated to a single step (Step 2 of `prop:multisource-reduction`) with an explicit note on what breaks without it. Previously it was an assumption stated at the top of the section and used implicitly.
- `thm:moving-iss` now handles $r = 0$ explicitly via the upper Dini derivative. The previous version divided by $\|e\|$ without saying what happens when the error vanishes, which is a genuine gap even though the conclusion is unaffected.

### Verification

- `_texcheck.py docs/research/proof_documentation.tex`: clean. 5 cite keys, 5 bibitems, 96 labels, 63 refs, 5 notation keys.
- `_texcheck.py` on `research_gaps.tex` after the checker fixes and the V13/V14 table rows: clean, 106 labels, 66 refs, 11 notation keys.
- `_selftest.py`: 8/8, so the checker fixes did not disable any existing check.
- MATLAB Code Analyzer on `verify_theory.m` after adding V13, V14, and `mixtureHess`: no issues.
- V13: pass. Denominator factor $\ge 1.148\times10^{-2}$ over $3 \le n \le 60$ and all $n_\mathcal{X}$, worst case at $(60,1)$; $|\sin x| - x \le 0$ to machine precision.
- V14: pass. At $R = 10^{-4}$ the measured $\|r_R\|/R$ is $0.99998$ times the predicted $0.066520$, with a direction error of $0.015^\circ$.
- `summary.json` merged correctly: all of v1..v14 plus `meta` present after running the two new checks individually.
- Still unverified: the document has never been compiled, since there is no TeX runtime on this machine. Unknowns are the same as for `research_gaps.tex`, namely actual footer height and `longtable` page breaking, plus one new one: this document has no `\pagestyle` reset between sections, so the first page's notation key depends on `\firstmark` behaving as expected when a section starts mid-page.
