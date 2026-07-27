# Research Gap Workflow Context

Status: Phase 1 validation harness (unified-split Gap 1, unicycle mode, config suite) plus Gap 1 related-work positioning.  
Last updated: 2026-07-27.

## What was done

1. **`docs/RESEARCH_GAPS_THEORY.md`** — Mathematical framing for three gaps, plus Gap 1 prior-work comparison against MESA and DIAS.
2. **`docs/research/research_gaps.tex`** — LaTeX with unified-split Gap 1, config table, unicycle note, and `sec:gap1-related` positioning section.
3. **`sims/research.m`** — Gap 1 configs `n8_N2`, `n9_N3`, `n12_N3`; unified cluster start then split; `unicycle` robot model for all gaps.
4. **`sims/verify_theory.m`** — 14 graphical checks (V1..V14), one per lemma/theorem/identity, embedded into `research_gaps.tex` beside each claim. All pass; full suite about 150 s.
5. **Notation aids in the `.tex`** — a Nomenclature `longtable` after the TOC, plus a per-page key in the footer driven by `\NotationContext{<id>}` marks. Eleven keys, twelve switch points. Disable with `\notationkeyfalse`.
6. **`docs/research/proof_documentation.tex`** — a proofs-only companion covering the three gaps with every algebraic step written out, plus numerical verification tables. Deliberately has no title, date, abstract, table of contents, reading guide, figures, or references to the MATLAB harness. Keeps the per-page notation key. Two new results came out of writing it: `lem:denominator` (the $\varepsilon$ positivity guard is vacuous, checked by V13) and `lem:n3-bias` (closed form for the $n=3$ estimator bias, checked by V14).

## Checking the LaTeX without a compiler

There is still no TeX runtime on this machine, so `research_gaps.tex` has never been compiled.

```bash
python docs/research/_texcheck.py                                    # research_gaps.tex by default
python docs/research/_texcheck.py docs/research/proof_documentation.tex
python docs/research/_texcheck.py --budget   # also print per-key footer height estimates
python docs/research/_selftest.py            # confirm the checker fires on injected faults
```

`_texcheck.py` covers delimiter balance, citation and label resolution, environment balance, bare-macro typos, notation-key declaration and activation, footer height budget, and longtable column counts. `_selftest.py` mutates a temporary copy with one fault per check and asserts each is caught, so the checker itself is not trusted blindly.

When a TeX runtime becomes available, the first compile should inspect the bottom of a page inside each of Gap 1, Gap 2, and Gap 3 in both `.tex` files to confirm the footer does not overflow the bottom margin.

## Gap 1 novelty claim (settled 2026-07-27)

Nearest prior work and why it does not close the gap:

| Capability | Du et al. 2024 | MESA 2018 | DIAS 2025 | Gap 1 |
|---|---|---|---|---|
| Circular formation | yes | yes | no | yes |
| Multiple sources | no | yes | yes | yes |
| Gradient-free, ternary comms | yes | no | no | yes |
| Steady-state bound | yes | density only | no | per-team $\varepsilon_k$ |
| Unknown source count | no | partial | yes | future |

MESA is a baseline to beat (gradient-based). DIAS is complementary (discovery front end, no formation). PDFs are in `docs/research/`.

## Gap 1 protocol

- **Unified phase** ($t < t_{\mathrm{split}}$): clustered start, ring graph, global slots, virtual source at mean of all sources.
- **Split**: balanced assignment ($n/N$ per source), per-team graphs and slots.
- **Plots**: `trajectory.png`, `team_localization.png`, `phase_timeline.png`, `deployment_map.png`, `motion.gif` (no per-robot formation plot).

## Run commands

```matlab
cd sims
research                    % all gaps
research gap1                 % n8_N2, n9_N3, n12_N3
research gap1 n9_N3
research gap2 unicycle

verify_theory                 % all 12 theory checks
verify_theory baseline        % v1..v6
verify_theory gap1            % v7, v8
verify_theory v10             % single check
```

## Output locations

```
sims/outputs/research/
  gap1/n8_N2/   team_localization.png, phase_timeline.png, deployment_map.png, ...
  gap1/n9_N3/
  gap1/n12_N3/
  gap1/gap1_summary.json
  gap2/           trajectory.png, formation_error.png, localization_error.png, motion.gif, ...
  gap3/
  research_summary.json
```

## Next steps

1. Run full `research gap1` suite and record metrics in `code_review.md`.
2. Add MESA-style per-team source density $p_{\psi_k}=\frac{1}{n}\sum_{i\in\xi_k}e^{-\|p_s^{(k)}-p_i\|}$ and density error to the Gap 1 summary JSON, as a cross-team fairness metric.
3. Add occupancy repulsion (MESA and DIAS both use a variant) before enabling dynamic reassignment, otherwise two teams can lock onto one source.
4. Add Gap 1 configs where teams $\ne$ sources; all three current configs assume equality.
5. Gap 2 circular source motion preset.
6. Gap 3 escape mechanisms (dither / multi-swarm).
7. Formal proofs in LaTeX.
8. Deferred sub-gap: GP plus LCB source discovery (DIAS style) to retire the "sources known a priori" oracle assumption. Sequence after the fixed-source decomposition proof lands.

## Environment note

No TeX runtime on this machine. A PATH sweep found none of `pdflatex`, `xelatex`, `lualatex`, `latexmk`, `tectonic`, `miktex`, `tex`.

Until one is installed, verify the manuscript with `python docs/research/_texcheck.py`. It checks `\left`/`\right` balance per display equation, citation and label resolution, environment balance, and bare-macro typos. It caught a missing `\right|` in `eq:lambda-informed` that would have broken the build. It is a stopgap, not a substitute for `pdflatex research_gaps.tex` run twice.

## Math status (audited 2026-07-27)

All derivations were rechecked by hand and are correct: polygon identities, connectivity bound, finite-time formation, exact gradient recovery, $\varepsilon$/$\lambda_{\mathcal X}$ consistency, team-size cancellation, basin certificate, moving-source ISS bound, estimator bias, and the multi-start selection theorem. Four defects were found and fixed (see `code_review.md`).

The math does not need adapting from MESA or DIAS. The three real open items are:

1. `eq:basin-certificate` is assumed, not proved. Gap 1's proposition is conditional and the simulation supplies source labels as an oracle. Largest open item. Check V8 tests the certificate but does not prove the controller enforces it.
2. The saturation geometry behind `eq:du-epsilon` is imported from Du et al., not rederived here. Check V6 only confirms internal algebraic consistency.
3. Theorems assume hard-bounded noise $|\eta_i|\leq\delta$; Gaussian simulation noise does not satisfy this. Truncate the noise or report the bound as empirical. `verify_theory.m` uses bounded uniform noise for exactly this reason.

## Numerical caveats found by the verification suite

- **Formation-error floor.** Fixed-step Euler chatters at a level proportional to `alpha * dt`, confirmed `O(dt^0.99)` by V3. At `research.m` defaults (`alpha=100`, `dt=5e-4`) the floor on `sqrt(Vf)` is about `0.15`. Formation errors reported below that are artefacts. Either shrink `dt` or state the floor alongside the result.
- **Seam crossing is real.** V8 shows containment genuinely fails when source spacing is tight relative to the ring radius. Always report the margin `m_k(t)`.
- **Use `n_k >= 4` for nonquadratic fields.** V10 measures the estimator bias exponent at 1.10 for `n=3` versus 1.99 for `n>=4`. The `n9_N3` config uses three-robot teams, so it is fine for the quadratic model but not for Gap 3 style fields.
