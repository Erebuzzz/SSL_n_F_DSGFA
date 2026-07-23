# Research Gap Workflow Context

Status: Phase 1 validation harness (unified-split Gap 1, unicycle mode, config suite).  
Last updated: 2026-07-21.

## What was done

1. **`docs/RESEARCH_GAPS_THEORY.md`** — Mathematical framing for three gaps.
2. **`docs/research/research_gaps.tex`** — LaTeX with unified-split Gap 1, config table, unicycle note.
3. **`sims/research.m`** — Gap 1 configs `n8_N2`, `n9_N3`, `n12_N3`; unified cluster start then split; `unicycle` robot model for all gaps.

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
2. Gap 2 circular source motion preset.
3. Gap 3 escape mechanisms (dither / multi-swarm).
4. Formal proofs in LaTeX.
