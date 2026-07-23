# sims — Standalone MATLAB Simulations

Self-contained scripts for the three robot models in this project. Each file runs end-to-end with no external helpers or JSON configs.

| Script | Model |
|---|---|
| [`SingleIntegrator.m`](SingleIntegrator.m) | Point robots |
| [`Unicycle.m`](Unicycle.m) | Unicycle + feedback linearization |
| [`TurtleBot.m`](TurtleBot.m) | Differential drive (numeric or Simulink) |
| [`research.m`](research.m) | Research-gap validation (multi-source, moving source, global max) |

**Run guide:** [`RUN_GUIDE.md`](RUN_GUIDE.md)

```matlab
cd sims
research                    % all gaps
research gap1                 % Gap 1: n8_N2, n9_N3, n12_N3
research gap1 n9_N3           % single Gap 1 config
research gap2 unicycle        % unicycle dynamics
```

**Research theory:** [`docs/RESEARCH_GAPS_THEORY.md`](../docs/RESEARCH_GAPS_THEORY.md)

Gap 1 outputs per config under `sims/outputs/research/gap1/<config>/`:
`trajectory.png`, `team_localization.png`, `phase_timeline.png`, `deployment_map.png`, `motion.gif`, `summary.json`, `result.mat`.
