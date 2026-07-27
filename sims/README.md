# sims — Standalone MATLAB Simulations

Self-contained scripts for the three robot models in this project. Each file runs end-to-end with no external helpers or JSON configs.

| Script | Model |
|---|---|
| [`SingleIntegrator.m`](SingleIntegrator.m) | Point robots |
| [`Unicycle.m`](Unicycle.m) | Unicycle + feedback linearization |
| [`TurtleBot.m`](TurtleBot.m) | Differential drive (numeric or Simulink) |
| [`research.m`](research.m) | Research-gap validation (multi-source, moving source, global max) |
| [`verify_theory.m`](verify_theory.m) | Graphical verification of the analytical claims |

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

## Theory verification

`verify_theory.m` plots each lemma, theorem, and identity from the research note against what it predicts. Every check is a falsification attempt, so a wrong claim shows up as a visibly wrong curve rather than a silent pass.

```matlab
cd sims
verify_theory              % all 12 checks, about 2.5 minutes
verify_theory baseline     % v1..v6  polygon identities through epsilon/lambda
verify_theory gap1         % v7, v8  team scaling, basin certificate
verify_theory gap2         % v9      moving-source lag
verify_theory gap3         % v10..v12 estimator bias, trapping, multi-start
verify_theory v10          % a single check
```

Outputs to `sims/outputs/verify_theory/`: one PNG per check plus `summary.json` holding the verdict and supporting numbers. Running a subset updates `summary.json` rather than replacing it.

Three results are worth knowing before interpreting any `research.m` run:

- **V3** measures a formation-error floor proportional to `alpha * dt`, confirmed to be `O(dt^0.99)`. At the `research.m` defaults that floor is roughly `0.15`, so smaller reported formation errors are discretization artefacts.
- **V8** shows that closely spaced sources genuinely break basin containment, so the Gap 1 certificate has to be checked per run rather than assumed.
- **V10** shows the circular gradient estimator is `O(R^2)` only for `n >= 4`; at `n = 3` it degrades to `O(R)`.
