# Publishable Extension Directions — Du et al. (2024) Sign Gradient-Free Source Localization + Circular Formation

Status: working analysis. Target venues: IEEE TCNS / TAC, Automatica, CDC / ACC.
Selection bias: directions that admit a NEW theorem (a new Lyapunov certificate, a new epsilon-style
steady-state bound, or a finite/fixed-time convergence-rate result), and that are validatable in the
existing simulation stack before committing to hardware.

This document fuses four independent analysis tracks:

1. Theory proof-gap analysis — for each direction, exactly which step of the Du et al. proof breaks,
   what the technical fix is, and a sketch of the new theorem. Grounded directly in the local proof
   text (`tmp/paper_fulltext.txt`, Theorem 1 both stages, and the epsilon derivation / Remarks 4-7).
2. Web novelty — whether the direction is already occupied in the literature (recovered from the
   deep-research pass; claims that survived 3-vote adversarial verification are marked CONFIRMED).
3. Existing empirical evidence — what the repo's current runs already show that motivates the direction.
4. Validation feasibility — can it be exercised in (i) the existing Python/CoppeliaSim/MATLAB/Simulink
   kinematic stack, (ii) a new ROS2/Gazebo distributed build, (iii) real TurtleBot3 hardware.

---

## 0. The load-bearing pillars of the original proof

Every extension is a stress test on one of these. Knowing which pillar a change leans on tells you
immediately how hard the re-proof is.

Control law (Eq. 4):

    u_i = alpha * sum_{j in N_i} sgn(z_j - z_i)  -  (2 beta / R) * sigma(p_i) * phi(theta_i)
    z_i = p_i - R * phi(theta_i),   phi(theta_i) = [cos(2 pi i / n), sin(2 pi i / n)],   theta_i = 2 pi i / n

Stage 1 — finite-time circular formation. Lyapunov Vc = (1/2) sum ||z_i - z*||^2.
  P1. Undirected-graph symmetry folds the sign double-sum (Eq. 6 -> 7). REQUIRES A = A^T.
  P2. A connectivity / spanning-path argument lower-bounds the folded sum (Eq. 8). REQUIRES connected.
  P3. Gain condition alpha/beta > 4 n f_Dmax / R gives Vc_dot <= -c sqrt(Vc) < 0 => finite time Tc.
      REQUIRES the localization term to be a bounded disturbance during Stage 1.

Stage 2 — asymptotic localization. Lyapunov V = (1/2) ||p* - ps||^2.
  P4. First-order Taylor of f at z* = p* (Eq. 11). REQUIRES f smooth; exactness REQUIRES f quadratic.
  P5. Symmetric-sum identities (sum phi = 0; sum cos 2 theta_i = sum sin 2 theta_i = 0 for n > 2)
      cancel the gradient/Hessian bias, giving -(2 beta / n R) sum f(p_i) phi(theta_i) = -beta grad f(p*)
      (Eq. 14-16). REQUIRES the fixed symmetric circular angle assignment.
  P6. Two-circle-intersection geometric bound on blind robots i in Y (Eq. 18-20). REQUIRES a static source
      and a fixed sensing radius Dmax.
  P7. Steady-state error epsilon = 2 pi n delta / (kappa R (2 pi n_X - n |sin(2 pi n_X / n)|)); with n_X = n,
      epsilon = delta / (kappa R) (Remark 4). REQUIRES bounded noise |eta| <= delta.

Static-source, static-topology, quadratic-field, undirected-graph, single-integrator, 2D. Six standing
idealizations. Each is a paper.

---

## 1. Ranked summary

Rank uses: (novelty x theorem-strength x validatability) / difficulty, with a bonus for directions the
paper itself names as open (time-varying sources, directed topologies, delayed/unsafe comms).

| # | Direction | New theorem | Pillar(s) broken | Difficulty | Validatable now | Novelty |
|---|-----------|-------------|------------------|------------|-----------------|---------|
| A | Fixed/prescribed-time localization (upgrade Stage 2) | Fixed-time bound independent of init | P3 (redesign), P5 | Medium-High | Yes, existing stack | High |
| B | Moving / time-varying source | ISS tracking-ball radius eps_track(||ps_dot||) | P6, P7 | Medium | Yes, trivially | High (paper-named open) |
| C | General strongly-convex field | Curvature-corrected epsilon bound | P4, P5 | Medium | Yes, one field swap | High |
| D | Directed / time-varying topology | Finite-time formation over digraphs | P1, P2 | High | Partial (need digraph layer) | High (paper-named open) |
| E | Communication delay tolerance | Delay-dependent max tau* for convergence | P1, P3 | High | Partial (need delay layer) | High (paper-named open) |
| F | Collision avoidance / safety (CBF) | Forward invariance + retained convergence | P3 | Medium-High | Yes, existing stack | Medium |
| G | Event-triggered / quantized comms | No-Zeno + convergence under triggering | P1, P3 | Medium | Yes, existing stack | Medium |
| H | Stochastic noise | a.s. / expected convergence + eps in expectation | P7 | Medium | Yes (gaussian model exists) | Medium |
| I | 3D / higher-dim formation | Spherical-design sum identities | P5 | Medium | Partial (2D locked in code) | Medium |
| J | Input-saturated / differential-drive dynamics | Convergence under actuator limits | P3 | Medium | Yes (unicycle + Simulink exist) | Medium-Low |

Recommended primary: A or B as the headline theorem, with C as a natural companion result in the same
paper. D and E are strong but are their own papers (both are genuinely hard re-proofs). F-J are good
"second contribution" or workshop/experiment-paper material.

The following sections give, per direction: the exact proof-gap, the technical fix, a new-theorem
sketch, difficulty and coupling risk, novelty status, existing evidence, three-tier validation, and a
skeletal implementation approach.
