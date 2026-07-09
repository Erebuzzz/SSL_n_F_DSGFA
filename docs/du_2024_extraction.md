# Du et al. (2024) — Deep Extraction: Sign Gradient-Free Source Localization + Formation

*IEEE TCNS Vol. 11, No. 1, March 2024. Extraction for resimulation purposes.*

---

## 1. What the paper actually simulated (architecture-wise)

The paper has **two completely separate implementations** that are easy to conflate. Keep them apart:

| | Section IV — "Simulation" | Section V — "Experiments" |
|---|---|---|
| Robot model | Single integrator (eq. 1) | Unicycle, feedback-linearized to a shifted point |
| Count | 6 robots | 4 TurtleBot3 |
| Localization | Assumed perfect state (`p_i(t)` known exactly) | OptiTrack motion capture (ground truth position) |
| Comms | Simulated graph, presumably instantaneous | Sampled-data, `T = 0.1 s` broadcast to all neighbors |
| Noise | Gaussian `η(p_i) ~ N(0, 0.2)` added to measurement `σ(p_i)` | Same noise model, but sourced from OptiTrack jitter |
| Platform | Pure numerical integration (MATLAB/Simulink almost certainly — this is a Chinese controls-theory lab, that's the default toolchain) | Optitrack + router + central PC computing control law + TurtleBot3 executing `(v_i, ω_i)` |
| Formation radius | `R = 2` | `R = 0.5 m` |

So there is **no ROS2/Gazebo anywhere in this paper** — that's Phase 3 of *your* project, not something Du et al. did. Their "real robot" architecture is old-school: centralized perception (OptiTrack) computing a centralized or per-robot control law and streaming velocity commands over WiFi/router to TurtleBot3s that just execute open-loop unicycle commands. This is **not truly distributed** in the experimental section — the graph topology is respected in the control law, but the *sensing* (OptiTrack) is centralized. Worth flagging: the "distributed" claim in the title really only holds at the control-law/communication level, not the perception level, in their hardware demo.

---

## 2. Core mathematical model

### 2.1 Robot dynamics (single integrator, used for the theory + Section IV)

$$\dot p_i(t) = u_i(t), \quad i \in \mathcal V,\quad p_i, u_i \in \mathbb R^2$$

### 2.2 Signal / source model (inverse-square law)

$$f(z) = \kappa \|z - p_s\|^2, \quad z\in\mathbb R^2,\ \kappa>0$$

Note: despite the paper's text invoking the "inverse-square law" motivation, the actual `f` used is a **quadratic bowl** centered at the source, not a literal `1/d²` field. This is a common simplification (it's smooth, convex, has one global min at `p_s`, and its gradient is linear — nice for the Taylor-expansion step in the proof). If you want a physically literal inverse-square field for your own resimulation, that's a genuine deviation you could explore (see Section 6 below).

### 2.3 Measurement model (bounded sensing range + additive noise)

$$
\sigma(p_i) =
\begin{cases}
f(p_i) + \eta(p_i), & \|p_i - p_s\| < D_{\max} \\
f_{D_{\max}}, & \text{otherwise}
\end{cases}
$$

where $f_{D_{\max}} = \kappa D_{\max}^2 + \delta$ is a *saturation value* (note: uses the small constant $\delta$, same symbol as the noise bound — mildly overloaded notation in the paper, watch for this if you're translating to code).

- $\mathcal X = \{i : \|p_i-p_s\| < D_{\max}\}$ — "informed" robots (can sense source)
- $\mathcal Y = \mathcal V \setminus \mathcal X$ — "blind" robots (only get the saturation value)
- $n_\mathcal X + n_\mathcal Y = n$

### 2.4 Assumptions (exact, load-bearing)

**Assumption 1**: Graph $\mathcal G$ undirected and connected.

**Assumption 2**: $|\eta(p_i)| \le \delta$ for some $\delta>0$, and at least one robot always satisfies $\|p_i - p_s\| < D_{\max}$ (i.e., $n_\mathcal X \ge 1$ always).

These two are the *entire* sufficient condition set — no assumptions on initial positions, no boundedness of $u_i$, no rigidity assumptions (explicitly called out in Remark 3 as **not needed**, unlike distance-based formation control).

### 2.5 Formation target

Circular formation with angular slots $\theta_i = 2\pi i/n$, radius $R \le D_{\max}$:

$$\phi(\theta_i) = [\cos\theta_i, \sin\theta_i]^T$$

Objective:
$$\lim_{t\to\infty}\|p_i(t) - R\phi(\theta_i) - p^*(t)\| = 0 \ \ \forall i, \qquad \lim_{t\to\infty}\|p^*(t)-p_s\| < \varepsilon$$

where $p^*(t) = \frac{1}{n}\sum_i p_i(t)$ is the formation centroid.

---

## 3. The control law (the actual algorithm to implement)

$$
u_i(t) = \alpha \sum_{j\in\mathcal N_i} \text{sgn}\big((p_j(t)-R\phi(\theta_j)) - (p_i(t)-R\phi(\theta_i))\big) \;-\; \frac{2\beta}{R}\,\sigma(p_i(t))\,\phi(\theta_i)
$$

Two decoupled terms:

1. **Formation term** (consensus-like, sign-based): drives the *shifted* states $z_i = p_i - R\phi(\theta_i)$ toward consensus. Because it's `sgn(...)` applied component-wise to a vector difference, only 3 values per channel are transmitted: `{-1, 0, 1}` — this is the "ternary communication" claim (2 bits/channel, 2 channels per neighbor pair = the "reduced cost" feature).
2. **Localization term**: a *gradient-free* pseudo-gradient step. Since $\nabla f$ isn't measurable, they approximate it using only the scalar reading $\sigma(p_i)$ times the known unit direction $\phi(\theta_i)$ (the robot's own angular slot on the circle). This only works *because* the robots are already arranged symmetrically around the circle — that symmetry is what cancels out the higher-order Taylor terms in the proof (eq. 14–15, using $\sum \phi(\theta_i)=0$ and even distribution of $\cos^2,\sin^2$ terms). This is the deep trick of the paper: **you get a gradient estimate for free from geometric symmetry, without ever differencing measurements between robots.**

Key coupling condition (Theorem 1, Remark 6):
$$\frac{\alpha}{\beta} > \frac{4n f_{D_{\max}}}{R}$$

This is a **hard gain-ratio constraint** — formation gain must dominate localization gain by this much or the proof (specifically the Lyapunov derivative bound in eq. 7–8) doesn't go through. This is the single most important parameter relationship to respect if you resimulate; violate it and you may still converge in practice (sufficient, not necessary condition) but you lose the theoretical guarantee and can plausibly get oscillation/non-convergence for adversarial noise draws.

---

## 4. Theory required (full toolchain)

### 4.1 Graph theory
- Undirected connectivity (Assumption 1) is enough — no algebraic connectivity (Fiedler value) threshold appears explicitly, unusual for consensus proofs; it's absorbed into the generic path-length argument in eq. (8) instead of a Laplacian eigenvalue bound. If you want to relate convergence *rate* to topology, you'd need to bring in $\lambda_2(L)$ yourself; the paper only proves finite-time convergence of formation error without an explicit rate constant tied to graph structure beyond path length $h$.

### 4.2 Lyapunov analysis — two separate Lyapunov functions, sequential (singular perturbation-style) argument

**Stage 1 — formation consensus** (finite-time):
$$V_c(t) = \frac12\sum_i \|z_i(t)-z^*(t)\|^2,\qquad z_i = p_i - R\phi(\theta_i),\ z^* = p^*$$

Key inequality trick: relate $\|z_i - z^*\|$ to the max pairwise distance via graph connectivity (a shortest-path chaining argument, eq. 8), then bound the sign-term contraction using
$$\|z_i-z_j\| \le -(z_i-z_j)^T\text{sgn}(z_j-z_i)$$
(true because $\text{sgn}$ of a vector applied component-wise satisfies $x^T\text{sgn}(x) = \|x\|_1 \ge \|x\|_2$ up to sign — the paper is a bit loose here notation-wise but it holds elementwise). This yields
$$\dot V_c \le 0 \text{ and, on } V_c\neq0,\ \frac{\dot V_c}{\sqrt{V_c}} \le -c < 0$$
i.e. a **finite-time (not just asymptotic) convergence** bound via the classic $\sqrt{V}$ trick (this is the standard finite-time Lyapunov lemma: if $\dot V \le -c\sqrt V$, then $V$ hits zero in finite time $\le 2\sqrt{V(0)}/c$).

**Stage 2 — source localization** (asymptotic, only valid *after* $t\ge T_c$ once circular formation is locked in):
$$V(p^*(t)) = \frac12\|p^*(t)-p_s\|^2$$

Requires:
- **First-order Taylor expansion** of $f$ around $p^*$ evaluated at each $p_i = p^*+R\phi(\theta_i)$ (eq. 11).
- **Symmetric-sum identities** for equally-spaced angles: $\sum_i \phi(\theta_i)=0$, and for $n>2$, $\sum_i\cos(2\theta_i)=\sum_i\sin(2\theta_i)=0$. These kill the gradient-bias and the full Hessian contribution respectively (eq. 14–15) — **this is why $n>2$ matters and why formation must be exactly circular with equally spaced slots**; a non-uniform angular spacing breaks the cancellation and reintroduces Hessian-dependent bias.
- A **geometric argument** (eq. involving $p_A, p_B, \omega, \phi$) to bound the contribution from "blind" robots (those in $\mathcal Y$, which read the saturated value $f_{D_{\max}}$ instead of the true field) — this is genuinely paper-specific geometry (intersection of two circles) and is the most tedious part to re-derive if you want to sanity check it symbolically.
- Final steady-state bound:
$$\varepsilon = \frac{2\pi n\delta}{\kappa R\left(2\pi n_\mathcal X - n\left|\sin\left(\frac{2\pi n_\mathcal X}{n}\right)\right|\right)}$$

**Sanity-check behaviors you should verify numerically in your sim** (these are explicit remarks in the paper — great as unit tests):
- Remark 4: if $n_\mathcal X = n$ (all robots informed), $\varepsilon \to \delta/(\kappa R)$ — a clean closed form, easy regression test.
- Remark 5: $\delta\to0 \Rightarrow \varepsilon\to0$.
- $\varepsilon$ increases with $n$ and decreases with $R$ and $\kappa$ — you should be able to sweep these and see monotonic trends; if your sweep script (`swarm_param_sweep.m`) doesn't show monotonicity in $\varepsilon$ vs $R$, that's a bug flag.

### 4.3 Unicycle feedback linearization (needed for hardware/TurtleBot3 realism, i.e. your Phase 2/3)

Unicycle model:
$$\dot x_i = v_i\cos\theta_i,\quad \dot y_i = v_i\sin\theta_i,\quad \dot\theta_i = \omega_i$$

Shift the control point off-center by $r$ along heading (standard trick to make unicycle "look like" a single integrator from the shifted point's perspective):
$$s_i = p_i + r\begin{bmatrix}\cos\theta_i\\\sin\theta_i\end{bmatrix}$$

Then:
$$\dot s_i = \begin{bmatrix}\cos\theta_i & -r\sin\theta_i\\ \sin\theta_i & r\cos\theta_i\end{bmatrix}\begin{bmatrix}v_i\\\omega_i\end{bmatrix} = f_i$$

where $f_i$ is exactly the single-integrator control law (4) evaluated at $s_i$. Invert to recover actual actuation:
$$\begin{bmatrix}v_i\\\omega_i\end{bmatrix} = \begin{bmatrix}\cos\theta_i & \sin\theta_i\\ -\frac1r\sin\theta_i & \frac1r\cos\theta_i\end{bmatrix} f_i$$

This inversion is **singular at $r=0$** — you must pick $r>0$ (paper used $r=2.5\,\text{cm}$). This is the exact mechanism you already have flagged in your memory as "unicycle feedback linearization for Phase 2" — confirmed, this is precisely eq. block in Section V, nothing more exotic than that.

### 4.4 Signum function subtleties
- $\text{sgn}(0) := 0$ is the implicit convention (needed or the sum in eq. 4 is undefined at consensus).
- Component-wise sign of a 2D vector difference, not a normalized direction vector — i.e. `sgn([dx,dy]) = [sgn(dx), sgn(dy)]`, **not** `diff/||diff||`. This is a very easy bug to introduce in code (people often reach for the normalized version by habit). Double check `swarm_sim.m` uses component-wise sign, not unit vector.

---

## 5. Full parameter/config table (everything explicit in the paper)

| Parameter | Section IV (sim) | Section V (hardware) | Meaning |
|---|---|---|---|
| $n$ | 6 | 4 | robot count |
| Topology | Fig. 1 (undirected, connected — exact edge list not given numerically, only graphically) | not specified beyond "communication realized in sampled-data manner" | graph $\mathcal G$ |
| $p_s$ | $[5.5, 5.5]^T$ | not given numerically | source location |
| $f(z)$ | $\|z-p_s\|^2$ (i.e. $\kappa=1$) | same field assumed | signal function |
| $\eta(p_i)$ | $\mathcal N(0, 0.2)$ | $\mathcal N(0,0.2)$ (from OptiTrack) | measurement noise |
| $R$ | 2 | 0.5 m | formation radius |
| $D_{\max}$ | 12 | not given | sensing range |
| $r$ | n/a (single integrator) | 2.5 cm | unicycle shift offset |
| $T$ (comms sample period) | not specified (implies continuous/instant) | 0.1 s | broadcast interval |
| $\alpha,\beta$ | **not given numerically** — only the ratio constraint $\alpha/\beta>4nf_{D_{\max}}/R$ | not given | control gains |
| Sim duration | plots run to ~60 s (Fig. 4) / ~8 s (Fig. 3, formation converges much faster) | ~40 s (formation, Fig. 9) / ~80 s (localization, Fig. 10) | — |

**Important gap**: the paper never publishes actual $\alpha,\beta$ values or the adjacency matrix numerically — only the qualitative ratio bound and a topology *picture* (Fig. 1). Any resimulation has to reverse-engineer or choose these; that's a legitimate design decision point, not something you're missing from a careless read.

Compute $f_{D_{\max}}$ for the sim case: $\kappa=1, D_{\max}=12 \Rightarrow f_{D_{\max}} = 144 + \delta \approx 144$ (using $\delta=0.2$ noise-bound proxy, though strictly $\delta$ here is the *noise bound* constant from Assumption 2, not literally the Gaussian std — another notation overload: they use one Gaussian with std 0.2 in simulation but the *proof* wants a hard bound $|\eta|\le\delta$, which a Gaussian technically never satisfies almost-surely. In practice they're using $\delta$ as roughly $3\sigma$ or just accepting the theorem as an "arbitrarily small ball" empirical approximation. **Flag this as a theory/practice mismatch**: Gaussian noise violates Assumption 2's hard bound in the tails. If you want to be rigorous, use truncated Gaussian or uniform noise in $[-\delta,\delta]$ instead for a resimulation that actually satisfies the stated hypotheses.

Given $R=2$: constraint becomes $\alpha/\beta > 4\cdot6\cdot144/2 = 1728$. That's a huge ratio — formation gain must be ~1700x localization gain in the worst case theoretical bound. In practice (since it's only sufficient, not necessary) they almost certainly used something far smaller and it still worked, illustrating that the bound is conservative. Worth verifying empirically in your sweep: does the system still converge well below $\alpha/\beta=1728$? Almost certainly yes — good discussion point for a "theory vs practice" section in your own writeup.

---

## 6. Alternative simulation architectures (paths not taken by the paper)

### 6.1 Pure numerical / MATLAB (what you're already doing)
Your existing setup (`swarm_sim.m` animated + `swarm_param_sweep.m` headless) is architecturally faithful to Section IV. Two refinements worth adding given the extraction above:
- Enforce noise as bounded (truncated/uniform), not raw Gaussian, if you want a version that literally satisfies Assumption 2.
- Add an explicit adjacency matrix input rather than hardcoding Fig. 1's topology — makes it trivial to test **directed** or **switching** graphs later (see 6.4).

### 6.2 Python-based (SciPy/NumPy `solve_ivp`, or discrete Euler)
Same math, different toolchain. Advantages: easier to hook into ROS2 later (same language), easier to vectorize the $n$-robot sign-sum with broadcasting, easier to add live matplotlib animation or export to your existing `Confluxa`-adjacent design sensibilities if you want the deliverable prettier. This is a near-zero-cost alternative path — literally the same equations, just implemented in `numpy` with `sign()`, and Euler-integrated with your already-validated `dt=0.15/alpha` overshoot fix from your prior Phase 1 work (that fix generalizes cleanly here too, since the instability mechanism — sign discontinuity + finite step causing chatter/overshoot — is identical).

### 6.3 ROS2/Gazebo (your planned Phase 3, genuinely new relative to the paper)
This is the biggest legitimate extension beyond what Du et al. did, since **they never used a physics simulator at all** — Section IV is a kinematic-only numerical integration, and Section V used real hardware with an external centralized MoCap system, not a self-contained physically-simulated stack. A ROS2/Gazebo TurtleBot3 implementation would be a genuine novel contribution path:
- Each robot as a Gazebo-spawned TurtleBot3 (Harmonic + Jazzy, matches your existing WSL2 setup).
- Replace OptiTrack with either (a) ground-truth Gazebo pose topics (cheating but useful for first-pass validation, mirrors the paper's centralized approach) or (b) a genuinely decentralized alternative — see 6.5.
- Replace `T=0.1s` sampled broadcast with actual ROS2 topics/DDS between per-robot nodes, each robot only subscribing to its graph-neighbors' `/z_i` topic — this makes the "distributed" architecture *actually* distributed at the software level, unlike the paper's own hardware demo.
- Control law runs per-robot as its own ROS2 node computing $u_i$ from local topic subscriptions, then feeding through the unicycle inversion (Section 4.3 above) into `cmd_vel`.

### 6.4 Directed / switching topology (explicitly flagged by the paper as unsolved future work)
The paper's Conclusion explicitly lists **directed network topologies** as future work. The current proof leans on undirected symmetry (e.g., "since the graph is undirected" is invoked directly in eq. 6-7 to flip a double sum). A directed-graph resimulation would need either:
- A balanced digraph assumption (weight-balanced Laplacian) to preserve the same double-sum trick, or
- A different Lyapunov certificate entirely (e.g., using the left eigenvector of the digraph Laplacian as a weighting, standard in directed consensus literature).
This is simulate-able today without waiting on new theory — you can just test empirically whether their sign-consensus formation term still converges under a fixed directed topology, then compare against the undirected case. That's a clean, self-contained experiment your parameter sweep script could add: `topology_type = {'undirected','directed_balanced','directed_unbalanced'}`.

### 6.5 Delayed / lossy / adversarial communications (also explicit future work)
Paper's Conclusion lists this directly. Cheap simulation extensions:
- **Communication delay**: buffer neighbor state by $\tau$ steps before use in the sign-sum.
- **Packet loss**: randomly drop the neighbor update with probability $p_{loss}$, holding last-known value (zero-order hold) — very easy to bolt onto your existing MATLAB loop.
- **Quantization beyond ternary**: since the whole point of the paper is *ternary* (3-level) communication as a feature, you could explore the tradeoff curve of communication levels (2-bit ternary vs 4-level vs full continuous) versus localization error $\varepsilon$ — this is a genuinely interesting sweep the paper never runs, and directly quantifiable using their own error formula, since $\varepsilon$ in the theorem doesn't actually depend on communication precision (only on $n_\mathcal X, R,\kappa,\delta,n$) — meaning empirically you'd expect quantization level to mostly affect *convergence speed*, not steady-state error. Testing that hypothesis would be a nice original contribution.
- **Time-varying source** $p_s(t)$: also explicit future work. You'd need to re-derive Stage 2's Lyapunov analysis since $\dot p_s \ne 0$ breaks the clean $V=\frac12\|p^*-p_s\|^2$ derivative — a tracking-error term appears. Cheapest empirical test: just move $p_s$ slowly (constant velocity) and see if $p^*$ tracks it with a bounded lag, without re-deriving the bound formally.

### 6.6 Higher-dimensional (3D) extension via "balanced formations"
Remark 7 explicitly points to reference [23] (Chen, Ren, Cao — "Surrounding control in cooperative agent networks") for a 3D generalization technique. Concretely this means replacing the 2D circular parametrization $\phi(\theta_i)=[\cos\theta_i,\sin\theta_i]^T$ with a **balanced spherical/3D configuration** (e.g., points on a sphere summing to zero, analogous to the $\sum\phi(\theta_i)=0$ identity that does all the heavy lifting in the 2D proof). This is a legitimate, nontrivial extension the paper gestures at but doesn't do — a good "stretch goal" for your simulation if you want a genuinely novel angle for something publishable, since 3D source localization (e.g. gas leak plume in 3D, or acoustic source with altitude) is more practically relevant than the flat-plane case.

### 6.7 Literal inverse-square field instead of quadratic bowl
As noted in 2.2, $f(z)=\kappa\|z-p_s\|^2$ is *not* actually $1/d^2$ despite the "inverse-square law" motivating language in the intro. A resimulation using a literal $f(z) = \kappa/\|z-p_s\|^2$ (or with a regularizer to avoid blowup at $z=p_s$) would break the clean Taylor-expansion cancellation in eq. 11–16 — the Hessian terms would no longer symmetrically cancel the same way — so this would require either (a) empirical-only testing (no closed-form $\varepsilon$ guarantee), or (b) a redone symbolic derivation. Good candidate for a "does the theorem's error bound still approximately hold under a more physically accurate field" numerical experiment.

---

## 7. Concrete alternative physical hardware paths (not just OptiTrack + TurtleBot3)

The paper's own hardware setup (Fig. 5) uses OptiTrack for ground truth + a router for the sampled-data comms + TurtleBot3 with a UWB module for something (unclear from the text exactly what role UWB plays here, unless it's just used for supplementary ranging or was part of unused infra — the paper doesn't explain the UWB module's role in Section V despite showing it in Fig. 7). Alternatives for a from-scratch resimulation that is *actually* distributed (no centralized MoCap):

- **UWB-only relative ranging** (e.g., DecaWave/Qorvo modules): each robot estimates neighbor relative position via UWB TDOA/TWR instead of MoCap — genuinely decentralized, matches the "no global position info" ethos referenced in their own citation [7].
- **Onboard camera + AprilTag/fiducial markers** for relative pose between neighbors, no external infrastructure at all.
- **Simulated-only via Gazebo ground truth** for a first-pass validation before committing to real UWB hardware (cheapest path, matches your existing WSL2/ROS2 Jazzy/Gazebo Harmonic environment already set up from Phase 1).

---

## 8. Summary checklist for your resimulation

- [ ] Confirm `sgn()` applied component-wise, not normalized-direction.
- [ ] Confirm noise model: switch to bounded (uniform/truncated) if you want literal Assumption 2 compliance.
- [ ] Confirm gain ratio $\alpha/\beta$ relative to the (likely very conservative) theoretical bound $4nf_{D_{\max}}/R$; empirically test how far below this bound you can go and still converge.
- [ ] Verify Remark 4 closed form ($n_\mathcal X=n \Rightarrow \varepsilon=\delta/\kappa R$) as a regression test.
- [ ] Verify monotonicity of $\varepsilon$ vs. $n$, $R$, $\delta$, $n_\mathcal X$ in your param sweep.
- [ ] Decide: quadratic-bowl field (paper's actual field) vs literal inverse-square field (paper's stated motivation) — these diverge, pick deliberately.
- [ ] For Phase 2 unicycle work: use $r>0$ always, never let the inversion matrix hit $r=0$.
- [ ] For Phase 3 ROS2/Gazebo: decide whether to keep centralized ground-truth localization (matches paper) or go fully decentralized with UWB/AprilTag (exceeds paper, more publishable).
- [ ] Consider directed-graph, delayed-comms, lossy-comms, and time-varying-source variants as clearly labeled "extension" experiments — the paper's own Conclusion names exactly these three as open problems, so results here would be a legitimate contribution.
