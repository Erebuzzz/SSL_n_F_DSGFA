# Deep Extraction: "Simultaneous Source Localization and Formation via a Distributed Sign Gradient-Free Algorithm"
Du, Chen, Xiang, Guo, Chen — IEEE TCNS, Vol. 11, No. 1, March 2024

---

## 1. What kind of "simulation" this actually is

There are **two separate experimental pipelines** in the paper, not one:

| Stage | Section | Platform | Purpose |
|---|---|---|---|
| **Numerical simulation** | Sec. IV | Pure math/ODE simulation (single-integrator dynamics, no physics engine, no ROS) — almost certainly run in **MATLAB/Simulink** (standard for this control-theory lab group; graph topology plotted as a discrete graph, Lyapunov-style plots of "distance vs time") | Validate Theorem 1 on an idealized point-robot model |
| **Physical experiment** | Sec. V | **6-DOF OptiTrack motion-capture system + TurtleBot3 ground robots**, unicycle kinematics, UWB module mounted on each robot, sampled-data communication at T = 0.1 s | Validate that the control law survives real actuation noise, sampling, and a non-holonomic (unicycle) platform |

There is **no Gazebo/ROS/Webots-style physics simulator** used anywhere in the paper — the "simulation" in Sec. IV is a bare numerical integration of the closed-loop ODEs (1)+(4), and the "experiment" in Sec. V is real hardware with OptiTrack as ground truth. This is an important architectural fact if you want to reproduce it: **you have to build the simulator yourself**, because the paper doesn't specify one — it specifies only the dynamics and control law, which is actually good news for you since it means you have freedom in choosing a re-implementation stack (Section 6 below).

---

## 2. System / control architecture

### 2.1 Agent model (Sec. II-C)
Single-integrator kinematics for each of $n$ robots in $\mathbb{R}^2$:
$$\dot p_i(t) = u_i(t), \quad i \in \mathcal V$$

### 2.2 Sensing model — inverse-square signal field
$$f(z) = \kappa \|z - p_s\|^2$$
(note: despite being called "inverse-square law" in the text, the function used is actually a **quadratic bowl** centered at the source, not a literal $1/d^2$ falloff — this is a common substitution because a quadratic has a clean, globally unique minimum and a linear gradient, which makes the Taylor-expansion proof in Sec. III tractable.)

Measured signal at robot $i$ (with a **finite sensing range** $D_{max}$ and **noise clipping**):
$$
\sigma(p_i) = \begin{cases} f(p_i) + \eta(p_i), & \|p_i - p_s\| < D_{max} \\ f_{D_{max}} = \kappa D_{max}^2 + \delta, & \text{otherwise} \end{cases}
$$

- $\mathcal X$ = "informed" robots (inside sensing range), $|\mathcal X| = n_X$
- $\mathcal Y$ = "uninformed" robots (outside range, robot literally can't measure the field and just gets a saturated ceiling value), $|\mathcal Y| = n_Y$
- $n_X + n_Y = n$

### 2.3 Communication architecture — the "ternary" part
Instead of exchanging full continuous position vectors, robots exchange **only the sign of the pairwise displacement error**:
$$\text{sgn}\big((p_j - R\phi(\theta_j)) - (p_i - R\phi(\theta_i))\big)$$
Each component of this signed vector is one of $\{-1, 0, +1\}$ → **2 bits per scalar per neighbor per timestep**. This is the "distributed sign gradient-free" and "ternary communication" claim in the title/abstract.

### 2.4 Control law (Eq. 4) — the core object to re-implement
$$
u_i(t) = \alpha \sum_{j \in \mathcal N_i} \text{sgn}\big((p_j(t) - R\phi(\theta_j)) - (p_i(t) - R\phi(\theta_i))\big) \;-\; \frac{2\beta}{R}\,\sigma(p_i(t))\,\phi(\theta_i)
$$

where:
- $\phi(\theta_i) = [\cos\theta_i, \sin\theta_i]^T$, with $\theta_i = 2\pi i/n$ — **fixed, pre-assigned angular slot** for robot $i$ on the target circle (this is a *centralized labeling assumption* baked into the algorithm — every robot must already know its own index $i$ and $n$)
- $\alpha$ = formation gain (drives sign-consensus of shifted positions $z_i = p_i - R\phi(\theta_i)$)
- $\beta$ = localization gain (drives the *centroid* toward the source)
- $R$ = radius of the desired circular formation ($0 < R \le D_{max}$)

Term 1 = discrete, bang-bang-like formation control (graph consensus on shifted coordinates using only sign information).
Term 2 = gradient-free ascent/descent surrogate: since $\nabla f$ is unmeasurable/noisy, $\sigma(p_i)\phi(\theta_i)$ is used as a *proxy gradient direction*, exploiting the geometric symmetry of $n$ robots arranged uniformly around a circle.

### 2.5 Why this architecture is clever (and what it depends on)
The proof shows (via Eq. 14–16) that summing $\sigma(p_i)\phi(\theta_i)$ over a **perfectly symmetric circular formation** cancels the gradient-orthogonal terms and the Hessian term exactly, leaving a clean estimate of $-\beta \nabla f(p^*)$ — i.e., **the circular formation geometry itself is what allows gradient-free operation.** This is why the paper enforces "formation first, then localization" (and indeed Figs. 3–4 show formation error vanishing before localization error). This is a structural dependency you must preserve in any re-simulation: *break the circular symmetry (e.g., irregular polygon, non-uniform $\theta_i$) and the gradient estimate becomes biased.*

---

## 3. Assumptions (explicit, load-bearing)

| # | Assumption | Where used |
|---|---|---|
| **Assumption 1** | Graph $\mathcal G$ is **undirected and connected** | Needed for consensus argument (Vc Lyapunov proof) and for $\sum \phi(\theta_i)=0$-type cancellations to propagate network-wide |
| **Assumption 2** | Perturbation bounded: $|\eta(p_i)| \le \delta$, and **at least one robot** is within $D_{max}$ of the source at all times | Guarantees $n_X \ge 1$; if this fails the whole team is "blind" and localization cannot start |
| (implicit) | $n > 2$ robots (used in the trig identities $\sum \cos 2\theta_i = 0$ etc., which require $n>2$) | Fourier-type cancellation in Eq. (14)–(15) |
| (implicit) | $\alpha/\beta > 4nf_{D_{max}}/R$ (Theorem 1's core gain condition) | Formation-gain-dominance condition — formation dynamics must be "faster/stronger" than localization dynamics or the two objectives fight each other |
| (implicit) | Source is static ($p_s$ constant) | Entire Lyapunov proof assumes $\dot p_s = 0$; explicitly listed as future work to relax |
| (implicit) | Full state (position) available to compute own $z_i$, and index $i$/angle $\theta_i$ known a priori | Not discussed as a limitation but is a real implementation assumption — some kind of self-localization/labeling scheme is required underneath this algorithm |
| (implicit) | Synchronous or at least reliable (non-lossy) sign communication | The "delayed and unsafe communications" is explicitly flagged as *not handled* (see Sec. 7 below) |

---

## 4. Parameters and configuration actually used

### 4.1 Numerical simulation (Sec. IV, Figs. 1–4)
| Parameter | Value |
|---|---|
| Number of robots $n$ | 6 |
| Network topology | Fixed graph shown in Fig. 1 (undirected, connected — exact edge list not given numerically, only pictorially) |
| Signal function | $f(z) = \|z - p_s\|^2$ (i.e. $\kappa = 1$) |
| Source location $p_s$ | $[5.5, 5.5]^T$ |
| Measurement noise | $\eta(p_i) \sim \mathcal N(0, 0.2)$ (Gaussian, **not the bounded noise model of Assumption 2** — a practical relaxation the authors make without discussion) |
| Formation radius $R$ | 2 |
| Max sensing distance $D_{max}$ | 12 |
| Gains $\alpha, \beta$ | **Not numerically specified in the text** — only the ratio condition $\alpha/\beta > 4nf_{D_{max}}/R$ is given. You must choose values satisfying this, e.g. compute $f_{D_{max}} = \kappa D_{max}^2 + \delta \approx 144$, so $\alpha/\beta > 4(6)(144)/2 = 1728$ — a **very large ratio**, meaning $\alpha \gg \beta$ in practice. |
| Simulation horizon | ~60 s (Fig. 4), formation converges by ~5–8 s (Fig. 3) |

### 4.2 Physical experiment (Sec. V, Figs. 5–10)
| Parameter | Value |
|---|---|
| Robots | TurtleBot3 (differential-drive / unicycle) |
| Ground truth / comms backbone | OptiTrack motion capture + UWB module on each robot |
| Formation radius $R$ | 0.5 m |
| Offset point radius $r$ (control point shift for feedback linearization) | 2.5 cm |
| Measurement noise | $\eta(p_i) \sim \mathcal N(0, 0.2)$ (same as simulation) |
| Communication scheme | Sampled-data, every $T = 0.1$ s each robot broadcasts to all neighbors |
| Number of robots | 4 (labeled tb3_0…tb3_3 in Fig. 8) — note this **differs from the 6 used in Sec. IV**, and the topology also isn't explicitly re-stated |
| Convergence | Formation error → 0 by ~20–30 s (Fig. 9); localization error → 0 by ~40–50 s (Fig. 10) |

---

## 5. Required theory, in full, with all the transformations

### 5.1 Graph-theoretic preliminaries
- $\mathcal G=(\mathcal V,\mathcal E)$, adjacency $A=[a_{ij}]$, $a_{ij}=1$ iff $(i,j)\in\mathcal E$.
- $\mathcal N_i = \{j : a_{ij} \ne 0\}$.
- Connectivity is required for the multi-hop "path" argument used in Eq. (8) (bounding $\|z_{i_0}-z_{j_0}\|$ by summing edge differences along a path).

### 5.2 Change of variables — the "shifted coordinate" trick
Define
$$z_i(t) = p_i(t) - R\phi(\theta_i), \qquad z^*(t) = \frac1n\sum_j z_j(t) = p^*(t)$$
This shifts the *desired circular formation* problem into a *plain consensus* problem on $z_i$: reaching consensus $z_i = z^*$ for all $i$ is equivalent to reaching the exact target circle $p_i = p^* + R\phi(\theta_i)$.

### 5.3 Formation-error Lyapunov function
$$V_c(t) = \tfrac12 \sum_i \|z_i(t) - z^*(t)\|^2$$
Derivative uses undirected-graph symmetry to rewrite the sum over neighbor pairs (Eq. 6→7), then a **finite-time consensus argument**: because $\|a\| \le -a^T\text{sgn}(a)$ for the *sign* nonlinearity (this is the whole reason sign-based control gives **finite-time**, not just asymptotic, convergence), you get
$$\dot V_c \le -c\sqrt{V_c}$$
for a constant $c>0$, which integrates to $V_c(t) \to 0$ in **finite time** $T_c$ (not just as $t\to\infty$) — this is the key theoretical payoff of using $\text{sgn}(\cdot)$ instead of a linear consensus term.

### 5.4 Localization dynamics after formation is locked ($t \ge T_c$)
Once $z_i \equiv p^*$ for all $i$, the closed-loop centroid dynamics reduce to
$$\dot p^*(t) = -\frac{2\beta}{nR}\sum_i \sigma(p_i(t))\phi(\theta_i)$$

### 5.5 First-order Taylor expansion + circular-symmetry cancellation
Expand $f(p_i)$ around $p^*$ using $p_i - p^* = R\phi(\theta_i)$:
$$f(p_i) - f(p^*) = \nabla f(p^*)^T R\phi(\theta_i) + \tfrac12 R^2\phi(\theta_i)^T H_f(p^*)\phi(\theta_i)$$
Then using the trig identities specific to a **regular $n$-gon on a circle** ($n>2$):
$$\sum_i \phi(\theta_i) = 0,\quad \sum_i \cos 2\theta_i = \sum_i \sin 2\theta_i = 0$$
the first-order term collapses to exactly $-\beta \nabla f(p^*)$ (Eq. 14), and the **second-order (Hessian) term vanishes identically** (Eq. 15) — this is the linchpin identity: *a uniform circular array is a natural finite-difference gradient estimator that cancels curvature bias exactly*, regardless of the (unknown) Hessian.

### 5.6 Bounding the "uninformed robot" and noise contributions
The remaining error terms are:
1. $\eta(p_i)$ noise bound $\to$ linear-in-$\delta$ error term (Eq. 19)
2. Geometric correction for $\mathcal Y$ (out-of-range) robots — uses the two-circle intersection geometry (circle of radius $D_{max}$ around $p_s$, circle of radius $R$ around $p^*$) to bound the "saturated" measurement contribution in terms of $n_X$, $n$, and $\sin(2\pi n_X/n)$ (Eqs. 20–21). This is where the final formula's dependence on the *number of informed robots* comes from.

### 5.7 Final Lyapunov/ISS-style bound (Theorem 1)
$$\dot V(p^*) \le -\beta\kappa\frac{2\pi n_X \mp n\sin(2\pi n_X/n)}{n\pi}\|p^*-p_s\|\Big(\|p^*-p_s\| - \varepsilon\Big)$$
giving the ultimate localization error bound
$$\varepsilon = \frac{2\pi n \delta}{\kappa R\big(2\pi n_X - n|\sin(2\pi n_X/n)|\big)}$$
Special case (**Remark 4**, all robots informed, $n_X = n$): $\varepsilon = \delta/(\kappa R)$ — a very clean, directly-checkable formula for validating any re-implementation.

### 5.8 Actuation transformation — unicycle feedback linearization (Sec. V)
Real TurtleBot3s are **not** single integrators; they're unicycles:
$$\dot x_i = v_i\cos\theta_i,\quad \dot y_i = v_i\sin\theta_i,\quad \dot\theta_i = \omega_i$$
To apply the single-integrator control law (4) to a unicycle, the paper uses the standard **point-offset feedback linearization** trick: control a point $s_i$ shifted by distance $r$ ahead of the robot center:
$$s_i = p_i + r[\cos\theta_i,\sin\theta_i]^T$$
$$\begin{bmatrix}\dot x_{si}\\ \dot y_{si}\end{bmatrix} = \begin{bmatrix}\cos\theta_i & -r\sin\theta_i\\ \sin\theta_i & r\cos\theta_i\end{bmatrix}\begin{bmatrix}v_i\\ \omega_i\end{bmatrix} = f_i$$
where $f_i$ is exactly the output of control law (4) (i.e., $\dot s_i = u_i$ is what you actually design, then you invert the matrix to recover real wheel-level commands):
$$\begin{bmatrix}v_i\\ \omega_i\end{bmatrix} = \begin{bmatrix}\cos\theta_i & \sin\theta_i\\ -\tfrac1r\sin\theta_i & \tfrac1r\cos\theta_i\end{bmatrix} f_i$$
This transformation is **required** any time you move this algorithm off single-integrator robots onto anything with non-holonomic (differential-drive, car-like) constraints — critically, $r \ne 0$ is required (the inversion is singular at $r=0$, i.e. you cannot directly control the wheel axle midpoint of a unicycle without this offset trick).

---

## 6. Re-simulating it: architecture options

### 6.1 Pure numerical re-implementation (fastest fidelity check, matches Sec. IV exactly)
- **Python + SciPy/NumPy**, `scipy.integrate.solve_ivp` (or simple Euler/RK4 fixed-step to mimic sampled-data comms) for the ODE system (1)+(4).
- Represent the graph as an adjacency list; recompute $\text{sgn}(\cdot)$ terms each step.
- This reproduces Figs. 2–4 almost exactly and is the right place to validate Theorem 1's formulas (Section 5.7) numerically before touching any physics/robot layer.
- **MATLAB/Simulink** is the more likely original toolchain (control-systems academic convention, easy graph/ODE integration, `sign()` built-in) — a MATLAB re-implementation is probably closest to what the authors ran.

### 6.2 Multi-agent robotics simulators (physics-in-the-loop, closer to Sec. V but virtual)
- **ROS 2 + Gazebo (or Ignition/Gz)**: spawn $n$ TurtleBot3 (or generic diff-drive) models, implement the control law as a ROS node per robot, publish `cmd_vel` via the unicycle inversion in Section 5.8. Gazebo gives you actuator noise, wheel slip, and sensor lag "for free," which the paper's pure math sim doesn't have.
- **Webots**: has a native TurtleBot3 model; similar workflow to Gazebo, arguably easier multi-robot swarm scripting via Webots' `Supervisor` API to inject ground-truth positions (mimicking OptiTrack).
- **PyBullet / MuJoCo**: lighter-weight, good if you want to batch many Monte Carlo runs quickly (e.g., sweeping $\delta$, $n_X$, $R$ against the closed-form $\varepsilon$ formula) — less "roboticist-friendly" than ROS/Gazebo but much faster to iterate and parallelize.
- **CoppeliaSim (formerly V-REP)**: also has ready diff-drive robot models, remote API is convenient for Python-driven multi-agent control loops.

### 6.3 Physical hardware options (paper used TurtleBot3 + OptiTrack; alternatives)
| Component | Paper's choice | Alternatives |
|---|---|---|
| Ground-truth / localization | OptiTrack (mocap) | Vicon, Qualisys, or on-board UWB-only localization (paper *has* UWB modules on the TurtleBot3 already per Fig. 7 — you could skip mocap entirely and do pure UWB trilateration, which is actually a more "distributed/decentralized" and scalable re-implementation, closer to the spirit of the algorithm's claimed communication savings) |
| Robot platform | TurtleBot3 (diff-drive) | e-puck2 (cheaper swarm robot, good for larger $n$), Crazyflie (if you want to extend to 3D/aerial, which ties into the paper's own Remark 7 on higher-dimensional balanced formations), custom differential-drive rig, or even a boat/underwater platform if simulating an "environmental monitoring" use case literally |
| Communication | OptiTrack-relayed / sampled broadcast every 0.1 s | Real inter-robot radio (XBee, ESP-NOW, nRF24, or ROS 2's DDS over Wi-Fi) — to actually test the "reduced communication cost" claim of ternary signaling, you'd want a **real bandwidth-constrained radio link**, not a mocap-relayed message, since the paper's own experimental setup doesn't really stress-test the communication savings it advertises |
| Source | Physical light/sound/RF emitter | Could realistically instrument with an actual RSSI (WiFi/BLE) beacon, a sound speaker + robot microphone array, or a simulated gas/chemical concentration field for an environmental-monitoring demo |

### 6.4 Suggested experiment matrix for a thorough re-simulation
1. **Numerical validation**: reproduce Theorem 1's $\varepsilon$ formula by sweeping $\delta, R, n_X/n$ and checking the ultimate bound numerically vs. the closed form.
2. **Gain-ratio sensitivity**: sweep $\alpha/\beta$ near the threshold $4nf_{D_{max}}/R$ to see how tightly the theorem's inequality needs to be respected in practice (the paper never explores "how much slack is needed" empirically).
3. **Physics-in-the-loop (Gazebo/Webots)** with real actuator saturation, to see how much the theoretical finite-time formation bound degrades under realistic dynamics before going to real hardware.
4. **Hardware** with UWB-only comms (no mocap) to genuinely test the "ternary/low-bandwidth communication" claim under real radio conditions, including packet loss — which is directly relevant to Section 7 below.

---

## 7. Directions the paper *names* but never simulates — and how you could

The Conclusion explicitly lists three unexplored extensions. Here's what implementing each would require, building on the theory above:

### 7.1 Time-varying source ($\dot p_s \ne 0$)
- Breaks the static-source assumption baked into the Lyapunov proof (Sec. 5.6–5.7 above implicitly assume $p_s$ fixed).
- To simulate: replace $p_s$ with $p_s(t)$ (e.g., constant-velocity or sinusoidal trajectory) and re-derive $\dot V(p^*-p_s(t))$, which will pick up an extra cross term $-(p^*-p_s)^T\dot p_s$. You'd need either (a) a feedforward compensation term added to the control law if $\dot p_s$ is known/estimable, or (b) accept a bounded tracking error proportional to $\|\dot p_s\|$ (a standard ISS-style extension). This is a genuinely open (if incremental) research extension you could implement and numerically test even without a full new proof — just simulate it and empirically compare tracking error growth vs. source speed.

### 7.2 Directed network topologies
- Assumption 1 requires undirected + connected; the whole finite-time consensus proof (Sec. 5.3) leans on symmetry ($a_{ij}=a_{ji}$) to rewrite the double sum in Eq. (6)–(7) as a clean pairwise sum. For a **directed** graph you'd need a different Lyapunov candidate (e.g. one weighted by left eigenvector of the graph Laplacian, standard in directed-consensus literature) and the sign function's finite-time property under directed graphs is a genuinely harder open problem (asymmetric sign interactions don't cancel as nicely).
- Practically: this is straightforward to *simulate* (just make the adjacency matrix asymmetric and rerun the same ODE) even though the *existing convergence proof no longer formally applies* — a good empirical experiment: does the control law still work in practice on a directed ring or leader-follower topology even without the current proof covering it?

### 7.3 Delayed and unsafe (lossy/adversarial) communications
- Currently the sign message $\text{sgn}(\cdot)$ is assumed to arrive instantly and correctly. Two separate things to add:
  - **Delay**: buffer neighbor states with a fixed or time-varying delay $\tau_{ij}$ before computing the sign term — classic delayed-consensus territory; the finite-time argument likely degrades to only asymptotic convergence, or requires a delay-dependent gain bound.
  - **Unsafe/adversarial** (referencing their own [21], which handles deception attacks on double integrators): simulate a subset of robots sending corrupted sign values (bit-flip attacks, since messages are literally only ∈{-1,0,1}, which is an interesting attack surface — a flipped sign is a maximally-disruptive attack given the ternary encoding) and test whether the formation/localization still converges, or add a resilience mechanism (e.g., median-based or trimmed-mean filtering of neighbor sign votes, analogous to MSR-type resilient consensus algorithms).

### 7.4 Higher-dimensional / balanced formations (Remark 7, cites ref [23])
- The paper explicitly says the 2D circular result "can be extended to the higher dimensional case... using balanced formations" but doesn't do it.
- To simulate in 3D: replace $\phi(\theta_i)$ with a **balanced spherical configuration** (e.g., points from a spherical code / Fibonacci sphere, or icosahedral vertex sets) satisfying the analogous identity $\sum_i \phi_i = 0$ and $\sum_i \phi_i\phi_i^T \propto I$ (this is exactly what "balanced formation" means in the cooperative-control literature) — these are the 3D conditions needed to reproduce the cancellations in Eq. (14)–(15). This is directly simulate-able (e.g., with Crazyflie drones or in a 3D physics sim) using the same control law structure with $\phi(\theta_i) \in \mathbb R^3$.

### 7.5 More complex robot dynamics beyond unicycle (also mentioned in Remark 7)
- The paper only goes as far as feedback-linearizing a unicycle. You could extend to a **bicycle/car-like model** (with steering-angle constraints), a **quadrotor** (needs a cascaded position/attitude controller under the same outer-loop $u_i$), or robots with actuator saturation/bounded velocity — all requiring their own version of the Section 5.8 linearization or a saturation-aware redesign of $\alpha,\beta$.

---

## 8. Quick checklist if you're about to build this

1. Pick $n$, generate a connected (undirected, or intentionally directed as an experiment) graph.
2. Fix $\kappa, D_{max}, \delta, p_s, R$; compute $f_{D_{max}} = \kappa D_{max}^2+\delta$.
3. Choose $\alpha,\beta$ satisfying $\alpha/\beta > 4nf_{D_{max}}/R$ (note this is a *sufficient*, likely conservative, condition — expect it to work even closer to the boundary in practice, which is itself worth testing).
4. Assign $\theta_i = 2\pi i/n$ (for 3D, use a balanced spherical configuration instead — Section 7.4).
5. Implement Eq. (4) exactly; if going to real/simulated unicycle robots, wrap it with the Section 5.8 transformation.
6. Validate against the closed-form $\varepsilon = \dfrac{2\pi n\delta}{\kappa R(2\pi n_X - n|\sin(2\pi n_X/n)|)}$ (or the clean special case $\varepsilon=\delta/(\kappa R)$ when $n_X=n$) as your ground-truth correctness check.
7. Layer in extensions from Section 7 one at a time (moving source → directed graph → delay/attacks → 3D) to explore exactly where the theory's guarantees start to break down empirically.
