"""Assemble REPORT.md from batch_summary.json.

Reads the aggregated summaries produced by run_batch.m and emits a single
markdown report: the 24-case uninformed-robot matrix (master table + per-case
sections) plus §6 gain sweeps (single-integrator alpha sweep, TurtleBot-Simulink
alpha sweep, and an extreme beta = 10000 probe). Image links are relative to the
report's own folder (outputs/batch_report/).
"""

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA = json.loads((HERE / "batch_summary.json").read_text())
REPORT = HERE / "REPORT.md"
_meta_path = HERE / "sweep_meta.json"
SWEEP_META = json.loads(_meta_path.read_text()) if _meta_path.exists() else {}
DATE = "2026-07-14"


def g(x, fmt="{:.4g}"):
    return fmt.format(x) if isinstance(x, (int, float)) else str(x)


def src(s):
    return f"[{g(s[0])}, {g(s[1])}]"


def rel(run_dir):
    # run_dir is 'outputs/batch_report/runs/<id>' ; report lives in outputs/batch_report
    return run_dir.replace("outputs/batch_report/", "")


def order_key(item):
    rid, rec = item
    p = rec["summary"]["parameters"]
    mode_rank = 0 if rec["mode"] == "single_integrator" else 1
    src_rank = 0 if p["source"][0] == 5.5 else 1
    noise_rank = 0 if p["noise_model"] == "none" else 1
    return (mode_rank, p["n"], src_rank, noise_rank)


# Prefixes: si_/ts_ = 24-case main matrix; fg_ = single-integrator alpha sweep;
# fgt_ = TurtleBot-Simulink alpha sweep; fgb_ = extreme beta = 10000 probe. Each
# sweep gets its own subsection in §6.
def _alpha(kv):
    return kv[1]["summary"]["parameters"]["alpha"]

cases = sorted((c for c in DATA.items() if c[0].startswith(("si_", "ts_"))), key=order_key)
si = [c for c in cases if c[1]["mode"] == "single_integrator"]
ts = [c for c in cases if c[1]["mode"] == "turtlebot_simulink"]
fg_si = sorted((c for c in DATA.items() if c[0].startswith("fg_")), key=_alpha)
fg_ts = sorted((c for c in DATA.items() if c[0].startswith("fgt_")), key=_alpha)
fg_beta = sorted((c for c in DATA.items() if c[0].startswith("fgb_")), key=_alpha)

_meta_path = HERE / "sweep_meta.json"
SWEEP_META = json.loads(_meta_path.read_text()) if _meta_path.exists() else {}

L = []
w = L.append

w(f"# SGF Batch Report: uninformed-robot cases (MATLAB single-integrator & TurtleBot Simulink)")
w("")
w(f"*Generated {DATE} · MATLAB R2026a (+ Simulink) · reproduction of Du et al. (2024), "
  "\"Simultaneous Source Localization and Formation via a Distributed Sign Gradient-Free "
  "Algorithm\". 24 runs, each with 1–3 robots starting outside the sensing radius.*")
w("")
w("## 1. Objective and case matrix")
w("")
w("This report sweeps two MATLAB runtimes of the paper's Eq. 4 sign gradient-free control "
  "law across robot count, source location, and noise, and: unlike the earlier all-informed "
  "report: deliberately starts a **fraction of each swarm outside the sensing radius**. Those "
  "robots are *uninformed* (blind): by Eq. 2 they measure only the constant saturation value "
  "`f_Dmax = κ·Dmax² + δ = 144.2`, never the true field. Each run records whether the swarm "
  "still (a) forms the target circle and (b) localizes its centroid to the source despite the "
  "blind members.")
w("")
w("| Axis | Values |")
w("|---|---|")
w("| Mode | `single_integrator` (point robots, MATLAB parity sim) · `turtlebot_simulink` "
  "(differential-drive, Simulink `.slx`) |")
w("| n / uninformed | **n=4** (1 blind / 3 informed) · **n=6** (2 / 4) · **n=8** (3 / 5) |")
w("| Source | **A** = [5.5, 5.5] (paper) · **B** = [30, -20] (far-shifted) |")
w("| Noise | `none` · `gaussian` (η ~ N(0, 0.2), paper-faithful) |")
w("")
w("`2 modes × 3 sizes × 2 noise × 2 sources = ` **24 distinct runs. All 24 completed "
  "successfully.** The Simulink builder now injects **real seeded Gaussian measurement noise** "
  "([build_turtlebot_simulink_model.m](../../matlab_turtlebot/build_turtlebot_simulink_model.m), "
  "a vector Random Number source sampled at the solver step), so each noisy Simulink run is a "
  "genuinely distinct, reproducible run: not a duplicate of its noise-free twin as in the "
  "previous report.")
w("")
w("### Assumption 2 / Remark 1 (informed robots): exercised, not assumed")
w("")
w("The paper requires the perturbation to be bounded (|η(pᵢ)| ≤ δ) **and at least one robot "
  "within `Dmax`** so the field is observed. Here the uninformed robots are created purely by "
  "**initial distance** (started 14–17 m out, beyond `Dmax = 12`), not by the `informed` mask, "
  "so the split is identical in the single-integrator path and in the Simulink ODE (which "
  "honours only the `dist < Dmax` rule). Every case therefore begins with `min_n_informed = "
  "n − k` (k = 1, 2, 3 blind for n = 4, 6, 8). As the circle forms, the blind robots are pulled "
  "inside `Dmax` and become informed, so `final_n_informed = n`; the `min_n_informed` column "
  "records that the uninformed phase was genuinely exercised. Localization still converges from "
  "the informed sub-swarm alone, and the theorem's ε inflates with the blind fraction (ε = "
  "0.169, 0.189, 0.195 for n = 4, 6, 8 vs the all-informed 0.1).")
w("")
w("## 2. Fixed parameters")
w("")
w("Common to all runs: `kappa = 1`, `R = 2`, `Dmax = 12`, `seed = 1`, ring communication "
  "topology (connected for any n ≥ 3). Initial positions are placed on a ring **relative to "
  "the source**: the `n − k` informed robots at radii 5–10 m (inside `Dmax`) and the `k` "
  "uninformed robots at radii 14–17 m (outside `Dmax`). Because the layout is source-relative, "
  "the initial formation-error geometry is identical at both sources: which is why source A "
  "and B produce identical error curves.")
w("")
w("| Parameter | single_integrator | turtlebot_simulink |")
w("|---|---|---|")
w("| Control law | Eq. 4 sign gradient-free | Eq. 4 + unicycle feedback-linearization → diff-drive |")
w("| alpha / beta | 100 / 0.05 (ratio 2000) | 10 / 0.05 (ratio 200) |")
w("| Signum | exact `sgn` | boundary layer `sat(·/0.2)` |")
w("| Integrator | explicit Euler | Simulink `ode4`, fixed-step |")
w("| dt | 0.0005 s | 0.004 s |")
w("| duration | 60 s | 90 s |")
w("| control-point offset r |: | 2.0 m |")
w("| noise (when on) | η ~ N(0, 0.2) additive on informed measurement | same, via seeded Simulink Random Number block |")
w("| perturbation bound δ (ε reference) | 0.2 | 0.2 |")
w("")
w("The `single_integrator` gain ratio (2000) satisfies the paper's conservative *sufficient* "
  "condition `alpha/beta > 4·n·f_Dmax/R` for n = 4, 6 (≈ 1154, 1730) but not n = 8 (≈ 2307); "
  "the n = 8 runs converge anyway, confirming the condition is sufficient, not necessary. The "
  "Simulink runs use ratio 200 (below the sufficient bound for all n) and also converge.")
w("")
w("**ε applicability.** The theorem's ε bound assumes *bounded* noise (|η| ≤ δ). It therefore "
  "applies to the **noise-free** runs (`inside ε?` shows yes/no there), but for the **gaussian** "
  "runs: whose noise is unbounded: ε is reported only as a *reference* value and `inside ε?` "
  "is shown as `n/a`. In practice the gaussian runs land at essentially the same final error as "
  "their noise-free twins, well inside the reference ε.")
w("")

# ---- master table ---------------------------------------------------------
w("## 3. Results at a glance")
w("")
w("| Run | Mode | n | Source | Noise | init→final formation | init→final localization | "
  "min inf. | ε | inside ε? |")
w("|---|---|---|---|---|---|---|---|---|---|")
for rid, rec in cases:
    s = rec["summary"]
    p, m, v = s["parameters"], s["metrics"], s["validation"]
    mode = "SI" if rec["mode"] == "single_integrator" else "TS"
    inside = ("yes" if v["inside_bound"] else "no") if v.get("bound_applicable") else "n/a"
    w(f"| `{rid}` | {mode} | {p['n']} | {src(p['source'])} | {p['noise_model']} | "
      f"{g(m['initial_formation_error'])} → {g(m['final_formation_error'])} | "
      f"{g(m['initial_localization_error'])} → {g(m['final_localization_error'])} | "
      f"{v['min_n_informed']} | {g(v['epsilon'])} | {inside} |")
w("")
w("`SI` = single_integrator, `TS` = turtlebot_simulink. `min inf.` = `min_n_informed` = "
  "`n − k` informed robots at the start (the rest are blind). `inside ε?`: yes/no against the "
  "theorem bound for bounded/noise-free runs; `n/a` for gaussian (unbounded) noise: see §2.")
w("")
w("**Animations.** One-third of the batch (8 runs) also exports a `motion.gif` showing the "
  "formation contracting and the centroid approaching the source (blind robots start outside "
  "the sensing ring and get pulled in): both modes at n = 4, 6, 8 with source-A gaussian noise, "
  "plus the n = 6 source-A noise-free pair. The GIFs are embedded in each run's section below.")
w("")


def param_table(p, mode):
    rows = [
        ("n", p["n"]), ("source", src(p["source"])), ("kappa", p["kappa"]),
        ("R", p["radius"]), ("Dmax", p["dmax"]), ("alpha", p["alpha"]),
        ("beta", p["beta"]), ("noise_model", p["noise_model"]),
        ("noise_bound δ", p["noise_bound"]), ("dt", p["dt"]),
        ("duration", p["duration"]), ("topology", p["topology"]), ("seed", p["seed"]),
    ]
    if mode == "turtlebot_simulink":
        rows += [("offset r", p.get("control_point_offset")),
                 ("sign_boundary_layer", p.get("sign_boundary_layer")),
                 ("wheel_radius", p.get("wheel_radius")),
                 ("wheel_separation", p.get("wheel_separation"))]
    out = ["| Parameter | Value |", "|---|---|"]
    out += [f"| {k} | {g(x) if isinstance(x,(int,float)) else x} |" for k, x in rows]
    return "\n".join(out)


def metric_table(m, v, mode):
    rows = [
        ("initial formation error", g(m["initial_formation_error"])),
        ("final formation error", g(m["final_formation_error"])),
        ("initial localization error", g(m["initial_localization_error"])),
        ("final localization error", g(m["final_localization_error"])),
    ]
    if "formation_entry_time" in m:
        rows.append(("formation entry time [s]", g(m["formation_entry_time"])))
    if "localization_entry_time" in m and m["localization_entry_time"] is not None:
        rows.append(("localization entry time [s]", g(m["localization_entry_time"])))
    rows += [
        ("gain ratio", g(v["gain_ratio"])),
        ("gain threshold 4n·f_Dmax/R", g(v["gain_threshold"])),
        ("gain condition passed", v["gain_condition_passed"]),
        ("min / final n_informed", f"{v['min_n_informed']} / {v['final_n_informed']}"),
        ("epsilon (ε)", g(v["epsilon"])),
        ("inside bound", v["inside_bound"] if v.get("bound_applicable")
         else "n/a (unbounded gaussian noise)"),
    ]
    if mode == "turtlebot_simulink":
        rows += [("max commanded |v| [m/s]", g(m["max_commanded_linear_velocity"])),
                 ("max commanded |ω| [rad/s]", g(m["max_commanded_angular_velocity"]))]
    out = ["| Metric | Value |", "|---|---|"]
    out += [f"| {k} | {x} |" for k, x in rows]
    return "\n".join(out)


def case_section(rid, rec):
    s = rec["summary"]
    p, m, v = s["parameters"], s["metrics"], s["validation"]
    r = rel(rec["run_dir"])
    out = [f"### `{rid}`  ·  {rec['mode']}  ·  n={p['n']}  ·  source {src(p['source'])}  "
           f"·  noise `{p['noise_model']}`", ""]
    out.append(f"*Run folder:* `{rec['run_dir']}`  ·  *wall time:* {g(rec['elapsed_s'])} s")
    out.append("")
    out.append("<table><tr><td valign=top>")
    out.append("")
    out.append(param_table(p, rec["mode"]))
    out.append("")
    out.append("</td><td valign=top>")
    out.append("")
    out.append(metric_table(m, v, rec["mode"]))
    out.append("")
    out.append("</td></tr></table>")
    out.append("")
    out.append(f"![trajectory]({r}/trajectory.png)")
    out.append("")
    out.append(f"![formation error]({r}/formation_error.png) "
               f"![localization error]({r}/localization_error.png)")
    if rec["mode"] == "single_integrator":
        out.append("")
        out.append(f"*Per-robot formation error (paper Fig. 3 style):*")
        out.append("")
        out.append(f"![per-robot formation error]({r}/formation_error_per_robot.png)")
    if (HERE / r / "motion.gif").exists():
        out.append("")
        out.append(f"*Animation: formation + source approach (source = red star, target circle dotted):*")
        out.append("")
        out.append(f"![motion]({r}/motion.gif)")
    out.append("")
    return "\n".join(out)


w("## 4. Single-integrator runs (12)")
w("")
for rid, rec in si:
    w(case_section(rid, rec))
w("## 5. TurtleBot Simulink runs (12)")
w("")
for rid, rec in ts:
    w(case_section(rid, rec))

w("## 6. Gain sweeps (n = 6): what each gain actually controls")
w("")
w("The 24 runs above use a large formation gain, so formation looks instant. These sweeps "
  "isolate each gain by varying one at a time with everything else identical (n = 6, source A, "
  "noise-free, the same 2-uninformed layout). §6.1 sweeps the formation gain α on the "
  "single-integrator model; §6.2 repeats the same α values on the TurtleBot-Simulink "
  "differential-drive model (plots + gifs); §6.3 pushes the localization gain β to an extreme "
  "(10000) to show the failure mode. Formation entry time is the first instant the spread falls "
  "below 5 % of its initial value (consistent across both models).")
w("")


def per_run_plots(group, is_si):
    """Embed each sweep run's own static plots (trajectory + error curves, plus
    the per-robot formation panel for single-integrator runs), α increasing."""
    w("*Per-run static plots, α increasing (each run's own trajectory, then its "
      "formation & localization error):*")
    w("")
    for rid, rec in group:
        r = rel(rec["run_dir"])
        a = g(rec["summary"]["parameters"]["alpha"])
        w(f"**α = {a}**  (`{rid}`)")
        w("")
        w(f"![trajectory α={a}]({r}/trajectory.png)")
        w("")
        w(f"![formation error α={a}]({r}/formation_error.png) "
          f"![localization error α={a}]({r}/localization_error.png)")
        if is_si and (HERE / r / "formation_error_per_robot.png").exists():
            w("")
            w(f"![per-robot formation error α={a}]({r}/formation_error_per_robot.png)")
        w("")


def sweep_block(header, group, overlay_png, intro, reading, is_ts):
    w(f"### {header}")
    w("")
    w(intro)
    w("")
    w(f"![{header}]({overlay_png})")
    w("")
    if is_ts:
        w("| α | α/β | formation entry [s] | final formation | final localization | "
          "max \\|v\\| [m/s] | inside ε? |")
        w("|---|---|---|---|---|---|---|")
    else:
        w("| α | α/β | formation entry [s] | final formation | final localization | inside ε? |")
        w("|---|---|---|---|---|---|")
    for rid, rec in group:
        s = rec["summary"]; p, m, v = s["parameters"], s["metrics"], s["validation"]
        fe = SWEEP_META.get(rid, {}).get("formation_entry")
        fe_s = g(fe) if isinstance(fe, (int, float)) else "never"
        inside = ("yes" if v["inside_bound"] else "no") if v.get("bound_applicable") else "n/a"
        row = (f"| {g(p['alpha'])} | {g(v['gain_ratio'])} | {fe_s} | "
               f"{g(m['final_formation_error'])} | {g(m['final_localization_error'])} |")
        if is_ts:
            row += f" {g(m.get('max_commanded_linear_velocity'))} |"
        w(row + f" {inside} |")
    w("")
    for line in reading:
        w(line)
    w("")
    per_run_plots(group, is_si=not is_ts)
    gifs = []
    for rid, rec in group:
        r = rel(rec["run_dir"])
        if (HERE / r / "motion.gif").exists():
            gifs.append(f"![alpha = {g(rec['summary']['parameters']['alpha'])}]({r}/motion.gif)")
    if gifs:
        w("*Animations, α increasing left to right (source = red star, target circle dotted):*")
        w("")
        w(" ".join(gifs))
        w("")


sweep_block(
    "6.1 Single-integrator α sweep",
    fg_si,
    "formation_gain_sweep.png",
    "Point-robot model with exact `sgn`. Only α varies; β = 0.05 fixed.",
    [
        "Reading the sweep:",
        "",
        "- **Formation time ∝ 1/α.** Formation entry drops from ≈ 2.9 s at α = 1 to ≈ 0.09 s at "
        "α = 100: the finite-time consensus term `α·Σ sgn(z_j − z_i)` closes the initial spread "
        "faster the larger α is. On a 0–60 s axis, α = 100 looks like an instantaneous drop.",
        "- **The chatter floor grows with α.** With the exact `sgn`, the steady-state formation "
        "residual *rises* with α (≈ 2×10⁻³ at α = 1 up to ≈ 0.18 at α = 100): a bigger gain "
        "drives a larger limit-cycle around the sliding surface. Faster formation is paid for "
        "with a coarser final circle.",
        "- **Localization timescale is α-independent.** All five localization curves decay at "
        "nearly the same exponential rate `2βκ = 0.1 /s`, set by β, not α: smaller α even "
        "localizes slightly *faster* because it injects less chatter into the centroid.",
    ],
    is_ts=False,
)

sweep_block(
    "6.2 TurtleBot-Simulink α sweep",
    fg_ts,
    "formation_gain_sweep_ts.png",
    "The same five α values through the differential-drive Simulink model (boundary-layer "
    "`sat(·/0.2)`, unicycle feedback-linearization, offset r = 2, β = 0.05).",
    [
        "Reading the sweep:",
        "",
        "- **The 1/α formation law survives the vehicle model.** Formation entry tracks the "
        "single-integrator almost exactly (≈ 2.9 s at α = 1 down to ≈ 0.10 s at α = 100), so the "
        "finite-time mechanism carries through the feedback-linearization.",
        "- **But accuracy is non-monotonic: there is an upper useful α.** α = 20 is the sweet "
        "spot (final formation ≈ 5×10⁻³). Beyond it the large gain excites the differential-drive "
        "/ boundary-layer dynamics: α = 50 overshoots and leaves the ε bound (final localization "
        "≈ 0.27 > ε = 0.19), and α = 100 degrades the circle (final formation ≈ 0.87).",
        "- **Commanded speed explodes with α.** Peak \\|v\\| climbs from ≈ 10 m/s (α = 1) to "
        "≈ 283 m/s (α = 100): unphysical for a real TurtleBot, and the practical reason the main "
        "Simulink runs use α = 10. Unlike the ideal single-integrator, the real-vehicle model "
        "has a ceiling on useful formation gain.",
    ],
    is_ts=True,
)

w("### 6.3 Extreme localization gain: β = 10000 (Simulink)")
w("")
w("Holding β = 10000 (vs the paper's 0.05) and sweeping α = 1, 10, 100 probes the opposite "
  "imbalance: localization gain overwhelming formation. Plots and summaries are saved; no gif "
  "(the trajectory scale is non-physical). The overlay is log-scale because the errors diverge.")
w("")
w("![6.3 beta = 10000 sweep](formation_gain_beta_sweep.png)")
w("")
w("| α | α/β | final formation | final localization | max \\|v\\| [m/s] | forms? |")
w("|---|---|---|---|---|---|")
for rid, rec in fg_beta:
    s = rec["summary"]; p, m, v = s["parameters"], s["metrics"], s["validation"]
    w(f"| {g(p['alpha'])} | {g(v['gain_ratio'])} | {g(m['final_formation_error'])} | "
      f"{g(m['final_localization_error'])} | {g(m.get('max_commanded_linear_velocity'))} | "
      "no (diverges) |")
w("")
w("Reading the probe:")
w("")
w("- **The swarm diverges regardless of α.** All three runs blow up to a formation error "
  "≈ 1.9×10⁸ and a localization error ≈ 1.1×10⁷, nearly identical for α = 1, 10, 100: the "
  "localization term `(2β/R)·σ ≈ 1.4×10⁶` dwarfs the formation term (≤ 2α ≤ 200) by four-plus "
  "orders of magnitude, so α is irrelevant here.")
w("- **Mechanism.** The huge push drives every robot outward at the saturation velocity "
  "\\|v\\| = (2β/R)·f_Dmax = 1.442×10⁶ m/s; they immediately leave `Dmax`, then all read the "
  "constant `f_Dmax` and keep accelerating outward in a fixed direction: neither formation nor "
  "localization ever begins.")
w("- **Lesson.** β does not buy localization *speed* (the rate is `2βκ` only in the small-gain "
  "regime where formation stays intact); an oversized β destabilizes the whole system. The "
  "localization gain must stay small, as the paper's β = 0.05 does.")
w("")
per_run_plots(fg_beta, is_si=False)
w("## 7. Observations")
w("")
w("- **Uninformed robots do not break localization.** Despite 1–3 blind robots (33–25 % of the "
  "swarm) that only ever report the saturation constant `f_Dmax = 144.2`, every run still forms "
  "the circle and drives the centroid to the source: single-integrator final localization "
  "0.014–0.035, Simulink 2–9×10⁻⁴. The informed sub-swarm supplies enough gradient information "
  "to localize, exactly as Theorem 1 predicts for `n_𝒳 ≥ 1`.")
w("- **The blind phase is real, then self-heals.** Every case starts with `min_n_informed = "
  "n − k` (3/4/5 for n = 4/6/8); as the circle contracts around the source the out-of-range "
  "robots cross back inside `Dmax`, so `final_n_informed = n`. The uninformed measurement branch "
  "(Eq. 2) is genuinely exercised during the transient.")
w("- **ε inflates with the blind fraction, as the theorem says.** ε = 0.169 / 0.189 / 0.195 for "
  "n = 4 / 6 / 8 (vs the all-informed 0.1 = δ/κR), and every noise-free run lands inside its ε. "
  "The gaussian runs (unbounded noise, ε not formally applicable) land at essentially the same "
  "final error as their noise-free twins.")
w("- **Source-independence is exact.** For every (mode, n, noise), source A [5.5,5.5] and "
  "source B [30,-20] produce identical error curves and metrics: the control law plus the "
  "source-relative initial layout are translation-invariant, so a far source localizes as well "
  "as the paper's.")
w("- **Simulink noise is now real and reproducible.** With the seeded Random Number source, each "
  "gaussian Simulink run differs measurably from its noise-free twin (e.g. n=4 final "
  "localization 8.9×10⁻⁴ vs 2.0×10⁻⁴) yet stays fully convergent: and the differential-drive "
  "runs converge at ratio 200, below the sufficient bound for all n, reconfirming it is "
  "sufficient, not necessary.")
w("- **Commanded velocities are large** (|v| ≈ 24–30 m/s, |ω| ≈ 13–18 rad/s) because the "
  "Simulink reference runs unsaturated with offset r = 2; these are ideal-actuator numbers, not "
  "TurtleBot3 hardware limits. Set `max_linear_velocity` / `max_angular_velocity` to clamp them.")
w("- **The gain sweeps (§6) separate the two knobs.** α sets formation speed (∝ 1/α) in both "
  "models, but the differential-drive model has an upper useful α (past α ≈ 20 accuracy degrades "
  "and commanded speed explodes), whereas β must stay small: the β = 10000 probe diverges "
  "outright. This is why the main matrix uses large α with a small β.")
w("")
w("## 8. Reproduction")
w("")
w("```matlab")
w("% from the repository root, with MATLAB + Simulink")
w("addpath('outputs/batch_report');")
w("run_batch('si_*');    % 12 single-integrator cases")
w("run_batch('ts_*');    % 12 turtlebot_simulink cases")
w("run_batch('fg_*');    % 5 single-integrator formation-gain sweep (n = 6)")
w("run_batch('fgt_*');   % 5 turtlebot_simulink formation-gain sweep (n = 6)")
w("run_batch('fgb_*');   % 3 extreme beta = 10000 probes (n = 6)")
w("make_formation_gain_plot;   % 3 overlay figures + sweep_meta.json")
w("```")
w("")
w("Configs live in `outputs/batch_report/configs/*.json` (generated by `gen_configs.py`); each "
  "run writes its plots + `summary.json` under `outputs/batch_report/runs/<run_id>_{matlab,"
  "turtlebot}/`, and `run_batch.m` aggregates every summary into `batch_summary.json`. This "
  "report is regenerated from that file by `make_report.py`.")
w("")

REPORT.write_text("\n".join(L), encoding="utf-8")
print(f"wrote {REPORT}  ({len(L)} lines, {len(cases)} cases)")
