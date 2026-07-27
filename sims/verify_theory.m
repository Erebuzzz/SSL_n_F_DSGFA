% verify_theory.m
% Graphical verification of the analytical claims in
% docs/research/research_gaps.tex and docs/RESEARCH_GAPS_THEORY.md.
%
% Each check produces one figure that plots a measured quantity against the
% quantity the theory predicts, plus a pass/fail record in a JSON summary.
% A check is a falsification attempt: it either reproduces the predicted
% curve or it does not. Numerical agreement is evidence, never a proof.
%
% Run from sims/:
%   verify_theory              % all checks
%   verify_theory v1           % single check by id
%   verify_theory baseline     % group: v1 v2 v3 v4 v5 v6
%   verify_theory gap1         % group: v7 v8
%   verify_theory gap2         % group: v9
%   verify_theory gap3         % group: v10 v11 v12
%
% Outputs: sims/outputs/verify_theory/
%   <id>_<name>.png  one figure per claim
%   summary.json     pass/fail plus the numbers behind each verdict

function verify_theory(varargin)
scriptDir = fileparts(mfilename("fullpath"));
outDir = fullfile(scriptDir, "outputs", "verify_theory");
if ~exist(outDir, "dir")
    mkdir(outDir);
end

registry = checkRegistry();
selected = selectChecks(registry, varargin);

fprintf("verify_theory.m — %d check(s)\n", numel(selected));
fprintf("Reference: docs/research/research_gaps.tex\n\n");

summaryPath = fullfile(outDir, "summary.json");
summary = loadExistingSummary(summaryPath);
passCount = 0;
for k = 1:numel(selected)
    entry = registry(selected(k));
    fprintf("[%s] %s\n", upper(entry.id), entry.title);
    record = entry.fcn(outDir);
    record.claim = entry.claim;
    record.title = entry.title;
    summary.(entry.id) = record;
    if record.pass
        passCount = passCount + 1;
        fprintf("      PASS  %s\n\n", record.verdict);
    else
        fprintf("      FAIL  %s\n\n", record.verdict);
    end
end

summary.meta = struct( ...
    "checks_run", numel(selected), ...
    "checks_passed", passCount, ...
    "timestamp", string(datetime("now", "Format", "uuuu-MM-dd HH:mm:ss")));

writeJson(summaryPath, summary);
fprintf("%d/%d checks passed. Figures and summary: %s\n", ...
    passCount, numel(selected), outDir);
end

%% Registry and selection

function registry = checkRegistry()
registry = struct( ...
    "id", {"v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12"}, ...
    "group", {"baseline", "baseline", "baseline", "baseline", "baseline", "baseline", ...
              "gap1", "gap1", "gap2", "gap3", "gap3", "gap3"}, ...
    "title", { ...
        "Regular-polygon identities", ...
        "Connectivity bound Q <= n S", ...
        "Finite-time formation envelope", ...
        "Exact circular gradient identity", ...
        "All-informed localization bound", ...
        "Consistency of epsilon and lambda", ...
        "Team-size cancellation in epsilon_k", ...
        "Basin-containment certificate", ...
        "Moving-source steady lag", ...
        "Circular estimator bias O(R^2)", ...
        "Local trapping of ideal ascent", ...
        "Multi-start selection condition"}, ...
    "claim", { ...
        "lem:circle-identities", ...
        "lem:connectivity-bound", ...
        "prop:formation / eq:formation-time-bound", ...
        "eq:exact-gradient-identity", ...
        "thm:all-informed / eq:all-informed-epsilon", ...
        "eq:du-epsilon / eq:lambda-informed", ...
        "eq:team-epsilon-scale", ...
        "lem:basin-certificate", ...
        "cor:constant-velocity / eq:moving-ultimate-bound", ...
        "lem:estimator-bias", ...
        "prop:no-global-guarantee", ...
        "thm:multistart / eq:selection-condition"}, ...
    "fcn", {@checkPolygonIdentities, @checkConnectivityBound, ...
            @checkFormationFiniteTime, @checkGradientIdentity, ...
            @checkAllInformedBound, @checkEpsilonLambda, ...
            @checkTeamScaling, @checkBasinCertificate, ...
            @checkMovingSourceLag, @checkEstimatorBias, ...
            @checkLocalTrapping, @checkMultiStartSelection});
end

function selected = selectChecks(registry, args)
if isempty(args) || isempty(args{1})
    selected = 1:numel(registry);
    return
end
wanted = lower(string(args{1}));
ids = string({registry.id});
groups = string({registry.group});
if wanted == "all"
    selected = 1:numel(registry);
elseif any(ids == wanted)
    selected = find(ids == wanted);
elseif any(groups == wanted)
    selected = find(groups == wanted);
else
    error("Unknown check '%s'. Use an id (%s), a group (baseline|gap1|gap2|gap3), or 'all'.", ...
        wanted, strjoin(ids, " "));
end
end

%% V1 — Regular-polygon identities (lem:circle-identities)

function record = checkPolygonIdentities(outDir)
% Verifies sum(phi)=0, sum(phi phi')=n/2 I, and sum((phi' H phi) phi)=0.
% The third identity is predicted to hold only for n >= 4.
rng(1, "twister");
nList = 3:20;
res1 = zeros(size(nList));
res2 = zeros(size(nList));
res3 = zeros(size(nList));

H = [0.8, -0.3; -0.3, 1.7];
H = H / norm(H);

for k = 1:numel(nList)
    phi = slots(nList(k));
    res1(k) = norm(sum(phi, 1));
    res2(k) = norm(phi' * phi - (nList(k) / 2) * eye(2));
    quad = sum((phi * H) .* phi, 2);
    res3(k) = norm(sum(quad .* phi, 1));
end

tol = 1e-10;
pass1 = all(res1 < tol);
pass2 = all(res2 < tol);
pass3n4 = all(res3(nList >= 4) < tol);
fail3n3 = res3(nList == 3) > 1e-3;
pass = pass1 && pass2 && pass3n4 && fail3n3;

fig = figure("Visible", "off", "Position", [100, 100, 900, 380]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile
semilogy(nList, max(res1, eps), "o-", "LineWidth", 1.2, "MarkerSize", 5);
hold on
semilogy(nList, max(res2, eps), "s-", "LineWidth", 1.2, "MarkerSize", 5);
yline(tol, "k:", "tolerance", "LineWidth", 1.0);
grid on
xlabel("robots per ring, n");
ylabel("residual norm");
title({"First and second moments", "$\sum\phi_i=0$,\ \ $\sum\phi_i\phi_i^T=\frac n2 I$"}, ...
    "Interpreter", "latex");
legend({"$\|\sum\phi_i\|$", "$\|\sum\phi_i\phi_i^T-\frac n2I\|$"}, ...
    "Interpreter", "latex", "Location", "east");

nexttile
semilogy(nList, max(res3, eps), "d-", "LineWidth", 1.4, "MarkerSize", 6, ...
    "Color", [0.85, 0.33, 0.10]);
hold on
yline(tol, "k:", "tolerance", "LineWidth", 1.0);
xline(4, "b--", "n = 4", "LineWidth", 1.0);
grid on
xlabel("robots per ring, n");
ylabel("residual norm");
title({"Third moment $\sum(\phi_i^TH\phi_i)\phi_i$", ...
       "predicted nonzero only at $n=3$"}, "Interpreter", "latex");

annotation("textbox", [0.56, 0.62, 0.30, 0.12], ...
    "String", sprintf("n=3 residual: %.3f\nn>=4 residual: < %.0e", ...
        res3(1), max(res3(nList >= 4)) + eps), ...
    "EdgeColor", [0.5, 0.5, 0.5], "BackgroundColor", [1, 1, 0.9], ...
    "FitBoxToText", "on");

finishFigure(fig, fullfile(outDir, "v1_polygon_identities.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("first/second moments at machine precision for n=3..20; third moment %.3g at n=3, below %.0e for n>=4", ...
        res3(1), tol), ...
    "max_residual_first", max(res1), ...
    "max_residual_second", max(res2), ...
    "residual_third_n3", res3(1), ...
    "max_residual_third_n_ge_4", max(res3(nList >= 4)), ...
    "figure", "v1_polygon_identities.png");
end

%% V2 — Connectivity bound (lem:connectivity-bound)

function record = checkConnectivityBound(outDir)
% Verifies Q(z) <= n S(z) and S(z) >= sqrt(2 Vf)/n on random connected
% graphs with random shifted-coordinate configurations.
rng(2, "twister");
trials = 600;
nAll = zeros(trials, 1);
qAll = zeros(trials, 1);
sAll = zeros(trials, 1);
vfAll = zeros(trials, 1);

for t = 1:trials
    n = randi([3, 12]);
    A = randomConnectedGraph(n);
    z = randn(n, 2) * (0.5 + 2 * rand());
    q = z - mean(z, 1);
    S = 0;
    for i = 1:n
        for j = i + 1:n
            if A(i, j)
                S = S + sum(abs(z(i, :) - z(j, :)));
            end
        end
    end
    nAll(t) = n;
    qAll(t) = sum(vecnorm(q, 2, 2));
    sAll(t) = S;
    vfAll(t) = 0.5 * sum(vecnorm(q, 2, 2).^2);
end

boundA = nAll .* sAll;
boundB = sqrt(2 * vfAll) ./ nAll;
slackA = boundA - qAll;
slackB = sAll - boundB;
pass = all(slackA >= -1e-9) && all(slackB >= -1e-9);

fig = figure("Visible", "off", "Position", [100, 100, 940, 400]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile
scatter(boundA, qAll, 14, nAll, "filled", "MarkerFaceAlpha", 0.55);
hold on
lim = [0, max(boundA) * 1.02];
plot(lim, lim, "k--", "LineWidth", 1.2);
grid on
axis([lim, 0, max(boundA) * 1.02]);
xlabel("$n\,S(z)$", "Interpreter", "latex");
ylabel("$Q(z)=\sum_i\|q_i\|$", "Interpreter", "latex");
title("Claim: every point on or below $y=x$", "Interpreter", "latex");
cb = colorbar; cb.Label.String = "n";

nexttile
scatter(boundB, sAll, 14, nAll, "filled", "MarkerFaceAlpha", 0.55);
hold on
lim2 = [0, max(sAll) * 1.02];
plot(lim2, lim2, "k--", "LineWidth", 1.2);
grid on
axis([0, max(boundB) * 1.05, lim2]);
xlabel("$\sqrt{2V_f}/n$", "Interpreter", "latex");
ylabel("$S(z)$", "Interpreter", "latex");
title("Claim: every point on or above $y=x$", "Interpreter", "latex");
cb2 = colorbar; cb2.Label.String = "n";

sgtitle(sprintf("Connectivity bound, %d random connected graphs, n = 3..12", trials));
finishFigure(fig, fullfile(outDir, "v2_connectivity_bound.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("%d/%d trials satisfy both inequalities; tightest slack %.3g and %.3g", ...
        trials, trials, min(slackA), min(slackB)), ...
    "trials", trials, ...
    "min_slack_Q_le_nS", min(slackA), ...
    "min_slack_S_ge_sqrt2Vf_over_n", min(slackB), ...
    "figure", "v2_connectivity_bound.png");
end

%% V3 — Finite-time formation (prop:formation)

function record = checkFormationFiniteTime(outDir)
% The continuous-time claim is finite-time convergence to exactly zero.
% A fixed-step Euler implementation cannot reach zero: the sign term
% chatters at a floor proportional to alpha*dt. This check therefore
% verifies two things: the decay respects the predicted envelope while it
% is above that floor, and the floor itself vanishes linearly with dt.
prm = baselineParams();
prm.T = 0.6;
gamma = prm.alpha - 2 * prm.beta * prm.n * prm.fDmax / prm.R;

dtList = [2e-3, 1e-3, 5e-4, 2.5e-4, 1.25e-4];
floors = zeros(size(dtList));
settleTimes = zeros(size(dtList));
bounds = zeros(size(dtList));
runs = cell(size(dtList));

for k = 1:numel(dtList)
    p = prm;
    p.dt = dtList(k);
    rng(3, "twister");
    sim = simulateBaseline(p, struct("logEvery", 1));
    rootVf = sqrt(sim.vf);

    % Chatter floor: median of the last 20% of the run.
    tailIdx = sim.times > 0.8 * p.T;
    floors(k) = median(rootVf(tailIdx));

    thresh = 3 * floors(k);
    idx = find(rootVf < thresh, 1);
    settleTimes(k) = sim.times(idx);
    bounds(k) = sqrt(2) * prm.n * sqrt(sim.vf(1)) / gamma;
    runs{k} = struct("times", sim.times, "rootVf", rootVf, ...
        "envelope", max(sqrt(sim.vf(1)) - (sqrt(2) * gamma / (2 * prm.n)) * sim.times, 0), ...
        "floor", floors(k), "thresh", thresh);
end

% Envelope must not be exceeded while the state is above the chatter floor.
excess = zeros(size(dtList));
for k = 1:numel(dtList)
    r = runs{k};
    above = r.rootVf > r.thresh;
    excess(k) = max([r.rootVf(above) - r.envelope(above); -inf]);
end

floorSlope = polyfit(log(dtList), log(floors), 1);
pass = all(settleTimes <= bounds) && all(excess <= 1e-9) && ...
    abs(floorSlope(1) - 1) < 0.25;

ref = runs{3};
fig = figure("Visible", "off", "Position", [100, 100, 1150, 400]);
tiledlayout(1, 3, "TileSpacing", "compact", "Padding", "compact");

nexttile
% Log time axis: the true decay finishes two decades before the bound, so a
% linear axis would compress it into the y-axis.
plot(max(ref.times, dtList(3)), ref.rootVf, "LineWidth", 1.6);
hold on
plot(max(ref.times, dtList(3)), ref.envelope, "r--", "LineWidth", 1.4);
yline(ref.floor, "m-.", "LineWidth", 1.2);
xline(bounds(3), "k:", "LineWidth", 1.4);
set(gca, "XScale", "log");
xlim([dtList(3), prm.T]);
grid on
xlabel("time [s], log scale");
ylabel("$\sqrt{V_f(t)}$", "Interpreter", "latex");
title(sprintf("dt = %.0e: decay vs envelope", dtList(3)));
legend({"measured", "predicted envelope", "chatter floor", ...
        sprintf("T_f bound = %.3f s", bounds(3))}, "Location", "northeast");

nexttile
hold on
for k = 1:numel(dtList)
    plot(max(runs{k}.times, dtList(k)), max(runs{k}.rootVf, 1e-16), "LineWidth", 1.3);
end
set(gca, "YScale", "log", "XScale", "log");
xline(bounds(3), "k:", "LineWidth", 1.4);
xlim([min(dtList), prm.T]);
grid on
xlabel("time [s], log scale");
ylabel("$\sqrt{V_f(t)}$ (log)", "Interpreter", "latex");
title("Floor drops with the step size");
legend([compose("dt = %.2e", dtList), "T_f bound"], "Location", "southwest");

nexttile
loglog(dtList, floors, "o-", "LineWidth", 1.5, "MarkerSize", 6);
hold on
loglog(dtList, floors(1) * (dtList / dtList(1)), "k--", "LineWidth", 1.3);
grid on
xlabel("Euler step dt [s]");
ylabel("chatter floor of $\sqrt{V_f}$", "Interpreter", "latex");
title(sprintf("Floor is O(dt), fitted slope %.2f", floorSlope(1)));
legend({"measured floor", "slope-1 reference"}, "Location", "southeast");

sgtitle("V3: finite-time formation, and the discretization floor that replaces exact zero");
finishFigure(fig, fullfile(outDir, "v3_formation_finite_time.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("settling times %s s all under the %.3f s bound; envelope never exceeded above the floor; floor scales as dt^%.2f", ...
        strjoin(compose("%.3f", settleTimes), "/"), max(bounds), floorSlope(1)), ...
    "gamma", gamma, ...
    "dt_list", dtList, ...
    "chatter_floors", floors, ...
    "floor_slope_vs_dt", floorSlope(1), ...
    "settle_times", settleTimes, ...
    "t_bound", bounds(1), ...
    "max_envelope_excess_above_floor", max(excess), ...
    "figure", "v3_formation_finite_time.png");
end

%% V4 — Exact circular gradient identity (eq:exact-gradient-identity)

function record = checkGradientIdentity(outDir)
% For the quadratic field the identity sum f(p_i) phi_i = kappa n R e is
% exact. For a Gaussian field it is only an approximation, and the residual
% should be visibly nonzero. Both are plotted for contrast.
rng(4, "twister");
n = 8;
R = 1.0;
kappa = 1.0;
phi = slots(n);
errNorms = logspace(-3, 0.7, 40);

resQuad = zeros(size(errNorms));
resGauss = zeros(size(errNorms));
for k = 1:numel(errNorms)
    e = errNorms(k) * [cos(0.7), sin(0.7)];
    p = e + R * phi;                       % centroid at e, source at origin
    fQuad = kappa * vecnorm(p, 2, 2).^2;
    resQuad(k) = norm(sum(fQuad .* phi, 1) - kappa * n * R * e);

    fGauss = -exp(-vecnorm(p, 2, 2).^2 / 2);
    gradGauss = e * exp(-norm(e)^2 / 2);   % analytic gradient of -exp(-|p|^2/2)
    resGauss(k) = norm((2 / (n * R)) * sum(fGauss .* phi, 1) - gradGauss);
end

pass = max(resQuad) < 1e-12;

fig = figure("Visible", "off", "Position", [100, 100, 940, 400]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile
loglog(errNorms, max(resQuad, eps), "o-", "LineWidth", 1.4, "MarkerSize", 5);
hold on
yline(1e-12, "k:", "machine precision", "LineWidth", 1.0);
grid on
xlabel("$\|e\|=\|p^*-p_s\|$", "Interpreter", "latex");
ylabel("residual");
ylim([1e-18, 1e-8]);
title({"Quadratic field", "$\|\sum f(p_i)\phi_i-\kappa nRe\|$ is exactly zero"}, ...
    "Interpreter", "latex");

nexttile
loglog(errNorms, resGauss, "s-", "LineWidth", 1.4, "MarkerSize", 5, ...
    "Color", [0.85, 0.33, 0.10]);
grid on
xlabel("$\|e\|$", "Interpreter", "latex");
ylabel("residual");
title({"Gaussian field, same estimator", "identity degrades to an approximation"}, ...
    "Interpreter", "latex");

sgtitle(sprintf("Circular gradient identity, n = %d, R = %.1f", n, R));
finishFigure(fig, fullfile(outDir, "v4_gradient_identity.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("quadratic residual stays below %.1e over three decades of |e|; Gaussian residual reaches %.3g", ...
        max(resQuad) + eps, max(resGauss)), ...
    "max_residual_quadratic", max(resQuad), ...
    "max_residual_gaussian", max(resGauss), ...
    "figure", "v4_gradient_identity.png");
end

%% V5 — All-informed localization bound (thm:all-informed)

function record = checkAllInformedBound(outDir)
% Verifies the transient envelope and the ultimate radius delta/(kappa R)
% across several bounded-noise seeds.
prm = baselineParams();
prm.T = 90;
seeds = 1:4;
epsAll = prm.delta / (prm.kappa * prm.R);
lambda = 2 * prm.beta * prm.kappa;

runs = cell(numel(seeds), 1);
tailMax = zeros(numel(seeds), 1);
allInformed = true;
for s = 1:numel(seeds)
    rng(100 + seeds(s), "twister");
    sim = simulateBaseline(prm, struct("logEvery", 200));
    runs{s} = sim;
    tailIdx = sim.times > 0.8 * prm.T;
    tailMax(s) = max(sim.errNorm(tailIdx));
    allInformed = allInformed && all(sim.informedCount == prm.n);
end

ref = runs{1};
envelope = exp(-lambda * ref.times) * ref.errNorm(1) + ...
    epsAll * (1 - exp(-lambda * ref.times));

margin = 1.05;
pass = all(tailMax <= epsAll * margin) && allInformed;

fig = figure("Visible", "off", "Position", [100, 100, 960, 400]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile
hold on
for s = 1:numel(seeds)
    plot(runs{s}.times, runs{s}.errNorm, "LineWidth", 1.0);
end
plot(ref.times, envelope, "k--", "LineWidth", 1.6);
yline(epsAll, "r-", "LineWidth", 1.4);
grid on
xlabel("time [s]");
ylabel("$\|p^*(t)-p_s\|$", "Interpreter", "latex");
title("Transient envelope and ultimate radius", "Interpreter", "latex");
legend([compose("seed %d", seeds), "predicted envelope", ...
        sprintf("\\epsilon = \\delta/(\\kappa R) = %.3f", epsAll)], ...
    "Location", "northeast");

nexttile
hold on
for s = 1:numel(seeds)
    plot(runs{s}.times, runs{s}.errNorm, "LineWidth", 1.0);
end
yline(epsAll, "r-", "LineWidth", 1.4);
grid on
xlim([0.6 * prm.T, prm.T]);
ylim([0, epsAll * 1.6]);
xlabel("time [s]");
ylabel("$\|p^*(t)-p_s\|$", "Interpreter", "latex");
title(sprintf("Tail detail: worst tail error %.4f vs bound %.4f", ...
    max(tailMax), epsAll));

finishFigure(fig, fullfile(outDir, "v5_all_informed_bound.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("worst tail error %.4f against bound %.4f over %d seeds; all robots informed throughout", ...
        max(tailMax), epsAll, numel(seeds)), ...
    "epsilon_all", epsAll, ...
    "worst_tail_error", max(tailMax), ...
    "seeds", numel(seeds), ...
    "all_robots_informed", allInformed, ...
    "figure", "v5_all_informed_bound.png");
end

%% V6 — Consistency of epsilon and lambda (eq:du-epsilon, eq:lambda-informed)

function record = checkEpsilonLambda(outDir)
% Verifies the algebraic identity lambda_X * epsilon(n_X) = 2 beta delta / R
% and the collapse to the all-informed values at n_X = n.
prm = baselineParams();
n = 12;
nx = 1:n;

denom = 2 * pi * nx - n * abs(sin(2 * pi * nx / n));
epsVals = 2 * pi * n * prm.delta ./ (prm.kappa * prm.R * denom);
lamVals = (prm.beta * prm.kappa / (n * pi)) * denom;
product = lamVals .* epsVals;
target = 2 * prm.beta * prm.delta / prm.R;

valid = denom > 0;
relErr = abs(product(valid) - target) / target;
epsAll = prm.delta / (prm.kappa * prm.R);
lamAll = 2 * prm.beta * prm.kappa;
collapseErr = max(abs(epsVals(end) - epsAll) / epsAll, ...
                  abs(lamVals(end) - lamAll) / lamAll);
pass = max(relErr) < 1e-12 && collapseErr < 1e-12;

fig = figure("Visible", "off", "Position", [100, 100, 960, 400]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile
yyaxis left
semilogy(nx(valid), epsVals(valid), "o-", "LineWidth", 1.4, "MarkerSize", 5);
ylabel("$\varepsilon(\underline n_{\mathcal X})$", "Interpreter", "latex");
yyaxis right
plot(nx(valid), lamVals(valid), "s-", "LineWidth", 1.4, "MarkerSize", 5);
ylabel("$\lambda_{\mathcal X}$", "Interpreter", "latex");
grid on
xlabel("guaranteed informed count $\underline n_{\mathcal X}$", "Interpreter", "latex");
title(sprintf("Error radius and contraction rate, n = %d", n));

nexttile
plot(nx(valid), product(valid), "d-", "LineWidth", 1.6, "MarkerSize", 6);
hold on
yline(target, "r--", "LineWidth", 1.4);
grid on
xlabel("$\underline n_{\mathcal X}$", "Interpreter", "latex");
ylabel("$\lambda_{\mathcal X}\,\varepsilon(\underline n_{\mathcal X})$", "Interpreter", "latex");
ylim(target * [0.9, 1.1]);
title("Claim: product is constant at $2\beta\delta/R$", "Interpreter", "latex");
legend({"measured product", sprintf("2\\beta\\delta/R = %.4g", target)}, ...
    "Location", "best");

finishFigure(fig, fullfile(outDir, "v6_epsilon_lambda_consistency.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("product constant to %.1e relative error; collapse to all-informed values exact to %.1e", ...
        max(relErr), collapseErr), ...
    "target_product", target, ...
    "max_relative_error", max(relErr), ...
    "collapse_error", collapseErr, ...
    "figure", "v6_epsilon_lambda_consistency.png");
end

%% V7 — Team-size cancellation (eq:team-epsilon-scale)

function record = checkTeamScaling(outDir)
% Verifies that epsilon_k depends on the informed fraction chi_k and the
% radius R_k, but not on the raw team size n_k.
prm = baselineParams();
nkList = 3:2:31;
chiList = [0.5, 0.7, 0.9, 1.0];

epsGrid = zeros(numel(chiList), numel(nkList));
for a = 1:numel(chiList)
    chi = chiList(a);
    for b = 1:numel(nkList)
        nk = nkList(b);
        nxk = chi * nk;
        denom = 2 * pi * nxk - nk * abs(sin(2 * pi * nxk / nk));
        epsGrid(a, b) = 2 * pi * nk * prm.delta / (prm.kappa * prm.R * denom);
    end
end

spread = max(epsGrid, [], 2) - min(epsGrid, [], 2);
pass = max(spread ./ mean(epsGrid, 2)) < 1e-12;

chiFine = linspace(0.35, 1, 300);
epsFine = 2 * pi * prm.delta ./ ...
    (prm.kappa * prm.R * (2 * pi * chiFine - abs(sin(2 * pi * chiFine))));

fig = figure("Visible", "off", "Position", [100, 100, 960, 400]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile
hold on
for a = 1:numel(chiList)
    plot(nkList, epsGrid(a, :), "o-", "LineWidth", 1.4, "MarkerSize", 5);
end
grid on
xlabel("team size $n_k$", "Interpreter", "latex");
ylabel("$\varepsilon_k$", "Interpreter", "latex");
title({"Claim: horizontal lines.", "Adding robots does not shrink the radius."}, ...
    "Interpreter", "latex");
legend(compose("\\chi_k = %.2f", chiList), "Location", "northeast");

nexttile
plot(chiFine, epsFine, "LineWidth", 1.8);
hold on
plot(chiList, epsGrid(:, 1), "ro", "MarkerSize", 8, "LineWidth", 1.4);
yline(prm.delta / (prm.kappa * prm.R), "k--", "LineWidth", 1.2);
grid on
set(gca, "YScale", "log");
xlabel("informed fraction $\chi_k$", "Interpreter", "latex");
ylabel("$\varepsilon_k$", "Interpreter", "latex");
title("The informed fraction is what matters", "Interpreter", "latex");
legend({"$\varepsilon_k(\chi_k)$", "sampled configurations", ...
        "$\delta/(\kappa R)$ at $\chi_k=1$"}, ...
    "Interpreter", "latex", "Location", "northeast");

finishFigure(fig, fullfile(outDir, "v7_team_size_cancellation.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("epsilon_k varies by at most %.1e relative across n_k = 3..31 at fixed chi_k", ...
        max(spread ./ mean(epsGrid, 2))), ...
    "max_relative_spread_over_nk", max(spread ./ mean(epsGrid, 2)), ...
    "epsilon_at_chi_1", prm.delta / (prm.kappa * prm.R), ...
    "figure", "v7_team_size_cancellation.png");
end

%% V8 — Basin-containment certificate (lem:basin-certificate)

function record = checkBasinCertificate(outDir)
% Two scenarios, because a certificate that is always satisfied proves
% nothing about whether it discriminates.
%   separated: sources far apart, certificate should hold and teams stay
%              inside their own Voronoi cell.
%   tight:     sources close enough that R_k alone nearly exhausts d_k, so
%              the certificate should fail. Whether containment survives
%              anyway is then an open empirical question, which is exactly
%              what a sufficient-but-not-necessary condition means.
scen(1) = struct("name", "separated", "sources", [-4, 0; 4, 0], "spread", 0.8);
scen(2) = struct("name", "tight", "sources", [-1.35, 0; 1.35, 0], "spread", 0.5);

results = cell(numel(scen), 1);
for sIdx = 1:numel(scen)
    results{sIdx} = runTwoTeamScenario(scen(sIdx));
end

% The lemma is sufficient, not necessary: whenever the certificate holds,
% containment must hold. The converse is not claimed.
sufficiencyOK = true;
for sIdx = 1:numel(results)
    if results{sIdx}.certHolds && ~results{sIdx}.contained
        sufficiencyOK = false;
    end
end
discriminates = results{1}.certHolds && ~results{2}.certHolds;
pass = sufficiencyOK && discriminates;

colors = lines(2);
fig = figure("Visible", "off", "Position", [100, 100, 1200, 780]);
tiledlayout(2, 3, "TileSpacing", "compact", "Padding", "compact");

for sIdx = 1:numel(results)
    r = results{sIdx};
    nRobots = size(r.trail, 2);

    nexttile
    hold on
    for k = 1:2
        idx = (1:nRobots/2) + (k - 1) * nRobots / 2;
        for i = idx
            plot(r.trail(:, i, 1), r.trail(:, i, 2), "-", ...
                "Color", [colors(k, :), 0.5], "LineWidth", 0.9);
        end
        plot(squeeze(r.trail(end, idx, 1)), squeeze(r.trail(end, idx, 2)), "o", ...
            "MarkerFaceColor", colors(k, :), "MarkerEdgeColor", "k", "MarkerSize", 5);
    end
    plot(r.sources(:, 1), r.sources(:, 2), "kp", "MarkerSize", 15, ...
        "MarkerFaceColor", "y");
    xline(0, "k--", "LineWidth", 1.2);
    grid on
    axis equal
    xlabel("x [m]"); ylabel("y [m]");
    title(sprintf("%s: d_k = %.2f, R = 1", r.name, r.dk));

    nexttile
    hold on
    for k = 1:2
        plot(r.times, r.certLhs(:, k), "LineWidth", 1.4, "Color", colors(k, :));
    end
    yline(r.dk, "r--", "LineWidth", 1.6);
    grid on
    xlabel("time [s]");
    ylabel("$\|p_k^*-p_s^{(k)}\|+R_k+q_k$", "Interpreter", "latex");
    title(sprintf("Certificate %s", ternary(r.certHolds, "HOLDS", "FAILS")));
    legend({"team 1", "team 2", sprintf("d_k = %.2f", r.dk)}, "Location", "best");

    nexttile
    hold on
    for k = 1:2
        plot(r.times, r.margin(:, k), "LineWidth", 1.4, "Color", colors(k, :));
    end
    yline(0, "r--", "LineWidth", 1.6);
    grid on
    xlabel("time [s]");
    ylabel("$m_k(t)$", "Interpreter", "latex");
    title(sprintf("Containment %s (min m_k = %.2f)", ...
        ternary(r.contained, "holds", "lost"), min(r.margin(:))));
    legend({"team 1", "team 2", "seam"}, "Location", "best");
end

sgtitle("V8: the basin certificate is sufficient, not necessary");
finishFigure(fig, fullfile(outDir, "v8_basin_certificate.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf(['separated case: certificate holds and containment holds (min margin %.2f). ' ...
        'tight case: certificate fails (max left side %.2f vs d_k %.2f) and containment %s. ' ...
        'Sufficiency never violated.'], ...
        min(results{1}.margin(:)), max(results{2}.certLhs(:)), results{2}.dk, ...
        ternary(results{2}.contained, "still holds", "is lost")), ...
    "separated_min_margin", min(results{1}.margin(:)), ...
    "separated_certificate_holds", results{1}.certHolds, ...
    "tight_min_margin", min(results{2}.margin(:)), ...
    "tight_certificate_holds", results{2}.certHolds, ...
    "tight_contained", results{2}.contained, ...
    "sufficiency_respected", sufficiencyOK, ...
    "figure", "v8_basin_certificate.png");
end

function out = runTwoTeamScenario(scenario)
% Two four-robot teams on the equal-curvature lower-envelope field.
prm = baselineParams();
prm.T = 120;
rng(8, "twister");

sources = scenario.sources;
dk = 0.5 * norm(sources(1, :) - sources(2, :));
n = 8;
R = 1.0;
teams = {1:4, 5:8};

p = [sources(1, :) + scenario.spread * randn(4, 2); ...
     sources(2, :) + scenario.spread * randn(4, 2)];
phiTeam = zeros(n, 2);
teamAgg = cell(2, 1);
for k = 1:2
    idx = teams{k};
    phiTeam(idx, :) = slots(numel(idx));
    teamAgg{k} = consensusAggregator(ringAdj(numel(idx)));
end

steps = round(prm.T / prm.dt);
logEvery = 200;
nLog = floor(steps / logEvery) + 1;
times = zeros(nLog, 1);
certLhs = zeros(nLog, 2);
margin = zeros(nLog, 2);
teamErr = zeros(nLog, 2);
trail = zeros(nLog, n, 2);
row = 1;

for s = 0:steps
    if mod(s, logEvery) == 0
        times(row) = s * prm.dt;
        trail(row, :, :) = p;
        for k = 1:2
            idx = teams{k};
            ck = mean(p(idx, :), 1);
            teamErr(row, k) = norm(ck - sources(k, :));
            qDev = max(vecnorm(p(idx, :) - ck - R * phiTeam(idx, :), 2, 2));
            certLhs(row, k) = teamErr(row, k) + R + qDev;
            margin(row, k) = assignedMargin(p(idx, :), sources, k);
        end
        row = row + 1;
    end

    dAll = zeros(n, 2);
    for k = 1:2
        dAll(:, k) = vecnorm(p - sources(k, :), 2, 2);
    end
    dOwn = min(dAll, [], 2);                       % lower-envelope field
    informed = dOwn < prm.Dmax;
    eta = prm.delta * (2 * rand(n, 1) - 1);
    sigma = (prm.kappa * dOwn.^2 + eta) .* informed + prm.fDmax * (~informed);

    u = zeros(n, 2);
    for k = 1:2
        idx = teams{k};
        z = p(idx, :) - R * phiTeam(idx, :);
        cons = signConsensus(z, teamAgg{k});
        u(idx, :) = prm.alpha * cons - ...
            (2 * prm.beta / R) * (sigma(idx) .* phiTeam(idx, :));
    end
    p = p + prm.dt * u;
end

keep = 1:row - 1;
out = struct( ...
    "name", scenario.name, ...
    "times", times(keep), ...
    "certLhs", certLhs(keep, :), ...
    "margin", margin(keep, :), ...
    "teamErr", teamErr(keep, :), ...
    "trail", trail(keep, :, :), ...
    "sources", sources, ...
    "dk", dk, ...
    "certHolds", all(certLhs(keep, :) < dk, "all"), ...
    "contained", all(margin(keep, :) > 0, "all"));
end

%% V9 — Moving-source steady lag (cor:constant-velocity)

function record = checkMovingSourceLag(outDir)
% Noiseless constant-velocity source: the predicted steady lag is the
% vector -v/(2 beta kappa). Also sweeps speed to check the affine law.
prm = baselineParams();
prm.T = 150;
prm.delta = 0;
lambda = 2 * prm.beta * prm.kappa;

% Speeds are chosen so the predicted lag v/lambda is well clear of the
% Euler chatter floor measured in V3, otherwise the test cannot resolve it.
vRef = [0.08, 0.04];
rng(9, "twister");
sim = simulateBaseline(prm, struct("logEvery", 400, "sourceVel", vRef));
predicted = -vRef / lambda;
finalLag = sim.err(end, :);
vecErr = norm(finalLag - predicted) / norm(predicted);

speeds = linspace(0.02, 0.12, 6);
measured = zeros(size(speeds));
for k = 1:numel(speeds)
    v = speeds(k) * [1, 0];
    s = simulateBaseline(prm, struct("logEvery", 4000, "sourceVel", v));
    measured(k) = norm(s.err(end, :));
end
predictedSweep = speeds / lambda;
sweepErr = max(abs(measured - predictedSweep) ./ max(predictedSweep, eps));

pass = vecErr < 0.05 && sweepErr < 0.10;

fig = figure("Visible", "off", "Position", [100, 100, 1100, 400]);
tiledlayout(1, 3, "TileSpacing", "compact", "Padding", "compact");

nexttile
plot(sim.times, sim.err(:, 1), "LineWidth", 1.4);
hold on
plot(sim.times, sim.err(:, 2), "LineWidth", 1.4);
yline(predicted(1), "b--", "LineWidth", 1.2);
yline(predicted(2), "r--", "LineWidth", 1.2);
grid on
xlabel("time [s]");
ylabel("lag components [m]");
title("Lag vector, not just its norm", "Interpreter", "latex");
legend({"e_x", "e_y", "predicted e_x", "predicted e_y"}, "Location", "east");

nexttile
plot(sim.err(:, 1), sim.err(:, 2), "LineWidth", 1.4);
hold on
plot(predicted(1), predicted(2), "rp", "MarkerSize", 16, "MarkerFaceColor", "y");
plot(0, 0, "ko", "MarkerSize", 6);
grid on
axis equal
xlabel("e_x [m]"); ylabel("e_y [m]");
title(sprintf("Lag converges to -v/(2\\beta\\kappa)\nrelative error %.2f%%", 100 * vecErr));
legend({"trajectory", "prediction", "origin"}, "Location", "best");

nexttile
plot(speeds, measured, "o", "MarkerSize", 8, "LineWidth", 1.4);
hold on
plot(speeds, predictedSweep, "r--", "LineWidth", 1.4);
grid on
xlabel("source speed $\|v\|$ [m/s]", "Interpreter", "latex");
ylabel("steady lag [m]");
title("Affine in speed, slope $1/(2\beta\kappa)$", "Interpreter", "latex");
legend({"measured", sprintf("predicted, slope %.1f", 1 / lambda)}, "Location", "southeast");

finishFigure(fig, fullfile(outDir, "v9_moving_source_lag.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("lag vector matches prediction to %.2f%%; speed sweep matches slope 1/(2 beta kappa) to %.2f%%", ...
        100 * vecErr, 100 * sweepErr), ...
    "predicted_lag", predicted, ...
    "measured_lag", finalLag, ...
    "vector_relative_error", vecErr, ...
    "sweep_max_relative_error", sweepErr, ...
    "figure", "v9_moving_source_lag.png");
end

%% V10 — Circular estimator bias (lem:estimator-bias)

function record = checkEstimatorBias(outDir)
% The bias should scale as O(R^2) when n >= 4 and degrade to O(R) at n = 3,
% because the third-moment identity fails there.
field = mixtureField();
p = [0.6, -0.4];
gradTrue = mixtureGrad(p, field);
radii = logspace(-2, -0.2, 25);
nList = [3, 4, 6, 8];

bias = zeros(numel(nList), numel(radii));
for a = 1:numel(nList)
    phi = slots(nList(a));
    for b = 1:numel(radii)
        R = radii(b);
        samples = p + R * phi;
        vals = mixtureValue(samples, field);
        gR = (2 / (nList(a) * R)) * sum(vals .* phi, 1);
        bias(a, b) = norm(gR - gradTrue);
    end
end

slopes = zeros(numel(nList), 1);
for a = 1:numel(nList)
    coef = polyfit(log(radii), log(bias(a, :)), 1);
    slopes(a) = coef(1);
end

pass = abs(slopes(1) - 1) < 0.25 && all(abs(slopes(2:end) - 2) < 0.25);

fig = figure("Visible", "off", "Position", [100, 100, 960, 420]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile
hold on
mk = ["d", "o", "s", "^"];
for a = 1:numel(nList)
    loglog(radii, bias(a, :), mk(a) + "-", "LineWidth", 1.4, "MarkerSize", 5);
end
set(gca, "XScale", "log", "YScale", "log");
refR = radii;
loglog(refR, 0.5 * refR.^2 * bias(2, end) / (0.5 * radii(end)^2), "k:", "LineWidth", 1.4);
grid on
xlabel("ring radius R [m]");
ylabel("$\|g_R(p)-\nabla f(p)\|$", "Interpreter", "latex");
title("Estimator bias against ring radius", "Interpreter", "latex");
legend([compose("n = %d", nList), "slope-2 reference"], "Location", "southeast");

nexttile
bar(categorical(compose("n = %d", nList)), slopes, "FaceColor", [0.30, 0.60, 0.80]);
hold on
yline(2, "r--", "LineWidth", 1.6);
yline(1, "k:", "LineWidth", 1.4);
grid on
ylabel("fitted log-log slope");
ylim([0, 2.6]);
title({"Predicted slope 2 for $n\geq4$", "and 1 for $n=3$"}, "Interpreter", "latex");
for a = 1:numel(nList)
    text(a, slopes(a) + 0.1, sprintf("%.2f", slopes(a)), ...
        "HorizontalAlignment", "center", "FontWeight", "bold");
end

finishFigure(fig, fullfile(outDir, "v10_estimator_bias.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("fitted slopes: n=3 gives %.2f (expected 1), n>=4 gives %s (expected 2)", ...
        slopes(1), strjoin(compose("%.2f", slopes(2:end)'), ", ")), ...
    "n_values", nList, ...
    "fitted_slopes", slopes', ...
    "figure", "v10_estimator_bias.png");
end

%% V11 — Local trapping of ideal ascent (prop:no-global-guarantee)

function record = checkLocalTrapping(outDir)
% Maps the basin of attraction of every local maximum under the ideal
% ascent flow. A nonempty suboptimal basin falsifies any global claim.
field = mixtureField();
gridN = 220;
xs = linspace(-6, 6, gridN);
ys = linspace(-5, 5, gridN);
[X, Y] = meshgrid(xs, ys);
P = [X(:), Y(:)];

dt = 0.05;
for s = 1:3000
    P = P + dt * mixtureGrad(P, field);
end

peaks = refinePeaks(field);
[~, basin] = min(pdist2Simple(P, peaks), [], 2);
basinMap = reshape(basin, size(X));

fVals = mixtureValue(peaks, field);
[~, globalIdx] = max(fVals);
fraction = zeros(size(peaks, 1), 1);
for j = 1:size(peaks, 1)
    fraction(j) = mean(basin == j);
end
subOptimalShare = 1 - fraction(globalIdx);
pass = subOptimalShare > 0.05;

Z = reshape(mixtureValue([X(:), Y(:)], field), size(X));

fig = figure("Visible", "off", "Position", [100, 100, 1060, 430]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile
contourf(X, Y, Z, 25, "LineColor", "none");
hold on
plot(peaks(:, 1), peaks(:, 2), "wo", "MarkerSize", 9, "LineWidth", 1.6);
plot(peaks(globalIdx, 1), peaks(globalIdx, 2), "wp", "MarkerSize", 18, ...
    "MarkerFaceColor", "y");
colorbar
axis equal tight
xlabel("x [m]"); ylabel("y [m]");
title("Scalar field with three local maxima");

nexttile
imagesc(xs, ys, basinMap);
set(gca, "YDir", "normal");
hold on
colormap(gca, lines(size(peaks, 1)));
for j = 1:size(peaks, 1)
    plot(peaks(j, 1), peaks(j, 2), "ko", "MarkerSize", 9, ...
        "MarkerFaceColor", "w", "LineWidth", 1.4);
    text(peaks(j, 1), peaks(j, 2) - 0.55, ...
        sprintf("f=%.2f (%.0f%%)", fVals(j), 100 * fraction(j)), ...
        "HorizontalAlignment", "center", "FontWeight", "bold");
end
axis equal tight
xlabel("x [m]"); ylabel("y [m]");
title(sprintf("Basins of the ideal ascent flow\n%.0f%% of starts converge to a suboptimal peak", ...
    100 * subOptimalShare));

finishFigure(fig, fullfile(outDir, "v11_local_trapping.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("%.1f%% of the %d sampled initial centroids converge to a suboptimal peak, so the suboptimal basins are nonempty", ...
        100 * subOptimalShare, gridN^2), ...
    "peak_count", size(peaks, 1), ...
    "peak_values", fVals', ...
    "basin_fractions", fraction', ...
    "suboptimal_share", subOptimalShare, ...
    "figure", "v11_local_trapping.png");
end

%% V12 — Multi-start selection condition (thm:multistart)

function record = checkMultiStartSelection(outDir)
% The theorem guarantees correct selection when 2(L_f eps_loc + delta_F) < Delta.
% Sweeping the score uncertainty should show perfect selection strictly
% inside the guaranteed region and degradation only outside it.
rng(12, "twister");
field = mixtureField();
peaks = refinePeaks(field);
fVals = mixtureValue(peaks, field);
[fBest, globalIdx] = max(fVals);
Delta = fBest - max(fVals(setdiff(1:numel(fVals), globalIdx)));

Lf = 0.6;
epsLoc = 0.05;
deltaFList = linspace(0, 1.2 * (Delta / 2), 25);
trials = 4000;
accuracy = zeros(size(deltaFList));

for a = 1:numel(deltaFList)
    dF = deltaFList(a);
    correct = 0;
    for t = 1:trials
        locErr = epsLoc * (2 * rand(numel(fVals), 1) - 1);
        scoreErr = dF * (2 * rand(numel(fVals), 1) - 1);
        reported = fVals(:) + Lf * locErr + scoreErr;
        [~, pick] = max(reported);
        correct = correct + (pick == globalIdx);
    end
    accuracy(a) = correct / trials;
end

margin = 2 * (Lf * epsLoc + deltaFList);
guaranteed = margin < Delta;
pass = all(accuracy(guaranteed) == 1);

fig = figure("Visible", "off", "Position", [100, 100, 960, 400]);
tiledlayout(1, 2, "TileSpacing", "compact", "Padding", "compact");

nexttile
hold on
bar(1:numel(fVals), fVals, 0.5, "FaceColor", [0.55, 0.72, 0.86]);
errorbar(1:numel(fVals), fVals, ...
    repmat(Lf * epsLoc + deltaFList(end), numel(fVals), 1), "k.", "LineWidth", 1.4);
plot(globalIdx, fBest, "rp", "MarkerSize", 18, "MarkerFaceColor", "y");
grid on
xticks(1:numel(fVals));
xlabel("local peak index");
ylabel("$f(\ell_j)$", "Interpreter", "latex");
title(sprintf("Peak scores, gap \\Delta = %.3f", Delta));
legend({"true score", "worst-case reporting error", "global maximum"}, ...
    "Location", "southwest");

nexttile
plot(margin, 100 * accuracy, "o-", "LineWidth", 1.5, "MarkerSize", 5);
hold on
xline(Delta, "r--", "LineWidth", 1.6);
yline(100, "k:", "LineWidth", 1.2);
grid on
xlabel("$2(L_f\epsilon_{\mathrm{loc}}+\delta_F)$", "Interpreter", "latex");
ylabel("selection accuracy [%]");
ylim([0, 105]);
title({"Claim: 100\% accuracy left of the line.", "Right of it the theorem says nothing."}, ...
    "Interpreter", "latex");
legend({"measured", sprintf("\\Delta = %.3f", Delta)}, "Location", "southwest");

finishFigure(fig, fullfile(outDir, "v12_multistart_selection.png"));

record = struct( ...
    "pass", pass, ...
    "verdict", sprintf("selection accuracy is 100%% at every sampled point satisfying the condition (%d of %d points), and falls to %.0f%% at the widest uncertainty", ...
        sum(guaranteed), numel(deltaFList), 100 * accuracy(end)), ...
    "peak_gap", Delta, ...
    "trials_per_point", trials, ...
    "min_accuracy_inside_condition", min(accuracy(guaranteed)), ...
    "accuracy_at_widest_uncertainty", accuracy(end), ...
    "figure", "v12_multistart_selection.png");
end

%% Shared model helpers

function prm = baselineParams()
prm = struct();
prm.n = 6;
prm.R = 1.0;
prm.kappa = 1.0;
prm.delta = 0.05;
prm.alpha = 100.0;
prm.beta = 0.05;
prm.dt = 5e-4;
prm.T = 60;
prm.Dmax = 5.0;
prm.fDmax = prm.kappa * prm.Dmax^2 + prm.delta;
prm.p0Center = [1.5, -1.2];
prm.p0Spread = 1.2;
end

function out = simulateBaseline(prm, opts)
% Single-team baseline controller on a quadratic bowl. The source may drift
% at a constant velocity, which is what the Gap 2 corollary needs.
if ~isfield(opts, "logEvery"), opts.logEvery = 100; end
if ~isfield(opts, "sourceVel"), opts.sourceVel = [0, 0]; end

n = prm.n;
phi = slots(n);
agg = consensusAggregator(ringAdj(n));
p = prm.p0Center + prm.p0Spread * randn(n, 2);

steps = round(prm.T / prm.dt);
nLog = floor(steps / opts.logEvery) + 1;
times = zeros(nLog, 1);
vf = zeros(nLog, 1);
errVec = zeros(nLog, 2);
informedCount = zeros(nLog, 1);
row = 1;

for s = 0:steps
    t = s * prm.dt;
    ps = opts.sourceVel * t;
    d = vecnorm(p - ps, 2, 2);
    informed = d < prm.Dmax;
    if prm.delta > 0
        eta = prm.delta * (2 * rand(n, 1) - 1);
    else
        eta = zeros(n, 1);
    end
    sigma = (prm.kappa * d.^2 + eta) .* informed + prm.fDmax * (~informed);

    if mod(s, opts.logEvery) == 0
        z = p - prm.R * phi;
        q = z - mean(z, 1);
        times(row) = t;
        vf(row) = 0.5 * sum(vecnorm(q, 2, 2).^2);
        errVec(row, :) = mean(p, 1) - ps;
        informedCount(row) = sum(informed);
        row = row + 1;
    end

    z = p - prm.R * phi;
    cons = signConsensus(z, agg);
    u = prm.alpha * cons - (2 * prm.beta / prm.R) * (sigma .* phi);
    p = p + prm.dt * u;
end

out = struct( ...
    "times", times(1:row - 1), ...
    "vf", vf(1:row - 1), ...
    "err", errVec(1:row - 1, :), ...
    "errNorm", vecnorm(errVec(1:row - 1, :), 2, 2), ...
    "informedCount", informedCount(1:row - 1));
end

function phi = slots(n)
theta = 2 * pi * (0:(n - 1))' / n;
phi = [cos(theta), sin(theta)];
end

function A = ringAdj(n)
A = zeros(n, n);
for i = 1:n
    j = mod(i, n) + 1;
    A(i, j) = 1;
    A(j, i) = 1;
end
end

function agg = consensusAggregator(A)
% Precomputes the directed edge list and the edge-to-robot accumulation
% matrix, so the sign consensus becomes one sparse product per step.
[src, dst] = find(A);
m = numel(src);
agg = struct("src", src, "dst", dst, ...
    "M", sparse(src, 1:m, 1, size(A, 1), m));
end

function cons = signConsensus(z, agg)
% cons(i,:) = sum over neighbours j of sign(z_j - z_i), componentwise.
cons = agg.M * sign(z(agg.dst, :) - z(agg.src, :));
end

function A = randomConnectedGraph(n)
% Random spanning tree, then a few extra edges.
perm = randperm(n);
A = zeros(n, n);
for i = 2:n
    j = perm(randi(i - 1));
    A(perm(i), j) = 1;
    A(j, perm(i)) = 1;
end
extra = randi([0, n]);
for e = 1:extra
    i = randi(n); j = randi(n);
    if i ~= j
        A(i, j) = 1;
        A(j, i) = 1;
    end
end
end

function m = assignedMargin(points, sources, own)
dOwn = vecnorm(points - sources(own, :), 2, 2);
others = sources;
others(own, :) = [];
dOther = zeros(size(points, 1), size(others, 1));
for j = 1:size(others, 1)
    dOther(:, j) = vecnorm(points - others(j, :), 2, 2);
end
m = min(min(dOther, [], 2) - dOwn);
end

function field = mixtureField()
field = struct( ...
    "A", [1.00, 0.78, 0.60], ...
    "c", [0.0, 0.0; 3.2, 1.6; -2.8, 1.9], ...
    "s", [1.10, 0.95, 0.85]);
end

function v = mixtureValue(P, field)
v = zeros(size(P, 1), 1);
for j = 1:numel(field.A)
    dsq = sum((P - field.c(j, :)).^2, 2);
    v = v + field.A(j) * exp(-dsq / (2 * field.s(j)^2));
end
end

function g = mixtureGrad(P, field)
g = zeros(size(P));
for j = 1:numel(field.A)
    diff = P - field.c(j, :);
    dsq = sum(diff.^2, 2);
    w = field.A(j) * exp(-dsq / (2 * field.s(j)^2)) / field.s(j)^2;
    g = g - w .* diff;
end
end

function peaks = refinePeaks(field)
% Gradient ascent from each mixture center, then deduplicate.
P = field.c;
for s = 1:20000
    P = P + 0.02 * mixtureGrad(P, field);
end
peaks = P(1, :);
for j = 2:size(P, 1)
    if min(vecnorm(peaks - P(j, :), 2, 2)) > 1e-3
        peaks = [peaks; P(j, :)]; %#ok<AGROW>
    end
end
end

function D = pdist2Simple(A, B)
D = zeros(size(A, 1), size(B, 1));
for j = 1:size(B, 1)
    D(:, j) = vecnorm(A - B(j, :), 2, 2);
end
end

%% Output helpers

function finishFigure(fig, path)
exportgraphics(fig, path, "Resolution", 150);
close(fig);
end

function summary = loadExistingSummary(path)
% Running a subset of checks should update, not discard, earlier records.
summary = struct();
if ~isfile(path)
    return
end
try
    summary = jsondecode(fileread(path));
    if isfield(summary, "meta")
        summary = rmfield(summary, "meta");
    end
catch
    summary = struct();
end
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end

function writeJson(path, data)
try
    text = jsonencode(data, PrettyPrint=true);
catch
    text = jsonencode(data);
end
fid = fopen(path, "w");
if fid < 0
    error("Cannot write %s", path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "%s", text);
end
