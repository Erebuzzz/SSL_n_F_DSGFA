% SingleIntegrator.m
% Self-contained MATLAB simulation of the Du et al. (2024) sign gradient-free
% source localization and formation controller on single-integrator robots:
%   p_dot_i = u_i
%
% Run from anywhere (no addpath / no external helpers required):
%   cd sims
%   SingleIntegrator
%
% Default parameters match configs/paper_bounded_validation.json (theorem-faithful
% bounded-noise validation). Outputs go to sims/outputs/SingleIntegrator/.

%% Configuration
% Toggle outputs and pick the preset. All numeric parameters live in Parameters.

savePlots = true;
saveAnimation = true;    % writes motion.gif (~120 frames; can be slow on fine dt)
saveTelemetry = false;    % writes telemetry.json (time-series; decimated if very long)
telemetryMaxSamples = 10000;  % cap JSON length; full series stays in result.mat
animationFps = 20;

% Output folder next to this script (independent of the MATLAB current folder).
scriptDir = fileparts(mfilename("fullpath"));
runDir = fullfile(scriptDir, "outputs", "SingleIntegrator");
if ~exist(runDir, "dir")
    mkdir(runDir);
end

%% Parameters
% Paper Section IV defaults with bounded noise (Assumption 2 / theorem bound).

n = 6;
source = [5.5, 5.5];
kappa = 1.0;
R = 2.0;                 % formation radius
Dmax = 12.0;             % sensing saturation range
alpha = 100.0;           % formation (sign) gain
beta = 0.05;             % source-seeking gain; alpha/beta = 2000 > threshold ~1730
noiseModel = "bounded";  % "none" | "gaussian" | "bounded"
noiseStd = 0.2;
noiseBound = 0.2;
duration = 60.0;
dt = 0.0005;
seed = 1;
signBoundaryLayer = 0.0; % 0 = exact paper sgn (single-integrator parity mode)
topologyName = "paper_fig1_reconstructed";

% Alternative: paper-like Gaussian noise (configs/paper_default.json).
% noiseModel = "gaussian"; noiseStd = 0.2; noiseBound = 0.2;

% Alternative: paper Fig. 3 timescale (small alpha; see docs/RUNNING_MODES.md).
% alpha = 1.0; beta = 0.03; duration = 80.0; dt = 0.001;

% Deterministic layout used by matlab/sgf_config.m (n = 6 only). Shifted so the
% geometry is relative to `source` (source = [5.5 5.5] reproduces the base).
initialPositions = [
    0.0,  0.0;
   -2.5, -0.5;
    5.0,  1.0;
    0.5,  3.5;
    1.0,  4.0;
    6.0,  2.0
] + (source - [5.5, 5.5]);
informedMask = true(n, 1);  % all robots sensing-capable (still gated by Dmax)

%% System Matrices
% Undirected Fig. 1 reconstruction (zero-based edges converted to 1-based).
% Edges: (0,1),(0,2),(0,5),(1,2),(1,4),(2,3),(3,4),(4,5)

adjacency = zeros(n, n);
edges = [1 2; 1 3; 1 6; 2 3; 2 5; 3 4; 4 5; 5 6];
for e = 1:size(edges, 1)
    i = edges(e, 1);
    j = edges(e, 2);
    adjacency(i, j) = 1;
    adjacency(j, i) = 1;
end

% Circular formation slots phi_i = [cos(2*pi*i/n), sin(2*pi*i/n)].
thetaSlots = 2 * pi * (0:(n - 1))' / n;
phi = [cos(thetaSlots), sin(thetaSlots)];

%% Initialization
rng(seed, "twister");

steps = round(duration / dt);
times = (0:steps)' * dt;
positions = zeros(n, 2, steps + 1);
centroid = zeros(steps + 1, 2);
formationError = zeros(steps + 1, 1);
localizationError = zeros(steps + 1, 1);
nInformed = zeros(steps + 1, 1);
positions(:, :, 1) = initialPositions;

cfg = struct( ...
    "n", n, "source", source, "kappa", kappa, "R", R, "Dmax", Dmax, ...
    "alpha", alpha, "beta", beta, "noise_model", noiseModel, ...
    "noise_std", noiseStd, "noise_bound", noiseBound, ...
    "sign_boundary_layer", signBoundaryLayer, "informed_mask", informedMask, ...
    "topology_name", topologyName, "dt", dt, "duration", duration, "seed", seed);

%% Simulation
% Explicit Euler: p <- p + dt * u, with Eq. 4 control on the robot positions.

for step = 1:(steps + 1)
    current = positions(:, :, step);
    [sigma, informed] = measureSource(current, cfg);
    centroid(step, :) = mean(current, 1);
    formationError(step) = formationErrorValue(current, R, phi);
    localizationError(step) = norm(centroid(step, :) - source);
    nInformed(step) = sum(informed);

    if step == steps + 1
        break
    end

    u = eq4Control(current, adjacency, phi, sigma, cfg);
    positions(:, :, step + 1) = current + dt * u;
end

if min(nInformed) == 0
    warning(["No robot is ever within Dmax of the source: the centroid cannot " ...
        "localize. Move the source closer, raise Dmax, or start nearer the source."]);
end

bounds = theoryBounds(cfg, min(nInformed));
summary = buildSiSummary(cfg, times, formationError, localizationError, nInformed, bounds);

result = struct();
result.times = times;
result.positions = positions;
result.centroid = centroid;
result.formation_error = formationError;
result.localization_error = localizationError;
result.n_informed = nInformed;
result.summary = summary;

%% Visualization
if savePlots
    plotTrajectory(result, cfg, phi, runDir);
    plotSeries(times, formationError, "Formation error", "formation error", ...
        [], fullfile(runDir, "formation_error.png"));
    plotFormationErrorPerRobot(result, cfg, phi, runDir);
    epsilon = [];
    if summary.validation.bound_applicable && ~isempty(summary.validation.epsilon)
        epsilon = summary.validation.epsilon;
    end
    plotSeries(times, localizationError, "Localization error", ...
        "||centroid - source||", epsilon, fullfile(runDir, "localization_error.png"));
    if saveAnimation
        writeAnimation(result, cfg, phi, epsilon, runDir, animationFps);
    end
end

save(fullfile(runDir, "result.mat"), "result", "cfg");
writeJson(fullfile(runDir, "summary.json"), summary);
if saveTelemetry
    telemetry = buildSiTelemetry(result, cfg, phi, telemetryMaxSamples);
    writeJson(fullfile(runDir, "telemetry.json"), telemetry);
end

fprintf("SingleIntegrator complete.\n");
fprintf("Output folder: %s\n", runDir);
fprintf("Final localization error: %.6g\n", summary.metrics.final_localization_error);
if summary.validation.bound_applicable
    fprintf("Inside theorem bound: %d (epsilon=%.4g)\n", ...
        summary.validation.inside_bound, summary.validation.epsilon);
end

%% Helper Functions

function [sigma, informed] = measureSource(positions, cfg)
% Noisy saturated quadratic measurements of f(z) = kappa * ||z - p_s||^2.
distances = sqrt(sum((positions - cfg.source) .^ 2, 2));
informed = distances < cfg.Dmax;
if isfield(cfg, "informed_mask")
    informed = informed & cfg.informed_mask;
end
sigma = ones(cfg.n, 1) * (cfg.kappa * cfg.Dmax * cfg.Dmax + cfg.noise_bound);
if any(informed)
    sourceDelta = positions(informed, :) - cfg.source;
    clean = cfg.kappa * sum(sourceDelta .^ 2, 2);
    sigma(informed) = clean + sampleNoise(cfg, sum(informed));
end
end

function noise = sampleNoise(cfg, count)
switch cfg.noise_model
    case "none"
        noise = zeros(count, 1);
    case "gaussian"
        noise = cfg.noise_std * randn(count, 1);
    case "bounded"
        noise = cfg.noise_bound * (2 * rand(count, 1) - 1);
    otherwise
        error("Unsupported noise model: %s", cfg.noise_model);
end
end

function controls = eq4Control(positions, adjacency, phi, sigma, cfg)
% Paper Eq. 4: u_i = alpha * sum_j sgn(z_j - z_i) - (2 beta / R) sigma_i phi_i.
epsBl = 0.0;
if isfield(cfg, "sign_boundary_layer") && ~isempty(cfg.sign_boundary_layer)
    epsBl = cfg.sign_boundary_layer;
end
z = positions - cfg.R * phi;
controls = zeros(size(positions));
for i = 1:cfg.n
    neighbors = find(adjacency(i, :) ~= 0);
    if ~isempty(neighbors)
        controls(i, :) = controls(i, :) + ...
            cfg.alpha * sum(satSign(z(neighbors, :) - z(i, :), epsBl), 1);
    end
    controls(i, :) = controls(i, :) - (2 * cfg.beta / cfg.R) * sigma(i) * phi(i, :);
end
end

function s = satSign(d, epsBl)
% Exact sgn when epsBl = 0; otherwise boundary-layer sat(d/epsBl).
if epsBl > 0
    s = max(min(d / epsBl, 1.0), -1.0);
else
    s = sign(d);
end
end

function value = formationErrorValue(positions, R, phi)
z = positions - R * phi;
zCentroid = mean(z, 1);
value = sqrt(sum((z - zCentroid) .^ 2, "all"));
end

function bounds = theoryBounds(cfg, minNInformed)
fDmax = cfg.kappa * cfg.Dmax * cfg.Dmax + cfg.noise_bound;
gainThreshold = 4 * cfg.n * fDmax / cfg.R;
epsilon = [];
if minNInformed > 0
    denominator = cfg.kappa * cfg.R * ( ...
        2 * pi * minNInformed - cfg.n * abs(sin(2 * pi * minNInformed / cfg.n)));
    if denominator <= 0
        error("Epsilon denominator must be positive.");
    end
    epsilon = 2 * pi * cfg.n * cfg.noise_bound / denominator;
end
bounds = struct();
bounds.gain_threshold = gainThreshold;
bounds.epsilon = epsilon;
bounds.bound_applicable = ismember(cfg.noise_model, ["bounded", "none"]) && minNInformed > 0;
end

function summary = buildSiSummary(cfg, times, formationError, localizationError, nInformed, bounds)
finalFormation = formationError(end);
finalLocalization = localizationError(end);
insideBound = [];
if bounds.bound_applicable
    insideBound = finalLocalization <= bounds.epsilon;
end
tailStart = max(1, floor(0.75 * numel(localizationError)));
formationTail = formationError(tailStart:end);
localizationTail = localizationError(tailStart:end);
formationThreshold = max(0.1, 0.05 * formationError(1));
if isempty(bounds.epsilon)
    localizationThreshold = 0.1;
else
    localizationThreshold = bounds.epsilon;
end

summary = struct();
summary.parameters = struct( ...
    "n", cfg.n, "source", cfg.source, "kappa", cfg.kappa, "radius", cfg.R, ...
    "dmax", cfg.Dmax, "alpha", cfg.alpha, "beta", cfg.beta, ...
    "duration", cfg.duration, "dt", cfg.dt, "seed", cfg.seed, ...
    "noise_model", cfg.noise_model, "noise_std", cfg.noise_std, ...
    "noise_bound", cfg.noise_bound, "topology", cfg.topology_name);
summary.validation = struct( ...
    "gain_ratio", cfg.alpha / cfg.beta, ...
    "gain_threshold", bounds.gain_threshold, ...
    "gain_condition_passed", (cfg.alpha / cfg.beta) > bounds.gain_threshold, ...
    "min_n_informed", min(nInformed), "final_n_informed", nInformed(end), ...
    "epsilon", bounds.epsilon, "bound_applicable", bounds.bound_applicable, ...
    "inside_bound", insideBound);
summary.metrics = struct( ...
    "initial_formation_error", formationError(1), ...
    "final_formation_error", finalFormation, ...
    "initial_localization_error", localizationError(1), ...
    "final_localization_error", finalLocalization, ...
    "formation_error_drop", formationError(1) - finalFormation, ...
    "localization_error_drop", localizationError(1) - finalLocalization, ...
    "tail_formation_error_span", max(formationTail) - min(formationTail), ...
    "tail_formation_error_std", std(formationTail), ...
    "tail_localization_error_span", max(localizationTail) - min(localizationTail), ...
    "tail_localization_error_std", std(localizationTail), ...
    "formation_threshold", formationThreshold, ...
    "formation_entry_time", firstEntryTime(times, formationError, formationThreshold), ...
    "localization_threshold", localizationThreshold, ...
    "localization_entry_time", firstEntryTime(times, localizationError, localizationThreshold), ...
    "time_inside_localization_threshold_after_entry", ...
        timeInsideAfterEntry(times, localizationError, localizationThreshold));
end

function value = firstEntryTime(times, values, threshold)
idx = find(values <= threshold, 1, "first");
if isempty(idx)
    value = [];
else
    value = times(idx);
end
end

function value = timeInsideAfterEntry(times, values, threshold)
idx = find(values <= threshold, 1, "first");
if isempty(idx) || numel(times) < 2
    value = 0;
else
    dtLocal = times(2) - times(1);
    value = sum(values(idx:end) <= threshold) * dtLocal;
end
end

function plotTrajectory(result, cfg, phi, runDir)
fig = figure("Visible", "off");
hold on
for i = 1:cfg.n
    xy = squeeze(result.positions(i, :, :))';
    plot(xy(:, 1), xy(:, 2), "LineWidth", 1.0);
    plot(xy(1, 1), xy(1, 2), "o", "MarkerSize", 5);
    plot(xy(end, 1), xy(end, 2), "s", "MarkerSize", 6);
end
finalCentroid = result.centroid(end, :);
circle = finalCentroid + cfg.R * phi;
circle = [circle; circle(1, :)];
plot(circle(:, 1), circle(:, 2), "k:", "LineWidth", 1.2);
plot(result.centroid(:, 1), result.centroid(:, 2), "k-", "LineWidth", 1.0);
plot(cfg.source(1), cfg.source(2), "rp", "MarkerSize", 14, "MarkerFaceColor", "r");
plot(finalCentroid(1), finalCentroid(2), "kx", "MarkerSize", 10, "LineWidth", 1.5);
axis equal
grid on
xlabel("x [m]");
ylabel("y [m]");
title(sprintf("Robot trajectories (single integrator, %s)", cfg.topology_name));
saveas(fig, fullfile(runDir, "trajectory.png"));
close(fig);
end

function plotFormationErrorPerRobot(result, cfg, phi, runDir)
T = numel(result.times);
z = result.positions - cfg.R * phi;
zStar = mean(z, 1);
diff = z - zStar;
e = squeeze(sqrt(sum(diff .^ 2, 2)));
if T == 1
    e = e(:);
end
fig = figure("Visible", "off");
hold on
for i = 1:cfg.n
    plot(result.times, e(i, :), "LineWidth", 1.0);
end
grid on
xlabel("time [s]");
ylabel("||z_i - z*||");
title("Per-robot formation error (paper Fig. 3)");
legend(arrayfun(@(i) sprintf("robot %d", i - 1), 1:cfg.n, "UniformOutput", false), ...
    "Location", "best", "FontSize", 7);
saveas(fig, fullfile(runDir, "formation_error_per_robot.png"));
close(fig);
end

function plotSeries(times, values, ttl, ylab, bound, path)
fig = figure("Visible", "off");
plot(times, values, "LineWidth", 1.2);
hold on
if ~isempty(bound)
    yline(bound, "r--", "LineWidth", 1.0);
    legend({"error", "theoretical bound"}, "Location", "best");
end
grid on
xlabel("time [s]");
ylabel(ylab);
title(ttl);
saveas(fig, path);
close(fig);
end

function writeAnimation(result, cfg, phi, epsilon, runDir, animationFps)
gifPath = fullfile(runDir, "motion.gif");
nFrames = numel(result.times);
stride = max(1, floor(nFrames / 120));
frames = 1:stride:nFrames;
histStride = max(1, floor(nFrames / 1200));
bgT = result.times(1:histStride:end);
bgFe = result.formation_error(1:histStride:end);
bgLe = result.localization_error(1:histStride:end);
meta = sprintf("SingleIntegrator  topology=%s  alpha/beta=%g  n=%d", ...
    cfg.topology_name, result.summary.validation.gain_ratio, cfg.n);
allXy = reshape(permute(result.positions, [3 1 2]), [], 2);
pad = 1.0;
xlims = [min([allXy(:, 1); cfg.source(1)]) - pad, max([allXy(:, 1); cfg.source(1)]) + pad];
ylims = [min([allXy(:, 2); cfg.source(2)]) - pad, max([allXy(:, 2); cfg.source(2)]) + pad];
fig = figure("Visible", "off", "Position", [100, 100, 1000, 500]);
first = true;
for idx = frames
    clf(fig);
    subplot(1, 2, 1);
    hold on
    hist = unique([1:histStride:idx, idx]);
    for i = 1:cfg.n
        xi = reshape(result.positions(i, 1, hist), [], 1);
        yi = reshape(result.positions(i, 2, hist), [], 1);
        plot(xi, yi, "LineWidth", 0.7);
    end
    current = result.positions(:, :, idx);
    plot(current(:, 1), current(:, 2), "bo", "MarkerFaceColor", "b", "MarkerSize", 5);
    c = result.centroid(idx, :);
    circle = c + cfg.R * phi;
    circle = [circle; circle(1, :)];
    plot(circle(:, 1), circle(:, 2), "k:", "LineWidth", 1.2);
    plot(c(1), c(2), "kx", "MarkerSize", 10, "LineWidth", 1.5);
    plot(cfg.source(1), cfg.source(2), "rp", "MarkerSize", 14, "MarkerFaceColor", "r");
    xlim(xlims); ylim(ylims);
    axis equal
    grid on
    xlabel("x [m]"); ylabel("y [m]");
    title(sprintf("Formation and source (t = %.2f s)", result.times(idx)));
    subplot(2, 2, 2);
    plot(bgT, bgFe, "Color", [0.6 0.6 0.6], "LineWidth", 0.8);
    hold on
    plot(result.times(hist), result.formation_error(hist), "b", "LineWidth", 1.2);
    plot(result.times(idx), result.formation_error(idx), "ro", "MarkerFaceColor", "r");
    grid on
    ylabel("formation error");
    title("Formation error");
    subplot(2, 2, 4);
    plot(bgT, bgLe, "Color", [0.6 0.6 0.6], "LineWidth", 0.8);
    hold on
    plot(result.times(hist), result.localization_error(hist), "b", "LineWidth", 1.2);
    plot(result.times(idx), result.localization_error(idx), "ro", "MarkerFaceColor", "r");
    if ~isempty(epsilon)
        yline(epsilon, "r--", "LineWidth", 1.0);
    end
    grid on
    xlabel("time [s]"); ylabel("||centroid - source||");
    title("Localization error");
    sgtitle(meta);
    frame = getframe(fig);
    [imageData, colorMap] = rgb2ind(frame2im(frame), 256);
    if first
        imwrite(imageData, colorMap, gifPath, "gif", "LoopCount", Inf, ...
            "DelayTime", 1 / animationFps);
        first = false;
    else
        imwrite(imageData, colorMap, gifPath, "gif", "WriteMode", "append", ...
            "DelayTime", 1 / animationFps);
    end
end
close(fig);
end

function writeJson(path, data)
try
    text = jsonencode(data, PrettyPrint=true);
catch
    text = jsonencode(data);
end
fid = fopen(path, "w");
if fid < 0
    error("Could not open %s for writing.", path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, "%s", text);
end

function telemetry = buildSiTelemetry(result, cfg, phi, maxSamples)
% Build JSON-serializable per-step telemetry (decimated when n_samples > maxSamples).
[idx, stride] = decimateIndices(numel(result.times), maxSamples);
T = numel(idx);
n = cfg.n;

fePerRobot = zeros(n, T);
posOut = zeros(T, n, 2);
for k = 1:T
    t = idx(k);
    p = result.positions(:, :, t);
    posOut(k, :, :) = p;
    z = p - cfg.R * phi;
    zc = mean(z, 1);
    fePerRobot(:, k) = sqrt(sum((z - zc) .^ 2, 2));
end

telemetry = struct();
telemetry.meta = struct( ...
    "model", "single_integrator", ...
    "generated_at", char(datetime("now", "TimeZone", "local", "Format", "yyyy-MM-dd'T'HH:mm:ss")), ...
    "dt", cfg.dt, "duration", cfg.duration, "n_robots", n, ...
    "seed", cfg.seed, "topology", char(cfg.topology_name), ...
    "telemetry_stride", stride, "n_samples", T, ...
    "note", "Full-resolution data is in result.mat; JSON may be decimated.");
telemetry.parameters = result.summary.parameters;
telemetry.validation = result.summary.validation;
telemetry.metrics = result.summary.metrics;
telemetry.times = result.times(idx)';
telemetry.centroid = result.centroid(idx, :);
telemetry.formation_error = result.formation_error(idx)';
telemetry.localization_error = result.localization_error(idx)';
telemetry.n_informed = result.n_informed(idx)';
telemetry.formation_error_per_robot = fePerRobot;
telemetry.positions = posOut;
telemetry.source = cfg.source;
end

function [idx, stride] = decimateIndices(nSamples, maxSamples)
if nSamples <= maxSamples
    idx = (1:nSamples)';
    stride = 1;
    return
end
stride = ceil(nSamples / maxSamples);
idx = (1:stride:nSamples)';
if idx(end) ~= nSamples
    idx = [idx; nSamples];
end
end
