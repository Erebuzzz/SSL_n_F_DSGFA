% Unicycle.m
% Self-contained MATLAB simulation of the Du et al. (2024) sign gradient-free
% controller on unicycle robots via point-offset feedback linearization:
%   s_i = p_i + r [cos theta_i; sin theta_i]
%   [v; omega] = [cos, sin; -sin/r, cos/r] * f_i
%   p_dot = v [cos theta; sin theta],  theta_dot = omega
%
% Metrics are computed on the control points s_i (the states the single-
% integrator law governs), matching Python Phase 2 / sgf_sim.unicycle.
%
% Run from anywhere (no addpath / no external helpers required):
%   cd sims
%   Unicycle
%
% Defaults match the working unicycle preset (alpha=10, r=2, boundary layer
% 0.2). Pure paper sgn (boundary layer 0) forms the formation but stalls
% localization outside the epsilon bound due to heading chatter.

%% Configuration

savePlots = true;
saveAnimation = true;    % writes motion.gif with heading arrows (~120 frames)
saveTelemetry = false;    % writes telemetry.json (time-series; decimated if very long)
telemetryMaxSamples = 10000;
animationFps = 20;

scriptDir = fileparts(mfilename("fullpath"));
runDir = fullfile(scriptDir, "outputs", "Unicycle");
if ~exist(runDir, "dir")
    mkdir(runDir);
end

%% Parameters
% Working unicycle gains (see matlab_turtlebot README / Python unicycle CLI).

n = 6;
source = [5.5, 5.5];
kappa = 1.0;
R = 2.0;
Dmax = 12.0;
alpha = 10.0;            % reduced vs paper SI alpha=100 (avoids huge omega)
beta = 0.05;
noiseModel = "bounded";
noiseStd = 0.2;
noiseBound = 0.2;
duration = 90.0;
dt = 0.004;
seed = 1;
offset = 2.0;            % feedback-linearization shift r > 0
signBoundaryLayer = 0.2; % critical: removes sgn chatter on unicycle
commandPeriod = 0.0;     % 0 = recompute every step; >0 = hold commands
maxLinearVelocity = [];  % [] = no limit
maxAngularVelocity = [];
topologyName = "paper_fig1_reconstructed";

% Alternative: exact paper sgn (localization stalls ~0.7–0.8 m from source).
% signBoundaryLayer = 0.0;

% Alternative: configs/unicycle_default.json (paper alpha, tiny r, hardware caps).
% These command large |omega| and starve localization under true TurtleBot limits.
% alpha = 100.0; offset = 0.025; duration = 60.0; dt = 0.001;
% maxLinearVelocity = 0.22; maxAngularVelocity = 2.84; signBoundaryLayer = 0.0;

% Optional sampled communication (hold period in seconds).
% commandPeriod = 0.1;

% Initial poses: layout from matlab_turtlebot/tb_config.m (differs from SI).
initialPositions = [
    0.0,  0.0;
    2.5, -0.5;
    5.0,  0.0;
    0.5,  3.5;
    3.0,  4.0;
    5.5,  3.0
] + (source - [5.5, 5.5]);
initialHeadings = zeros(n, 1);
informedMask = true(n, 1);

%% System Matrices

adjacency = zeros(n, n);
edges = [1 2; 1 3; 1 6; 2 3; 2 5; 3 4; 4 5; 5 6];
for e = 1:size(edges, 1)
    i = edges(e, 1);
    j = edges(e, 2);
    adjacency(i, j) = 1;
    adjacency(j, i) = 1;
end

thetaSlots = 2 * pi * (0:(n - 1))' / n;
phi = [cos(thetaSlots), sin(thetaSlots)];

%% Initialization
rng(seed, "twister");

steps = round(duration / dt);
times = (0:steps)' * dt;
positions = zeros(n, 2, steps + 1);
headings = zeros(n, steps + 1);
controlPts = zeros(n, 2, steps + 1);
centroid = zeros(steps + 1, 2);
formationError = zeros(steps + 1, 1);
localizationError = zeros(steps + 1, 1);
nInformed = zeros(steps + 1, 1);
commandedV = zeros(n, steps);
commandedOmega = zeros(n, steps);

positions(:, :, 1) = initialPositions;
headings(:, 1) = initialHeadings;
holdSteps = max(1, round(commandPeriod / dt));
v = zeros(n, 1);
omega = zeros(n, 1);

cfg = struct( ...
    "n", n, "source", source, "kappa", kappa, "R", R, "Dmax", Dmax, ...
    "alpha", alpha, "beta", beta, "noise_model", noiseModel, ...
    "noise_std", noiseStd, "noise_bound", noiseBound, ...
    "sign_boundary_layer", signBoundaryLayer, "informed_mask", informedMask, ...
    "topology_name", topologyName, "dt", dt, "duration", duration, "seed", seed, ...
    "offset", offset, "command_period", commandPeriod, ...
    "max_linear_velocity", maxLinearVelocity, ...
    "max_angular_velocity", maxAngularVelocity);

%% Controller / Simulation
% On each communication tick: measure(s) -> Eq.4 f -> (v, omega) -> Euler step.

for step = 1:(steps + 1)
    p = positions(:, :, step);
    theta = headings(:, step);
    s = p + offset * [cos(theta), sin(theta)];

    controlPts(:, :, step) = s;
    centroid(step, :) = mean(s, 1);
    formationError(step) = formationErrorValue(s, R, phi);
    localizationError(step) = norm(centroid(step, :) - source);

    if mod(step - 1, holdSteps) == 0
        [sigma, informed] = measureSource(s, cfg);
        nInformed(step) = sum(informed);
        f = eq4Control(s, adjacency, phi, sigma, cfg);
        [v, omega] = feedbackLinearize(f, theta, offset);
        [v, omega] = clipCommands(v, omega, maxLinearVelocity, maxAngularVelocity);
    else
        distances = sqrt(sum((s - source) .^ 2, 2));
        nInformed(step) = sum((distances < Dmax) & informedMask);
    end

    if step == steps + 1
        break
    end

    commandedV(:, step) = v;
    commandedOmega(:, step) = omega;
    positions(:, :, step + 1) = p + dt * [v .* cos(theta), v .* sin(theta)];
    headings(:, step + 1) = wrapAngle(theta + dt * omega);
end

if min(nInformed) == 0
    warning(["No robot is ever within Dmax of the source: the centroid cannot " ...
        "localize. Move the source closer, raise Dmax, or start nearer the source."]);
end

bounds = theoryBounds(cfg, min(nInformed));
summary = buildUnicycleSummary(cfg, formationError, localizationError, ...
    nInformed, commandedV, commandedOmega, bounds);

result = struct();
result.times = times;
result.positions = positions;
result.control_points = controlPts;
result.headings = headings;
result.centroid = centroid;
result.formation_error = formationError;
result.localization_error = localizationError;
result.n_informed = nInformed;
result.commanded_v = commandedV;
result.commanded_omega = commandedOmega;
result.summary = summary;

%% Visualization
if savePlots
    plotControlPointTrajectory(result, cfg, phi, runDir, "Unicycle");
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
        writeUnicycleAnimation(result, cfg, phi, epsilon, runDir, animationFps);
    end
end

save(fullfile(runDir, "result.mat"), "result", "cfg");
writeJson(fullfile(runDir, "summary.json"), summary);
if saveTelemetry
    telemetry = buildUnicycleTelemetry(result, cfg, phi, "unicycle", telemetryMaxSamples);
    writeJson(fullfile(runDir, "telemetry.json"), telemetry);
end

fprintf("Unicycle complete.\n");
fprintf("Output folder: %s\n", runDir);
fprintf("Final formation error:    %.5f\n", summary.metrics.final_formation_error);
fprintf("Final localization error: %.5f\n", summary.metrics.final_localization_error);
fprintf("Max |v|=%.3f m/s  Max |omega|=%.3f rad/s\n", ...
    summary.metrics.max_commanded_linear_velocity, ...
    summary.metrics.max_commanded_angular_velocity);
if summary.validation.bound_applicable && ~isempty(summary.validation.inside_bound)
    fprintf("Inside theorem bound: %d (epsilon=%.4f)\n", ...
        summary.validation.inside_bound, summary.validation.epsilon);
end

%% Helper Functions

function [sigma, informed] = measureSource(positions, cfg)
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
if epsBl > 0
    s = max(min(d / epsBl, 1.0), -1.0);
else
    s = sign(d);
end
end

function [v, omega] = feedbackLinearize(commands, headings, r)
if r <= 0
    error("control-point offset r must be positive (map is singular at r=0).");
end
c = cos(headings(:));
s = sin(headings(:));
fx = commands(:, 1);
fy = commands(:, 2);
v = c .* fx + s .* fy;
omega = (-s .* fx + c .* fy) / r;
end

function [v, omega] = clipCommands(v, omega, maxV, maxOmega)
if ~isempty(maxV)
    v = max(min(v, maxV), -maxV);
end
if ~isempty(maxOmega)
    omega = max(min(omega, maxOmega), -maxOmega);
end
end

function a = wrapAngle(a)
a = mod(a + pi, 2 * pi) - pi;
end

function value = formationErrorValue(s, R, phi)
z = s - R * phi;
zc = mean(z, 1);
value = sqrt(sum((z - zc) .^ 2, "all"));
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

function summary = buildUnicycleSummary(cfg, formationError, localizationError, ...
    nInformed, cmdV, cmdOmega, bounds)
finalFormation = formationError(end);
finalLocalization = localizationError(end);
insideBound = [];
if bounds.bound_applicable && ~isempty(bounds.epsilon)
    insideBound = finalLocalization <= bounds.epsilon;
end
tailStart = max(1, floor(0.75 * numel(localizationError)));
ft = formationError(tailStart:end);
lt = localizationError(tailStart:end);

summary = struct();
summary.model = "unicycle";
summary.parameters = struct( ...
    "n", cfg.n, "source", cfg.source, "kappa", cfg.kappa, "radius", cfg.R, ...
    "dmax", cfg.Dmax, "alpha", cfg.alpha, "beta", cfg.beta, ...
    "duration", cfg.duration, "dt", cfg.dt, "seed", cfg.seed, ...
    "noise_model", cfg.noise_model, "noise_bound", cfg.noise_bound, ...
    "topology", cfg.topology_name, "control_point_offset", cfg.offset, ...
    "command_period", cfg.command_period, ...
    "sign_boundary_layer", cfg.sign_boundary_layer);
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
    "tail_formation_error_span", max(ft) - min(ft), ...
    "tail_localization_error_span", max(lt) - min(lt), ...
    "max_commanded_linear_velocity", max(abs(cmdV), [], "all"), ...
    "max_commanded_angular_velocity", max(abs(cmdOmega), [], "all"));
end

function plotControlPointTrajectory(result, cfg, phi, runDir, label)
fig = figure("Visible", "off");
hold on
for i = 1:cfg.n
    xy = squeeze(result.control_points(i, :, :))';
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
title(sprintf("%s trajectories (%s)", label, cfg.topology_name));
saveas(fig, fullfile(runDir, "trajectory.png"));
close(fig);
end

function plotFormationErrorPerRobot(result, cfg, phi, runDir)
% Per-robot e_i = ||z_i - z*|| on control points (paper Fig. 3).
T = numel(result.times);
z = result.control_points - cfg.R * phi;
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
    legend({"error", "bound"}, "Location", "best");
end
grid on
xlabel("time [s]");
ylabel(ylab);
title(ttl);
saveas(fig, path);
close(fig);
end

function writeUnicycleAnimation(result, cfg, phi, epsilon, runDir, animationFps)
gifPath = fullfile(runDir, "motion.gif");
nFrames = numel(result.times);
stride = max(1, floor(nFrames / 120));
frames = 1:stride:nFrames;
histStride = max(1, floor(nFrames / 1200));
bgT = result.times(1:histStride:end);
bgFe = result.formation_error(1:histStride:end);
bgLe = result.localization_error(1:histStride:end);
meta = sprintf("Unicycle  topology=%s  alpha/beta=%g  T=%.3gs", ...
    cfg.topology_name, cfg.alpha / cfg.beta, cfg.command_period);
allXy = reshape(permute(result.control_points, [3 1 2]), [], 2);
pad = 1.0;
xl = [min([allXy(:, 1); cfg.source(1)]) - pad, max([allXy(:, 1); cfg.source(1)]) + pad];
yl = [min([allXy(:, 2); cfg.source(2)]) - pad, max([allXy(:, 2); cfg.source(2)]) + pad];
fig = figure("Visible", "off", "Position", [100, 100, 1000, 500]);
first = true;
for idx = frames
    clf(fig);
    subplot(1, 2, 1);
    hold on
    hist = unique([1:histStride:idx, idx]);
    for i = 1:cfg.n
        xi = reshape(result.control_points(i, 1, hist), [], 1);
        yi = reshape(result.control_points(i, 2, hist), [], 1);
        plot(xi, yi, "LineWidth", 0.7);
    end
    s = result.control_points(:, :, idx);
    th = result.headings(:, idx);
    quiver(s(:, 1), s(:, 2), cos(th), sin(th), 0.4, "b");
    plot(s(:, 1), s(:, 2), "bo", "MarkerFaceColor", "b", "MarkerSize", 5);
    c = result.centroid(idx, :);
    circle = c + cfg.R * phi;
    circle = [circle; circle(1, :)];
    plot(circle(:, 1), circle(:, 2), "k:", "LineWidth", 1.2);
    plot(c(1), c(2), "kx", "MarkerSize", 10, "LineWidth", 1.5);
    plot(cfg.source(1), cfg.source(2), "rp", "MarkerSize", 14, "MarkerFaceColor", "r");
    xlim(xl); ylim(yl);
    axis equal
    grid on
    xlabel("x [m]"); ylabel("y [m]");
    title(sprintf("Unicycles and source (t = %.2f s)", result.times(idx)));
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
    [im, cmap] = rgb2ind(frame2im(frame), 256);
    if first
        imwrite(im, cmap, gifPath, "gif", "LoopCount", Inf, "DelayTime", 1 / animationFps);
        first = false;
    else
        imwrite(im, cmap, gifPath, "gif", "WriteMode", "append", "DelayTime", 1 / animationFps);
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

function telemetry = buildUnicycleTelemetry(result, cfg, phi, modelName, maxSamples)
[idx, stride] = decimateIndices(numel(result.times), maxSamples);
T = numel(idx);
n = cfg.n;
nCmd = size(result.commanded_v, 2);

fePerRobot = zeros(n, T);
posOut = zeros(T, n, 2);
cpOut = zeros(T, n, 2);
headOut = zeros(T, n);
vOut = nan(n, T);
omegaOut = nan(n, T);

for k = 1:T
    t = idx(k);
    p = result.positions(:, :, t);
    theta = result.headings(:, t);
    s = result.control_points(:, :, t);
    posOut(k, :, :) = p;
    cpOut(k, :, :) = s;
    headOut(k, :) = theta';
    z = s - cfg.R * phi;
    zc = mean(z, 1);
    fePerRobot(:, k) = sqrt(sum((z - zc) .^ 2, 2));
    if t <= nCmd
        vOut(:, k) = result.commanded_v(:, t);
        omegaOut(:, k) = result.commanded_omega(:, t);
    end
end

telemetry = struct();
telemetry.meta = struct( ...
    "model", modelName, ...
    "generated_at", char(datetime("now", "TimeZone", "local", "Format", "yyyy-MM-dd'T'HH:mm:ss")), ...
    "dt", cfg.dt, "duration", cfg.duration, "n_robots", n, ...
    "seed", cfg.seed, "topology", char(cfg.topology_name), ...
    "control_point_offset", cfg.offset, "command_period", cfg.command_period, ...
    "sign_boundary_layer", cfg.sign_boundary_layer, ...
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
telemetry.control_points = cpOut;
telemetry.headings = headOut;
telemetry.commanded_v = vOut;
telemetry.commanded_omega = omegaOut;
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
