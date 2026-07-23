% TurtleBot.m
% Self-contained MATLAB / Simulink simulation of the Du et al. (2024) sign
% gradient-free controller on TurtleBot-style differential-drive robots.
%
% Two execution paths (set runMode below):
%   "numeric"  — explicit Euler, sampled commands, optional actuator limits
%   "simulink" — programmatically built .slx (Integrator + MATLAB Function ODE),
%                fixed-step ode4; requires Simulink + Stateflow (MATLAB Function)
%
% Run from anywhere (no addpath / no external helpers required):
%   cd sims
%   TurtleBot
%
% Defaults: numeric path with matlab_turtlebot/configs/turtlebot_working.json
% parameters. Switch runMode to "simulink" for the noise-free continuous reference
% (matches turtlebot_simulink_default.json). See sims/RUN_GUIDE.md.

%% Configuration

runMode = "numeric";     % "numeric" | "simulink"
savePlots = true;
saveAnimation = true;    % writes motion.gif (~120 frames)
saveTelemetry = false;    % writes telemetry.json (time-series; decimated if very long)
telemetryMaxSamples = 10000;
animationFps = 20;

scriptDir = fileparts(mfilename("fullpath"));
runDir = fullfile(scriptDir, "outputs", "TurtleBot");
if ~exist(runDir, "dir")
    mkdir(runDir);
end

%% Parameters
% Shared paper / formation parameters. Noise defaults depend on runMode.

n = 6;
source = [5.5, 5.5];
kappa = 1.0;
R = 2.0;
Dmax = 12.0;
alpha = 10.0;
beta = 0.05;
duration = 90.0;
dt = 0.004;
seed = 1;
offset = 2.0;            % control-point shift r (feedback linearization)
signBoundaryLayer = 0.2; % required for localization on differential drive
commandPeriod = 0.0;     % numeric only; Simulink path is continuous
maxLinearVelocity = [];  % [] = no limit (demo); see hardware note below
maxAngularVelocity = [];
topologyName = "paper_fig1_reconstructed";

% TurtleBot3 Burger wheel model (used for wheel-speed reporting / kinematics).
wheelRadius = 0.033;     % m
wheelSeparation = 0.16;  % m

% Simulink options (used when runMode = "simulink").
simulinkSolver = "ode4";
simulinkModelName = "sgf_turtlebot_swarm";

if runMode == "simulink"
    % Continuous ODE + stochastic measurement is ill-posed; use noise-free.
    noiseModel = "none";
    noiseStd = 0.0;
    noiseBound = 0.0;
else
    noiseModel = "bounded";
    noiseStd = 0.2;
    noiseBound = 0.2;
end

% Alternative: exact paper sgn (formation OK, localization outside epsilon).
% signBoundaryLayer = 0.0;

% Alternative: hardware-faithful TurtleBot3 Burger actuator limits / sample rate.
% These starve the slow localization term at paper-scale control-point speeds.
% maxLinearVelocity = 0.22;   % m/s
% maxAngularVelocity = 2.84;  % rad/s
% commandPeriod = 0.1;        % s (numeric path only)

% Initial poses from matlab_turtlebot/tb_config.m.
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

cfg = struct( ...
    "n", n, "source", source, "kappa", kappa, "R", R, "Dmax", Dmax, ...
    "alpha", alpha, "beta", beta, "noise_model", noiseModel, ...
    "noise_std", noiseStd, "noise_bound", noiseBound, ...
    "sign_boundary_layer", signBoundaryLayer, "informed_mask", informedMask, ...
    "topology_name", topologyName, "dt", dt, "duration", duration, "seed", seed, ...
    "offset", offset, "command_period", commandPeriod, ...
    "wheel_radius", wheelRadius, "wheel_separation", wheelSeparation, ...
    "max_linear_velocity", maxLinearVelocity, ...
    "max_angular_velocity", maxAngularVelocity, ...
    "initial_positions", initialPositions, ...
    "initial_headings", initialHeadings, ...
    "simulink_solver", simulinkSolver, ...
    "simulink_model_name", simulinkModelName, ...
    "adjacency", adjacency, "run_dir", runDir);

%% Simulation
% Branch on runMode. Both paths produce the same result/summary field layout.

if runMode == "simulink"
    result = runSimulinkPath(cfg, adjacency, phi);
else
    result = runNumericPath(cfg, adjacency, phi);
end

summary = result.summary;

%% Visualization
if savePlots
    plotControlPointTrajectory(result, cfg, phi, runDir);
    plotSeries(result.times, result.formation_error, "Formation error", ...
        "formation error", [], fullfile(runDir, "formation_error.png"));
    plotFormationErrorPerRobot(result, cfg, phi, runDir);
    epsilon = [];
    if summary.validation.bound_applicable && ~isempty(summary.validation.epsilon)
        epsilon = summary.validation.epsilon;
    end
    plotSeries(result.times, result.localization_error, "Localization error", ...
        "||centroid - source||", epsilon, fullfile(runDir, "localization_error.png"));
    if saveAnimation
        writeTurtlebotAnimation(result, cfg, phi, epsilon, runDir, animationFps);
    end
end

save(fullfile(runDir, "result.mat"), "result", "cfg");
writeJson(fullfile(runDir, "summary.json"), summary);
if saveTelemetry
    telemetry = buildTbTelemetry(result, cfg, phi, runMode, telemetryMaxSamples);
    writeJson(fullfile(runDir, "telemetry.json"), telemetry);
end

fprintf("TurtleBot (%s) complete.\n", runMode);
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

%% Helper Functions — Numeric path

function result = runNumericPath(cfg, adjacency, phi)
rng(cfg.seed, "twister");
params = struct("wheel_radius", cfg.wheel_radius, "wheel_separation", cfg.wheel_separation);
n = cfg.n;
steps = round(cfg.duration / cfg.dt);
times = (0:steps)' * cfg.dt;

positions = zeros(n, 2, steps + 1);
headings = zeros(n, steps + 1);
controlPts = zeros(n, 2, steps + 1);
centroid = zeros(steps + 1, 2);
formationError = zeros(steps + 1, 1);
localizationError = zeros(steps + 1, 1);
nInformed = zeros(steps + 1, 1);
commandedV = zeros(n, steps);
commandedOmega = zeros(n, steps);

positions(:, :, 1) = cfg.initial_positions;
headings(:, 1) = cfg.initial_headings;
holdSteps = max(1, round(cfg.command_period / cfg.dt));
v = zeros(n, 1);
omega = zeros(n, 1);

for step = 1:(steps + 1)
    p = positions(:, :, step);
    theta = headings(:, step);
    s = p + cfg.offset * [cos(theta), sin(theta)];

    controlPts(:, :, step) = s;
    centroid(step, :) = mean(s, 1);
    formationError(step) = formationErrorValue(s, cfg.R, phi);
    localizationError(step) = norm(centroid(step, :) - cfg.source);

    if mod(step - 1, holdSteps) == 0
        [sigma, informed] = measureSource(s, cfg);
        nInformed(step) = sum(informed);
        f = eq4Control(s, adjacency, phi, sigma, cfg);
        [v, omega] = feedbackLinearize(f, theta, cfg.offset);
        [v, omega] = clipCommands(v, omega, cfg.max_linear_velocity, cfg.max_angular_velocity);
    else
        distances = sqrt(sum((s - cfg.source) .^ 2, 2));
        nInformed(step) = sum((distances < cfg.Dmax) & cfg.informed_mask);
    end

    if step == steps + 1
        break
    end

    commandedV(:, step) = v;
    commandedOmega(:, step) = omega;
    [pNext, thetaNext] = differentialDriveStep(p, theta, v, omega, cfg.dt, params);
    positions(:, :, step + 1) = pNext;
    headings(:, step + 1) = thetaNext;
end

if min(nInformed) == 0
    warning(["No robot is ever within Dmax of the source: the centroid cannot " ...
        "localize. Move the source closer, raise Dmax, or start nearer the source."]);
end

bounds = theoryBounds(cfg, min(nInformed));
summary = buildTbSummary(cfg, formationError, localizationError, nInformed, ...
    commandedV, commandedOmega, bounds, "turtlebot");

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
end

function [posNext, thetaNext, wheelSpeeds] = differentialDriveStep(pos, theta, v, omega, dt, params)
% Unicycle kinematics + left/right wheel angular speeds (for inspection).
halfBase = 0.5 * params.wheel_separation;
right = (v(:) + omega(:) * halfBase) / params.wheel_radius;
left = (v(:) - omega(:) * halfBase) / params.wheel_radius;
wheelSpeeds = [left, right];
posNext = pos + dt * [v(:) .* cos(theta(:)), v(:) .* sin(theta(:))];
thetaNext = wrapToPiLocal(theta(:) + dt * omega(:));
end

%% Helper Functions — Simulink path

function result = runSimulinkPath(cfg, adjacency, phi)
if ~exist(cfg.run_dir, "dir")
    mkdir(cfg.run_dir);
end
modelDir = fullfile(cfg.run_dir, "models");
[modelPath, modelName] = buildSimulinkModel(cfg, adjacency, phi, modelDir);
cleanup = onCleanup(@() closeIfLoaded(modelName)); %#ok<NASGU>

simout = sim(modelName, "StopTime", num2str(cfg.duration, "%.10g"));
xout = simout.get("xout");
tout = simout.get("tout");

n = cfg.n;
X = reshape(xout, 3 * n, []);
T = size(X, 2);
positions = zeros(n, 2, T);
positions(:, 1, :) = reshape(X(1:n, :), [n, 1, T]);
positions(:, 2, :) = reshape(X(n + 1:2 * n, :), [n, 1, T]);
headings = X(2 * n + 1:3 * n, :);
times = tout(:);

result = resultFromTrajectory(cfg, adjacency, phi, positions, headings, times, ...
    "turtlebot_simulink");
result.model_path = modelPath;
end

function closeIfLoaded(modelName)
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end

function [modelPath, modelName] = buildSimulinkModel(cfg, adjacency, phi, outDir)
% Programmatic .slx: Integrator (3n) <-> MATLAB Function swarm_ode + logging.
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

if cfg.noise_model == "gaussian"
    noiseVar = cfg.noise_std ^ 2;
else
    noiseVar = 0.0;
end

modelName = char(cfg.simulink_model_name);
solver = char(cfg.simulink_solver);
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

x0 = [cfg.initial_positions(:, 1); cfg.initial_positions(:, 2); cfg.initial_headings];
code = generateOdeCode(cfg, adjacency, phi);

new_system(modelName);
load_system(modelName);

add_block('simulink/Continuous/Integrator', [modelName '/state']);
set_param([modelName '/state'], 'InitialCondition', mat2str(x0(:)), ...
    'Position', [300, 100, 340, 140]);

add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/swarm_ode'], ...
    'Position', [120, 95, 240, 175]);
setMatlabFunctionCode(modelName, 'swarm_ode', code);

add_block('simulink/Sources/Random Number', [modelName '/meas_noise'], ...
    'Mean', mat2str(zeros(cfg.n, 1)), ...
    'Variance', mat2str(noiseVar * ones(cfg.n, 1)), ...
    'Seed', mat2str((cfg.seed + (0:(cfg.n - 1)))'), ...
    'SampleTime', num2str(cfg.dt, '%.10g'), ...
    'Position', [-40, 180, 20, 220]);

add_block('simulink/Sinks/To Workspace', [modelName '/xout'], ...
    'VariableName', 'xout', 'SaveFormat', 'Array', 'SampleTime', '-1', ...
    'Position', [430, 100, 490, 140]);
add_block('simulink/Sources/Clock', [modelName '/clock'], ...
    'Position', [430, 200, 460, 230]);
add_block('simulink/Sinks/To Workspace', [modelName '/tout'], ...
    'VariableName', 'tout', 'SaveFormat', 'Array', 'SampleTime', '-1', ...
    'Position', [510, 195, 570, 235]);

add_line(modelName, 'swarm_ode/1', 'state/1', 'autorouting', 'on');
add_line(modelName, 'state/1', 'swarm_ode/1', 'autorouting', 'on');
add_line(modelName, 'meas_noise/1', 'swarm_ode/2', 'autorouting', 'on');
add_line(modelName, 'state/1', 'xout/1', 'autorouting', 'on');
add_line(modelName, 'clock/1', 'tout/1', 'autorouting', 'on');

set_param(modelName, 'SolverType', 'Fixed-step', 'Solver', solver, ...
    'FixedStep', num2str(cfg.dt, '%.10g'), ...
    'StopTime', num2str(cfg.duration, '%.10g'), ...
    'SaveOutput', 'off', 'SaveTime', 'off');

modelPath = fullfile(outDir, [modelName '.slx']);
save_system(modelName, modelPath);
end

function setMatlabFunctionCode(modelName, blockName, code)
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', [modelName '/' blockName]);
if isempty(chart)
    error('Could not locate MATLAB Function block %s/%s.', modelName, blockName);
end
chart.Script = code;
end

function code = generateOdeCode(cfg, adjacency, phi)
% Bake all parameters into the MATLAB Function body (codegen-safe, self-contained).
n = cfg.n;
adjLit = mat2str(double(adjacency ~= 0));
phiLit = mat2str(phi, 15);
srcLit = mat2str(cfg.source(:)', 15);

lines = {
    'function dx = swarm_ode(x, meas_noise)'
    '%#codegen'
    '% Auto-generated by TurtleBot.m. Do not edit the .slx by hand.'
    sprintf('n = %d;', n)
    sprintf('r = %.15g;', cfg.offset)
    sprintf('R = %.15g;', cfg.R)
    sprintf('kappa = %.15g;', cfg.kappa)
    sprintf('Dmax = %.15g;', cfg.Dmax)
    sprintf('alpha = %.15g;', cfg.alpha)
    sprintf('beta = %.15g;', cfg.beta)
    sprintf('noise_bound = %.15g;', cfg.noise_bound)
    sprintf('eps_bl = %.15g;', cfg.sign_boundary_layer)
    sprintf('src = %s;', srcLit)
    sprintf('adj = %s;', adjLit)
    sprintf('phi = %s;', phiLit)
    'px = x(1:n);'
    'py = x(n+1:2*n);'
    'th = x(2*n+1:3*n);'
    'sx = px + r .* cos(th);'
    'sy = py + r .* sin(th);'
    'zx = sx - R .* phi(:,1);'
    'zy = sy - R .* phi(:,2);'
    'dx = zeros(3*n, 1);'
    'for i = 1:n'
    '    fx = 0.0;'
    '    fy = 0.0;'
    '    for j = 1:n'
    '        if adj(i,j) ~= 0'
    '            fx = fx + alpha * ssign(zx(j) - zx(i), eps_bl);'
    '            fy = fy + alpha * ssign(zy(j) - zy(i), eps_bl);'
    '        end'
    '    end'
    '    dsx = sx(i) - src(1);'
    '    dsy = sy(i) - src(2);'
    '    dist = sqrt(dsx*dsx + dsy*dsy);'
    '    if dist < Dmax'
    '        sigma = kappa * (dsx*dsx + dsy*dsy) + meas_noise(i);'
    '    else'
    '        sigma = kappa * Dmax * Dmax + noise_bound;'
    '    end'
    '    fx = fx - (2*beta/R) * sigma * phi(i,1);'
    '    fy = fy - (2*beta/R) * sigma * phi(i,2);'
    '    c = cos(th(i));'
    '    s = sin(th(i));'
    '    v = c*fx + s*fy;'
    '    om = (-s*fx + c*fy) / r;'
    '    dx(i)       = v * c;'
    '    dx(n+i)     = v * s;'
    '    dx(2*n+i)   = om;'
    'end'
    'end'
    ''
    'function y = ssign(d, eps)'
    'if eps > 0'
    '    y = max(min(d / eps, 1.0), -1.0);'
    'else'
    '    y = sign(d);'
    'end'
    'end'
    };

code = strjoin(lines, newline);
end

function result = resultFromTrajectory(cfg, adjacency, phi, positions, headings, times, modelLabel)
% Replay logged poses through the same helpers to recover metrics / commands.
n = cfg.n;
T = numel(times);
controlPts = zeros(n, 2, T);
centroid = zeros(T, 2);
formationError = zeros(T, 1);
localizationError = zeros(T, 1);
nInformed = zeros(T, 1);
commandedV = zeros(n, T);
commandedOmega = zeros(n, T);

for step = 1:T
    p = positions(:, :, step);
    theta = headings(:, step);
    s = p + cfg.offset * [cos(theta), sin(theta)];
    controlPts(:, :, step) = s;
    centroid(step, :) = mean(s, 1);
    formationError(step) = formationErrorValue(s, cfg.R, phi);
    localizationError(step) = norm(centroid(step, :) - cfg.source);
    [sigma, informed] = measureSource(s, cfg);
    nInformed(step) = sum(informed);
    f = eq4Control(s, adjacency, phi, sigma, cfg);
    [v, omega] = feedbackLinearize(f, theta, cfg.offset);
    [v, omega] = clipCommands(v, omega, cfg.max_linear_velocity, cfg.max_angular_velocity);
    commandedV(:, step) = v;
    commandedOmega(:, step) = omega;
end

bounds = theoryBounds(cfg, min(nInformed));
summary = buildTbSummary(cfg, formationError, localizationError, nInformed, ...
    commandedV, commandedOmega, bounds, modelLabel);

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
end

%% Helper Functions — Shared control / measurement / theory

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

function a = wrapToPiLocal(a)
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

function summary = buildTbSummary(cfg, formationError, localizationError, ...
    nInformed, cmdV, cmdOmega, bounds, modelLabel)
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
summary.model = modelLabel;
summary.parameters = struct( ...
    "n", cfg.n, "source", cfg.source, "kappa", cfg.kappa, "radius", cfg.R, ...
    "dmax", cfg.Dmax, "alpha", cfg.alpha, "beta", cfg.beta, ...
    "duration", cfg.duration, "dt", cfg.dt, "seed", cfg.seed, ...
    "noise_model", cfg.noise_model, "noise_bound", cfg.noise_bound, ...
    "topology", cfg.topology_name, "control_point_offset", cfg.offset, ...
    "command_period", cfg.command_period, "wheel_radius", cfg.wheel_radius, ...
    "wheel_separation", cfg.wheel_separation, ...
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

%% Helper Functions — Plots / I/O

function plotControlPointTrajectory(result, cfg, phi, runDir)
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
title(sprintf("TurtleBot trajectories (%s)", cfg.topology_name));
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

function writeTurtlebotAnimation(result, cfg, phi, epsilon, runDir, animationFps)
gifPath = fullfile(runDir, "motion.gif");
nFrames = numel(result.times);
stride = max(1, floor(nFrames / 120));
frames = 1:stride:nFrames;
histStride = max(1, floor(nFrames / 1200));
bgT = result.times(1:histStride:end);
bgFe = result.formation_error(1:histStride:end);
bgLe = result.localization_error(1:histStride:end);
meta = sprintf("TurtleBot  topology=%s  alpha/beta=%g  T=%.3gs", ...
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
    title(sprintf("TurtleBots and source (t = %.2f s)", result.times(idx)));
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

function telemetry = buildTbTelemetry(result, cfg, phi, runMode, maxSamples)
% Per-step telemetry for TurtleBot (numeric or Simulink); same layout as Unicycle.
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

modelLabel = char(result.summary.model);
if isempty(modelLabel)
    modelLabel = char(runMode);
end

telemetry = struct();
telemetry.meta = struct( ...
    "model", modelLabel, ...
    "run_mode", char(runMode), ...
    "generated_at", char(datetime("now", "TimeZone", "local", "Format", "yyyy-MM-dd'T'HH:mm:ss")), ...
    "dt", cfg.dt, "duration", cfg.duration, "n_robots", n, ...
    "seed", cfg.seed, "topology", char(cfg.topology_name), ...
    "control_point_offset", cfg.offset, "command_period", cfg.command_period, ...
    "wheel_radius", cfg.wheel_radius, "wheel_separation", cfg.wheel_separation, ...
    "sign_boundary_layer", cfg.sign_boundary_layer, ...
    "telemetry_stride", stride, "n_samples", T, ...
    "note", "Full-resolution data is in result.mat; JSON may be decimated.");
if isfield(result, "model_path") && ~isempty(result.model_path)
    telemetry.meta.simulink_model_path = char(result.model_path);
end
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
