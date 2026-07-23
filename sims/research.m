% research.m
% Research-gap validation harness for extensions to Du et al. (2024).
%
% Validates three open directions (see docs/RESEARCH_GAPS_THEORY.md):
%   Gap 1 — N teams for N source minima (unified start, then split)
%   Gap 2 — Moving / non-stationary source p_s(t)
%   Gap 3 — Non-convex field; global maximum vs local traps
%
% Run from sims/:
%   research                    % all gaps, default robot model
%   research gap1               % all Gap 1 configs (n8_N2, n9_N3, n12_N3)
%   research gap1 n9_N3         % single Gap 1 config
%   research gap2 unicycle      % Gap 2 on unicycle model
%   research gap3               % Gap 3 single integrator
%
% Robot model (edit below or pass as 2nd arg: single_integrator | unicycle):
%   single_integrator — point robots, paper-like gains
%   unicycle — feedback-linearized control points (Unicycle.m preset)
%
% Gap 1 outputs per config: sims/outputs/research/gap1/<config_name>/
%   trajectory.png, team_localization.png, phase_timeline.png,
%   deployment_map.png, motion.gif, summary.json, result.mat

function research(varargin)
scriptDir = fileparts(mfilename("fullpath"));
runRoot = fullfile(scriptDir, "outputs", "research");
if ~exist(runRoot, "dir")
    mkdir(runRoot);
end

robotModel = "single_integrator";  % "single_integrator" | "unicycle"
savePlots = true;
saveAnimation = true;
saveTelemetry = false;
telemetryMaxSamples = 10000;
animationFps = 20;

gapsToRun = ["gap1", "gap2", "gap3"];
gap1Config = "";  % empty = all Gap 1 validation configs

if nargin >= 1 && ~isempty(varargin{1})
    arg1 = lower(string(varargin{1}));
    if startsWith(arg1, "gap")
        gapsToRun = arg1;
    elseif arg1 == "single_integrator" || arg1 == "unicycle"
        robotModel = arg1;
    else
        error("Unknown argument: %s", arg1);
    end
end
if nargin >= 2 && ~isempty(varargin{2})
    arg2 = lower(string(varargin{2}));
    if startsWith(arg2, "gap")
        gapsToRun = arg2;
    elseif arg2 == "single_integrator" || arg2 == "unicycle"
        robotModel = arg2;
    elseif startsWith(gapsToRun, "gap1")
        gap1Config = arg2;
    else
        error("Unknown argument: %s", arg2);
    end
end
if nargin >= 3 && ~isempty(varargin{3})
    arg3 = lower(string(varargin{3}));
    if arg3 == "single_integrator" || arg3 == "unicycle"
        robotModel = arg3;
    elseif startsWith(gapsToRun, "gap1")
        gap1Config = arg3;
    end
end

simOpts = buildSimOpts(robotModel);
outputOpts = struct( ...
    "save_plots", savePlots, ...
    "save_animation", saveAnimation, ...
    "save_telemetry", saveTelemetry, ...
    "telemetry_max_samples", telemetryMaxSamples, ...
    "animation_fps", animationFps, ...
    "robot_model", robotModel);

fprintf("research.m — gaps: %s | robot: %s\n", strjoin(gapsToRun, ", "), robotModel);
fprintf("Theory reference: docs/RESEARCH_GAPS_THEORY.md\n\n");

allSummaries = struct();

for g = 1:numel(gapsToRun)
    gapName = gapsToRun(g);
    switch gapName
        case "gap1"
            allSummaries.gap1 = runGap1MultiSource(runRoot, outputOpts, simOpts, gap1Config);
        case "gap2"
            allSummaries.gap2 = runGap2MovingSource(runRoot, outputOpts, simOpts);
        case "gap3"
            allSummaries.gap3 = runGap3GlobalMaximum(runRoot, outputOpts, simOpts);
        otherwise
            error("Unknown gap: %s", gapName);
    end
end

writeJson(fullfile(runRoot, "research_summary.json"), allSummaries);
fprintf("\nAll requested gaps complete. Summary: %s\n", fullfile(runRoot, "research_summary.json"));
end

%% Gap 1 — Multi-source teams (unified start, then split)
function summaries = runGap1MultiSource(runRoot, outputOpts, simOpts, configFilter)
gapRoot = fullfile(runRoot, "gap1");
if ~exist(gapRoot, "dir")
    mkdir(gapRoot);
end

configs = gap1ValidationConfigs();
if strlength(configFilter) > 0
    names = strings(numel(configs), 1);
    for i = 1:numel(configs)
        names(i) = configs(i).name;
    end
    idx = find(lower(names) == lower(configFilter), 1);
    if isempty(idx)
        error("Unknown Gap 1 config '%s'. Options: %s", configFilter, strjoin(names, ", "));
    end
    configs = configs(idx);
end

fprintf("=== Gap 1: N teams for N source minima (%d config(s)) ===\n", numel(configs));
summaries = struct();

for c = 1:numel(configs)
    cfgDef = configs(c);
    cfgDir = fullfile(gapRoot, cfgDef.name);
    if ~exist(cfgDir, "dir")
        mkdir(cfgDir);
    end
    fprintf("  Config %s: n=%d, N=%d sources, split at t=%.1f s\n", ...
        cfgDef.name, cfgDef.n, size(cfgDef.sources, 1), cfgDef.split_time);

    result = simulateGap1TwoPhase(cfgDef, simOpts);
    saveGap1Outputs(result, cfgDir, outputOpts);
    save(fullfile(cfgDir, "result.mat"), "result", "cfgDef");
    writeJson(fullfile(cfgDir, "summary.json"), result.summary);

    summaries.(matlab.lang.makeValidName(cfgDef.name)) = result.summary;
    fprintf("    Team errors: [%s] | inside bounds: %d\n", ...
        num2str(result.summary.metrics.final_team_localization_errors, "%.4f "), ...
        result.summary.validation.all_teams_inside_bound);
    fprintf("    Output: %s\n", cfgDir);
end
writeJson(fullfile(gapRoot, "gap1_summary.json"), summaries);
fprintf("\n");
end

function result = simulateGap1TwoPhase(cfgDef, simOpts)
n = cfgDef.n;
sources = cfgDef.sources;
Nsrc = size(sources, 1);
splitTime = cfgDef.split_time;
splitStep = max(2, round(splitTime / simOpts.dt) + 1);

rng(cfgDef.seed, "twister");
initialPositions = clusterInitialPositions(n, cfgDef.cluster_center, cfgDef.cluster_spread);
initialHeadings = zeros(n, 1);

kappa = 1.0;
R = 2.0;
Dmax = 12.0;
noiseModel = "bounded";
noiseBound = 0.2;
duration = cfgDef.duration;

adjUnified = ringAdjacency(n);
phiUnified = globalFormationSlots(n);
virtualSource = mean(sources, 1);

cfg = struct( ...
    "n", n, "sources", sources, "virtual_source", virtualSource, ...
    "kappa", kappa, "R", R, "Dmax", Dmax, ...
    "alpha", simOpts.alpha, "beta", simOpts.beta, ...
    "noise_model", noiseModel, "noise_bound", noiseBound, ...
    "sign_boundary_layer", simOpts.sign_boundary_layer, ...
    "dt", simOpts.dt, "duration", duration, "seed", cfgDef.seed, ...
    "split_time", splitTime, "config_name", cfgDef.name, ...
    "robot_model", simOpts.model, "offset", simOpts.offset, ...
    "gap", "gap1_multi_source");

steps = round(duration / simOpts.dt);
times = (0:steps)' * simOpts.dt;
positions = zeros(n, 2, steps + 1);
headings = zeros(n, steps + 1);
controlPts = zeros(n, 2, steps + 1);
centroid = zeros(steps + 1, 2);
centroids = zeros(Nsrc, 2, steps + 1);
teamLocError = zeros(steps + 1, Nsrc);
formationError = zeros(steps + 1, 1);
teamSeparation = zeros(steps + 1, 1);
phaseTag = zeros(steps + 1, 1);  % 0 = unified, 1 = split
assignment = ones(n, 1);
assignmentTrace = zeros(n, steps + 1);

positions(:, :, 1) = initialPositions;
headings(:, 1) = initialHeadings;
adjacency = adjUnified;
phi = phiUnified;
splitDone = false;
splitStepActual = splitStep;

for step = 1:(steps + 1)
    t = times(step);
    p = positions(:, :, step);
    theta = headings(:, step);
    [statePos, ~] = controlStates(p, theta, simOpts);
    controlPts(:, :, step) = statePos;

    if ~splitDone && step >= splitStep
        assignment = balancedSourceAssign(statePos, sources);
        adjacency = teamAdjacency(n, assignment);
        phi = teamFormationSlots(n, assignment);
        splitDone = true;
        splitStepActual = step;
        cfg.assignment = assignment;
    end
    assignmentTrace(:, step) = assignment;
    phaseTag(step) = double(splitDone);

    if splitDone
        cfgStep = cfg;
        cfgStep.assignment = assignment;
        [sigma, ~] = measureMultiSource(statePos, cfgStep);
    else
        cfgStep = cfg;
        cfgStep.source = virtualSource;
        [sigma, ~] = measureSingleSource(statePos, cfgStep);
    end

    centroid(step, :) = mean(statePos, 1);
    formationError(step) = formationErrorValue(statePos, R, phi);

    teamIds = 1:Nsrc;
    if splitDone
        for k = 1:Nsrc
            members = find(assignment == teamIds(k));
            centroids(k, :, step) = mean(statePos(members, :), 1);
            teamLocError(step, k) = norm(centroids(k, :, step) - sources(k, :));
        end
        if Nsrc >= 2
            teamSeparation(step) = min(pairwiseDistances(squeeze(centroids(:, :, step))));
        end
    else
        for k = 1:Nsrc
            teamLocError(step, k) = norm(centroid(step, :) - sources(k, :));
            centroids(k, :, step) = centroid(step, :);
        end
        teamSeparation(step) = 0;
    end

    if step == steps + 1
        break
    end

    if splitDone
        u = eq4MultiSource(statePos, adjacency, phi, sigma, cfgStep);
    else
        u = eq4SingleSource(statePos, adjacency, phi, sigma, cfgStep);
    end
    [positions(:, :, step + 1), headings(:, step + 1)] = ...
        advanceDynamics(p, theta, u, simOpts);
end

teamSizes = zeros(Nsrc, 1);
for k = 1:Nsrc
    teamSizes(k) = sum(assignment == k);
end
bounds = teamTheoryBounds(cfg, teamSizes, noiseBound, teamLocError(end, :));

summary = struct();
summary.gap = "gap1_multi_source";
summary.description = "Unified swarm start, split into balanced teams at split_time";
summary.parameters = struct( ...
    "config_name", cfgDef.name, "n", n, "N_sources", Nsrc, "sources", sources, ...
    "cluster_center", cfgDef.cluster_center, "split_time", splitTime, ...
    "split_step", splitStepActual, "team_sizes", teamSizes, ...
    "final_assignment", assignment, "robot_model", simOpts.model, ...
    "alpha", simOpts.alpha, "beta", simOpts.beta, "duration", duration, "seed", cfgDef.seed);
summary.metrics = struct( ...
    "final_formation_error", formationError(end), ...
    "final_team_localization_errors", teamLocError(end, :), ...
    "mean_final_team_error", mean(teamLocError(end, :)), ...
    "max_final_team_error", max(teamLocError(end, :)), ...
    "final_team_separation", teamSeparation(end), ...
    "split_time_actual", times(splitStepActual));
summary.validation = struct( ...
    "per_team_epsilon", {bounds.epsilon}, ...
    "per_team_inside_bound", bounds.inside_bound, ...
    "all_teams_inside_bound", all(bounds.inside_bound), ...
    "gain_ratio", simOpts.alpha / simOpts.beta, ...
    "gain_threshold", bounds.gain_threshold);

cfg.assignment = assignment;
cfg.split_step = splitStepActual;
result = struct( ...
    "times", times, "positions", positions, "headings", headings, ...
    "control_points", controlPts, "centroid", centroid, "centroids", centroids, ...
    "team_loc_error", teamLocError, "team_separation", teamSeparation, ...
    "formation_error", formationError, "phase_tag", phaseTag, ...
    "assignment_trace", assignmentTrace, "phi", phi, "cfg", cfg, ...
    "summary", summary);
end

function configs = gap1ValidationConfigs()
% Three validation cases: equal team sizes, clustered unified start.
configs(1) = struct( ...
    "name", "n8_N2", "n", 8, "seed", 42, "duration", 70.0, ...
    "split_time", 12.0, ...
    "sources", [0.0, 10.0; 14.0, 10.0], ...
    "cluster_center", [7.0, 8.0], "cluster_spread", 0.8);
configs(2) = struct( ...
    "name", "n9_N3", "n", 9, "seed", 17, "duration", 75.0, ...
    "split_time", 12.0, ...
    "sources", [2.0, 12.0; 12.0, 12.0; 7.0, 2.0], ...
    "cluster_center", [7.0, 8.0], "cluster_spread", 0.7);
configs(3) = struct( ...
    "name", "n12_N3", "n", 12, "seed", 23, "duration", 80.0, ...
    "split_time", 15.0, ...
    "sources", [0.0, 11.0; 14.0, 11.0; 7.0, 1.0], ...
    "cluster_center", [7.0, 7.5], "cluster_spread", 0.9);
end

%% Gap 2 — Moving source
function summary = runGap2MovingSource(runRoot, outputOpts, simOpts)
gapDir = fullfile(runRoot, "gap2");
if ~exist(gapDir, "dir")
    mkdir(gapDir);
end

fprintf("=== Gap 2: Moving / non-stationary source (%s) ===\n", simOpts.model);

n = 6;
pS0 = [5.5, 5.5];
velocity = [0.03, 0.02];
motionType = "linear";
circCenter = [8.0, 8.0];
circRadius = 2.0;
circOmega = 0.05;

kappa = 1.0;
R = 2.0;
Dmax = 12.0;
noiseModel = "bounded";
noiseBound = 0.2;
duration = 80.0;
seed = 7;

rng(seed, "twister");
initialPositions = [
    0.0, 0.0; -2.5, -0.5; 5.0, 1.0;
    0.5, 3.5; 1.0, 4.0; 6.0, 2.0
] + (pS0 - [5.5, 5.5]);
initialHeadings = zeros(n, 1);

adjacency = paperFig1Adjacency(n);
phi = globalFormationSlots(n);

cfg = struct( ...
    "n", n, "source0", pS0, "motion_type", motionType, ...
    "velocity", velocity, "circ_center", circCenter, ...
    "circ_radius", circRadius, "circ_omega", circOmega, ...
    "kappa", kappa, "R", R, "Dmax", Dmax, ...
    "alpha", simOpts.alpha, "beta", simOpts.beta, ...
    "noise_model", noiseModel, "noise_bound", noiseBound, ...
    "sign_boundary_layer", simOpts.sign_boundary_layer, ...
    "dt", simOpts.dt, "duration", duration, "seed", seed, ...
    "robot_model", simOpts.model, "offset", simOpts.offset, ...
    "gap", "gap2_moving_source");

steps = round(duration / simOpts.dt);
times = (0:steps)' * simOpts.dt;
sourcePath = zeros(steps + 1, 2);
positions = zeros(n, 2, steps + 1);
headings = zeros(n, steps + 1);
centroid = zeros(steps + 1, 2);
trackingError = zeros(steps + 1, 1);
formationError = zeros(steps + 1, 1);
positions(:, :, 1) = initialPositions;
headings(:, 1) = initialHeadings;

for step = 1:(steps + 1)
    t = times(step);
    ps = sourceAtTime(t, cfg);
    sourcePath(step, :) = ps;

    p = positions(:, :, step);
    theta = headings(:, step);
    [statePos, ~] = controlStates(p, theta, simOpts);

    cfgStep = cfg;
    cfgStep.source = ps;
    [sigma, ~] = measureSingleSource(statePos, cfgStep);
    centroid(step, :) = mean(statePos, 1);
    trackingError(step) = norm(centroid(step, :) - ps);
    formationError(step) = formationErrorValue(statePos, R, phi);

    if step == steps + 1
        break
    end

    u = eq4SingleSource(statePos, adjacency, phi, sigma, cfgStep);
    [positions(:, :, step + 1), headings(:, step + 1)] = ...
        advanceDynamics(p, theta, u, simOpts);
end

vMax = norm(velocity);
if motionType == "circular"
    vMax = circRadius * circOmega;
end
issPredicted = vMax / (2 * simOpts.beta * kappa) + noiseBound / (kappa * R);
tailStart = max(1, floor(0.75 * numel(trackingError)));
tailMean = mean(trackingError(tailStart:end));

summary = struct();
summary.gap = "gap2_moving_source";
summary.description = "Track time-varying source with baseline Eq. 4";
summary.parameters = struct( ...
    "motion_type", motionType, "velocity", velocity, ...
    "circ_center", circCenter, "circ_radius", circRadius, ...
    "circ_omega", circOmega, "v_max", vMax, ...
    "alpha", simOpts.alpha, "beta", simOpts.beta, "kappa", kappa, "duration", duration);
summary.metrics = struct( ...
    "peak_tracking_error", max(trackingError), ...
    "final_tracking_error", trackingError(end), ...
    "tail_mean_tracking_error", tailMean, ...
    "final_formation_error", formationError(end));
summary.validation = struct( ...
    "iss_predicted_offset", issPredicted, ...
    "tail_mean_below_2x_predicted", tailMean < 2 * issPredicted, ...
    "gain_ratio", simOpts.alpha / simOpts.beta);

result = struct("times", times, "positions", positions, "headings", headings, ...
    "centroid", centroid, "source_path", sourcePath, ...
    "tracking_error", trackingError, "localization_error", trackingError, ...
    "formation_error", formationError, "phi", phi, "cfg", cfg, "summary", summary);

saveGapOutputs(result, cfg, gapDir, outputOpts);
save(fullfile(gapDir, "result.mat"), "result", "cfg");
writeJson(fullfile(gapDir, "summary.json"), summary);

fprintf("  Motion: %s, v_max=%.4g\n", motionType, vMax);
fprintf("  Tail mean tracking error: %.4f (ISS predicted offset ~ %.4f)\n", tailMean, issPredicted);
fprintf("  Output: %s\n\n", gapDir);
end

%% Gap 3 — Global maximum on non-convex field
function summary = runGap3GlobalMaximum(runRoot, outputOpts, simOpts)
gapDir = fullfile(runRoot, "gap3");
if ~exist(gapDir, "dir")
    mkdir(gapDir);
end

fprintf("=== Gap 3: Global maximum vs local traps (%s) ===\n", simOpts.model);

n = 6;
peakCenters = [8.0, 8.0; 3.0, 3.0; 8.0, 3.0];
peakAmplitudes = [5.0; 2.0; 1.5];
fieldSigma = 2.0;

kappa = 1.0;
R = 2.0;
Dmax = 15.0;
noiseModel = "none";
duration = 60.0;
seed = 11;
seekMode = "ascent";

rng(seed, "twister");
initialPositions = [
    2.0, 2.5; 3.5, 2.0; 2.5, 4.0;
    4.0, 3.5; 3.0, 3.0; 4.5, 2.5
];
initialHeadings = zeros(n, 1);

adjacency = paperFig1Adjacency(n);
phi = globalFormationSlots(n);

cfg = struct( ...
    "n", n, "peak_centers", peakCenters, "peak_amplitudes", peakAmplitudes, ...
    "field_sigma", fieldSigma, "seek_mode", seekMode, ...
    "kappa", kappa, "R", R, "Dmax", Dmax, ...
    "alpha", simOpts.alpha, "beta", simOpts.beta, ...
    "noise_model", noiseModel, "noise_bound", 0.0, ...
    "sign_boundary_layer", simOpts.sign_boundary_layer, ...
    "dt", simOpts.dt, "duration", duration, "seed", seed, ...
    "robot_model", simOpts.model, "offset", simOpts.offset, ...
    "gap", "gap3_global_maximum");

steps = round(duration / simOpts.dt);
times = (0:steps)' * simOpts.dt;
positions = zeros(n, 2, steps + 1);
headings = zeros(n, steps + 1);
centroid = zeros(steps + 1, 2);
fieldAtCentroid = zeros(steps + 1, 1);
nearestPeakDist = zeros(steps + 1, 1);
formationError = zeros(steps + 1, 1);
positions(:, :, 1) = initialPositions;
headings(:, 1) = initialHeadings;

for step = 1:(steps + 1)
    p = positions(:, :, step);
    theta = headings(:, step);
    [statePos, ~] = controlStates(p, theta, simOpts);

    [sigma, ~] = measureGaussianPeaks(statePos, cfg);
    centroid(step, :) = mean(statePos, 1);
    fieldAtCentroid(step) = gaussianPeakField(centroid(step, :), cfg);
    formationError(step) = formationErrorValue(statePos, R, phi);
    [~, nearestIdx] = min(vecnorm(peakCenters - centroid(step, :), 2, 2));
    nearestPeakDist(step) = norm(centroid(step, :) - peakCenters(nearestIdx, :));

    if step == steps + 1
        break
    end

    u = eq4PeakSeek(statePos, adjacency, phi, sigma, cfg);
    [positions(:, :, step + 1), headings(:, step + 1)] = ...
        advanceDynamics(p, theta, u, simOpts);
end

globalMaxValue = peakAmplitudes(1);
finalField = fieldAtCentroid(end);
[~, finalPeakIdx] = min(vecnorm(peakCenters - centroid(end, :), 2, 2));
reachedGlobal = finalPeakIdx == 1 && nearestPeakDist(end) < 2 * R;

summary = struct();
summary.gap = "gap3_global_maximum";
summary.description = "Non-convex multi-peak field; ascent via flipped localization term";
summary.parameters = struct( ...
    "peak_centers", peakCenters, "peak_amplitudes", peakAmplitudes, ...
    "field_sigma", fieldSigma, "seek_mode", seekMode, "seed", seed);
summary.metrics = struct( ...
    "initial_field_at_centroid", fieldAtCentroid(1), ...
    "final_field_at_centroid", finalField, ...
    "global_max_field_value", globalMaxValue, ...
    "field_fraction_of_global", finalField / globalMaxValue, ...
    "nearest_peak_index", finalPeakIdx, ...
    "nearest_peak_distance", nearestPeakDist(end), ...
    "final_formation_error", formationError(end));
summary.validation = struct( ...
    "reached_global_peak", reachedGlobal, ...
    "trapped_at_local_peak", finalPeakIdx ~= 1, ...
    "gain_ratio", simOpts.alpha / simOpts.beta, ...
    "note", "Local trap expected when initialized near suboptimal peak; escape mechanisms are future work");

result = struct("times", times, "positions", positions, ...
    "centroid", centroid, "field_at_centroid", fieldAtCentroid, ...
    "localization_error", nearestPeakDist, ...
    "formation_error", formationError, "phi", phi, "cfg", cfg, "summary", summary);

saveGapOutputs(result, cfg, gapDir, outputOpts);
save(fullfile(gapDir, "result.mat"), "result", "cfg");
writeJson(fullfile(gapDir, "summary.json"), summary);

fprintf("  Final peak index: %d (1 = global max at [%.1f, %.1f])\n", ...
    finalPeakIdx, peakCenters(1, 1), peakCenters(1, 2));
fprintf("  Field at centroid: %.4f / global max %.4f\n", finalField, globalMaxValue);
fprintf("  Trapped at local peak: %d\n", summary.validation.trapped_at_local_peak);
fprintf("  Output: %s\n\n", gapDir);
end

%% Shared helpers

function simOpts = buildSimOpts(robotModel)
simOpts = struct();
simOpts.model = robotModel;
switch robotModel
    case "single_integrator"
        simOpts.alpha = 100.0;
        simOpts.beta = 0.05;
        simOpts.dt = 0.0005;
        simOpts.offset = 0.0;
        simOpts.sign_boundary_layer = 0.0;
    case "unicycle"
        simOpts.alpha = 10.0;
        simOpts.beta = 0.05;
        simOpts.dt = 0.004;
        simOpts.offset = 2.0;
        simOpts.sign_boundary_layer = 0.2;
    otherwise
        error("Unknown robot model: %s", robotModel);
end
end

function [statePos, isUnicycle] = controlStates(positions, headings, simOpts)
isUnicycle = strcmp(simOpts.model, "unicycle");
if isUnicycle
    c = cos(headings(:));
    s = sin(headings(:));
    statePos = positions + simOpts.offset * [c, s];
else
    statePos = positions;
end
end

function [pNext, thetaNext] = advanceDynamics(p, theta, u, simOpts)
if strcmp(simOpts.model, "unicycle")
    [v, omega] = feedbackLinearize(u, theta, simOpts.offset);
    pNext = p + simOpts.dt * [v .* cos(theta), v .* sin(theta)];
    thetaNext = wrapAngle(theta + simOpts.dt * omega);
else
    pNext = p + simOpts.dt * u;
    thetaNext = theta;
end
end

function positions = clusterInitialPositions(n, center, spread)
angles = 2 * pi * rand(n, 1);
radii = spread * sqrt(rand(n, 1));
positions = center + radii .* [cos(angles), sin(angles)];
end

function assignment = balancedSourceAssign(positions, sources)
n = size(positions, 1);
N = size(sources, 1);
if mod(n, N) ~= 0
    error("balancedSourceAssign requires n divisible by N (got n=%d, N=%d).", n, N);
end
capacity = n / N;
pairs = [];
for i = 1:n
    for k = 1:N
        pairs = [pairs; i, k, norm(positions(i, :) - sources(k, :))]; %#ok<AGROW>
    end
end
[~, order] = sort(pairs(:, 3));
assignment = zeros(n, 1);
assignedCount = zeros(N, 1);
for r = 1:size(order, 1)
    idx = order(r);
    i = pairs(idx, 1);
    k = pairs(idx, 2);
    if assignment(i) == 0 && assignedCount(k) < capacity
        assignment(i) = k;
        assignedCount(k) = assignedCount(k) + 1;
    end
    if all(assignedCount == capacity)
        break
    end
end
if any(assignment == 0)
    error("balancedSourceAssign failed to assign all robots.");
end
end

function adjacency = ringAdjacency(n)
adjacency = zeros(n, n);
for i = 1:n
    j = mod(i, n) + 1;
    adjacency(i, j) = 1;
    adjacency(j, i) = 1;
end
end

function phi = globalFormationSlots(n)
theta = 2 * pi * (0:(n - 1))' / n;
phi = [cos(theta), sin(theta)];
end

function d = pairwiseDistances(points)
n = size(points, 1);
d = inf;
for i = 1:n
    for j = i + 1:n
        d = min(d, norm(points(i, :) - points(j, :)));
    end
end
end

function [v, omega] = feedbackLinearize(commands, headings, r)
c = cos(headings(:));
s = sin(headings(:));
fx = commands(:, 1);
fy = commands(:, 2);
v = c .* fx + s .* fy;
omega = (-s .* fx + c .* fy) / r;
end

function a = wrapAngle(a)
a = mod(a + pi, 2 * pi) - pi;
end

function s = satSign(d, epsBl)
if epsBl > 0
    s = max(min(d / epsBl, 1.0), -1.0);
else
    s = sign(d);
end
end

function assignment = assignNearestSource(positions, sources)
n = size(positions, 1);
assignment = zeros(n, 1);
for i = 1:n
    dists = vecnorm(sources - positions(i, :), 2, 2);
    [~, assignment(i)] = min(dists);
end
end

function adjacency = teamAdjacency(n, assignment)
adjacency = zeros(n, n);
teamIds = unique(assignment, "stable");
for k = 1:numel(teamIds)
    members = find(assignment == teamIds(k))';
    for a = 1:numel(members)
        i = members(a);
        j = members(mod(a, numel(members)) + 1);
        adjacency(i, j) = 1;
        adjacency(j, i) = 1;
    end
end
end

function phi = teamFormationSlots(n, assignment)
phi = zeros(n, 2);
teamIds = unique(assignment, "stable");
for k = 1:numel(teamIds)
    members = find(assignment == teamIds(k));
    nk = numel(members);
    for r = 1:nk
        theta = 2 * pi * (r - 1) / nk;
        phi(members(r), :) = [cos(theta), sin(theta)];
    end
end
end

function adjacency = paperFig1Adjacency(n)
adjacency = zeros(n, n);
edges = [1 2; 1 3; 1 6; 2 3; 2 5; 3 4; 4 5; 5 6];
for e = 1:size(edges, 1)
    i = edges(e, 1);
    j = edges(e, 2);
    adjacency(i, j) = 1;
    adjacency(j, i) = 1;
end
end

function ps = sourceAtTime(t, cfg)
switch cfg.motion_type
    case "linear"
        ps = cfg.source0 + t * cfg.velocity;
    case "circular"
        ps = cfg.circ_center + cfg.circ_radius * ...
            [cos(cfg.circ_omega * t), sin(cfg.circ_omega * t)];
    otherwise
        error("Unknown motion_type: %s", cfg.motion_type);
end
end

function [sigma, informed] = measureSingleSource(positions, cfg)
distances = sqrt(sum((positions - cfg.source) .^ 2, 2));
informed = distances < cfg.Dmax;
sigma = ones(cfg.n, 1) * (cfg.kappa * cfg.Dmax * cfg.Dmax + cfg.noise_bound);
if any(informed)
    delta = positions(informed, :) - cfg.source;
    clean = cfg.kappa * sum(delta .^ 2, 2);
    sigma(informed) = clean + sampleNoise(cfg, sum(informed));
end
end

function [sigma, informed] = measureMultiSource(positions, cfg)
n = cfg.n;
sigma = zeros(n, 1);
informed = false(n, 1);
for i = 1:n
    srcIdx = cfg.assignment(i);
    ps = cfg.sources(srcIdx, :);
    dist = norm(positions(i, :) - ps);
    if dist < cfg.Dmax
        informed(i) = true;
        clean = cfg.kappa * dist * dist;
        sigma(i) = clean + sampleNoise(cfg, 1);
    else
        sigma(i) = cfg.kappa * cfg.Dmax * cfg.Dmax + cfg.noise_bound;
    end
end
end

function val = gaussianPeakField(point, cfg)
val = 0.0;
for k = 1:size(cfg.peak_centers, 1)
    d2 = sum((point - cfg.peak_centers(k, :)) .^ 2);
    val = val + cfg.peak_amplitudes(k) * exp(-d2 / (2 * cfg.field_sigma ^ 2));
end
end

function [sigma, informed] = measureGaussianPeaks(positions, cfg)
n = cfg.n;
sigma = zeros(n, 1);
informed = false(n, 1);
satVal = max(cfg.peak_amplitudes);
for i = 1:n
    val = gaussianPeakField(positions(i, :), cfg);
    minDist = min(vecnorm(cfg.peak_centers - positions(i, :), 2, 2));
    if minDist < cfg.Dmax
        informed(i) = true;
        sigma(i) = val + sampleNoise(cfg, 1);
    else
        sigma(i) = satVal;
    end
end
end

function noise = sampleNoise(cfg, count)
switch cfg.noise_model
    case "none"
        noise = zeros(count, 1);
    case "bounded"
        noise = cfg.noise_bound * (2 * rand(count, 1) - 1);
    otherwise
        noise = zeros(count, 1);
end
end

function controls = eq4SingleSource(positions, adjacency, phi, sigma, cfg)
epsBl = 0.0;
if isfield(cfg, "sign_boundary_layer")
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
    controls(i, :) = controls(i, :) - ...
        (2 * cfg.beta / cfg.R) * sigma(i) * phi(i, :);
end
end

function controls = eq4MultiSource(positions, adjacency, phi, sigma, cfg)
controls = eq4SingleSource(positions, adjacency, phi, sigma, cfg);
end

function controls = eq4PeakSeek(positions, adjacency, phi, sigma, cfg)
epsBl = 0.0;
if isfield(cfg, "sign_boundary_layer")
    epsBl = cfg.sign_boundary_layer;
end
z = positions - cfg.R * phi;
controls = zeros(size(positions));
locSign = -1.0;
if isfield(cfg, "seek_mode") && cfg.seek_mode == "ascent"
    locSign = 1.0;
end
for i = 1:cfg.n
    neighbors = find(adjacency(i, :) ~= 0);
    if ~isempty(neighbors)
        controls(i, :) = controls(i, :) + ...
            cfg.alpha * sum(satSign(z(neighbors, :) - z(i, :), epsBl), 1);
    end
    controls(i, :) = controls(i, :) + ...
        locSign * (2 * cfg.beta / cfg.R) * sigma(i) * phi(i, :);
end
end

function value = formationErrorValue(positions, R, phi)
z = positions - R * phi;
zCentroid = mean(z, 1);
value = sqrt(sum((z - zCentroid) .^ 2, "all"));
end

function bounds = teamTheoryBounds(cfg, teamSizes, noiseBound, finalErrors)
fDmax = cfg.kappa * cfg.Dmax * cfg.Dmax + noiseBound;
gainThreshold = 4 * max(teamSizes) * fDmax / cfg.R;
nTeams = numel(teamSizes);
bounds.epsilon = cell(nTeams, 1);
bounds.inside_bound = false(nTeams, 1);
for k = 1:nTeams
    nk = teamSizes(k);
    denom = cfg.kappa * cfg.R * (2 * pi * nk - nk * abs(sin(2 * pi * nk / nk)));
    epsK = 2 * pi * nk * noiseBound / denom;
    bounds.epsilon{k} = epsK;
    bounds.inside_bound(k) = finalErrors(k) <= epsK;
end
bounds.gain_threshold = gainThreshold;
end

function saveGap1Outputs(result, runDir, outputOpts)
if ~outputOpts.save_plots && ~outputOpts.save_animation && ~outputOpts.save_telemetry
    return
end
if outputOpts.save_plots
    plotGap1Trajectory(result, runDir);
    plotGap1TeamLocalization(result, runDir);
    plotGap1PhaseTimeline(result, runDir);
    plotGap1DeploymentMap(result, runDir);
end
if outputOpts.save_animation
    writeGap1Animation(result, runDir, outputOpts.animation_fps);
end
if outputOpts.save_telemetry
    telemetry = buildGap1Telemetry(result, outputOpts.telemetry_max_samples);
    writeJson(fullfile(runDir, "telemetry.json"), telemetry);
end
end

function plotGap1Trajectory(result, runDir)
cfg = result.cfg;
n = cfg.n;
splitIdx = cfg.split_step;
palette = lines(max(result.assignment_trace(:, end)));
fig = figure("Visible", "off", "Position", [100 100 900 500]);
hold on

for i = 1:n
    if isfield(result, "control_points")
        xy = squeeze(result.control_points(i, :, :))';
    else
        xy = squeeze(result.positions(i, :, :))';
    end
    pre = xy(1:splitIdx, :);
    post = xy(splitIdx:end, :);
    team = result.assignment_trace(i, end);
    plot(pre(:, 1), pre(:, 2), "-", "Color", [0.55 0.55 0.55], "LineWidth", 1.0);
    plot(post(:, 1), post(:, 2), "-", "Color", palette(team, :), "LineWidth", 1.4);
    plot(xy(1, 1), xy(1, 2), "o", "Color", [0.4 0.4 0.4], "MarkerSize", 5);
    plot(xy(end, 1), xy(end, 2), "s", "Color", palette(team, :), "MarkerSize", 6);
end

for k = 1:size(cfg.sources, 1)
    plot(cfg.sources(k, 1), cfg.sources(k, 2), "p", "MarkerSize", 14, ...
        "MarkerFaceColor", palette(k, :), "Color", palette(k, :));
    text(cfg.sources(k, 1) + 0.2, cfg.sources(k, 2) + 0.2, sprintf("S%d", k));
end
plot(result.centroid(splitIdx, 1), result.centroid(splitIdx, 2), ...
    "kd", "MarkerSize", 10, "MarkerFaceColor", "y");
text(result.centroid(splitIdx, 1), result.centroid(splitIdx, 2), "  split", "FontSize", 9);

axis equal
grid on
xlabel("x [m]");
ylabel("y [m]");
title(sprintf("Gap 1 %s: unified start then team split (t_{split}=%.1f s)", ...
    cfg.config_name, result.summary.parameters.split_time));
legend({"unified phase", "team phase", "", "", "sources", "split event"}, ...
    "Location", "best");
saveas(fig, fullfile(runDir, "trajectory.png"));
close(fig);
end

function plotGap1TeamLocalization(result, runDir)
cfg = result.cfg;
splitT = result.summary.parameters.split_time;
fig = figure("Visible", "off");
hold on
colors = lines(size(result.team_loc_error, 2));
for k = 1:size(result.team_loc_error, 2)
    plot(result.times, result.team_loc_error(:, k), "LineWidth", 1.3, "Color", colors(k, :));
    if isfield(result.summary.validation, "per_team_epsilon")
        epsK = result.summary.validation.per_team_epsilon{k};
        if ~isempty(epsK)
            yline(epsK, "--", "Color", colors(k, :), "LineWidth", 1.0);
        end
    end
end
xline(splitT, "k--", "split", "LineWidth", 1.2, "LabelVerticalAlignment", "bottom");
grid on
xlabel("time [s]");
ylabel("||centroid_k - source_k||");
title("Per-team localization error (post-split meaningful)");
legend(arrayfun(@(k) sprintf("team %d", k), 1:size(result.team_loc_error, 2), "UniformOutput", false), ...
    "Location", "best");
saveas(fig, fullfile(runDir, "team_localization.png"));
close(fig);
end

function plotGap1PhaseTimeline(result, runDir)
cfg = result.cfg;
splitT = result.summary.parameters.split_time;
fig = figure("Visible", "off", "Position", [100 100 900 400]);

subplot(2, 1, 1);
plot(result.times, result.formation_error, "b", "LineWidth", 1.2);
hold on
xline(splitT, "k--", "split", "LineWidth", 1.2);
ylabel("formation error");
title("Swarm formation error (unified then per-team slots)");
grid on

subplot(2, 1, 2);
plot(result.times, result.team_separation, "m", "LineWidth", 1.2);
hold on
xline(splitT, "k--", "split", "LineWidth", 1.2);
xlabel("time [s]");
ylabel("min inter-team centroid distance");
title("Team separation (rises after split)");
grid on

sgtitle(sprintf("Gap 1 %s phase timeline", cfg.config_name));
saveas(fig, fullfile(runDir, "phase_timeline.png"));
close(fig);
end

function plotGap1DeploymentMap(result, runDir)
cfg = result.cfg;
n = cfg.n;
Nsrc = size(cfg.sources, 1);
assignment = result.assignment_trace(:, end);
palette = lines(Nsrc);

if isfield(result, "control_points")
    finalPos = squeeze(result.control_points(:, :, end));
else
    finalPos = squeeze(result.positions(:, :, end));
end

fig = figure("Visible", "off", "Position", [100 100 700 600]);
hold on

% Voronoi boundaries between sources (optional; skip if degenerate).
try
    [vx, vy] = voronoi(cfg.sources(:, 1), cfg.sources(:, 2));
    plot(vx, vy, ":", "Color", [0.6 0.6 0.6], "LineWidth", 1.0);
catch
    % Collinear or degenerate source layout; skip Voronoi overlay.
end

for k = 1:Nsrc
    plot(cfg.sources(k, 1), cfg.sources(k, 2), "p", "MarkerSize", 16, ...
        "MarkerFaceColor", palette(k, :), "Color", palette(k, :));
    ck = squeeze(result.centroids(k, :, end));
    plot(ck(1), ck(2), "x", "Color", palette(k, :), "MarkerSize", 12, "LineWidth", 2);
    plot([ck(1), cfg.sources(k, 1)], [ck(2), cfg.sources(k, 2)], ...
        "--", "Color", palette(k, :), "LineWidth", 1.0);
end

for i = 1:n
    plot(finalPos(i, 1), finalPos(i, 2), "o", "MarkerFaceColor", palette(assignment(i), :), ...
        "MarkerEdgeColor", "k", "MarkerSize", 8);
end

axis equal
grid on
xlabel("x [m]");
ylabel("y [m]");
title(sprintf("Gap 1 %s: final deployment (robots, team centroids, sources)", cfg.config_name));
legend({"Voronoi", "sources", "team centroids", "centroid-source", "robots"}, "Location", "best");
saveas(fig, fullfile(runDir, "deployment_map.png"));
close(fig);
end

function writeGap1Animation(result, runDir, animationFps)
gifPath = fullfile(runDir, "motion.gif");
cfg = result.cfg;
n = cfg.n;
splitIdx = cfg.split_step;
nFrames = numel(result.times);
stride = max(1, floor(nFrames / 120));
frames = 1:stride:nFrames;
palette = lines(max(result.assignment_trace(:, end)));

if isfield(result, "control_points")
    traj = result.control_points;
else
    traj = result.positions;
end
allXy = reshape(permute(traj, [3 1 2]), [], 2);
pad = 1.5;
xlims = [min([allXy(:, 1); cfg.sources(:, 1)]) - pad, max([allXy(:, 1); cfg.sources(:, 1)]) + pad];
ylims = [min([allXy(:, 2); cfg.sources(:, 2)]) - pad, max([allXy(:, 2); cfg.sources(:, 2)]) + pad];

fig = figure("Visible", "off", "Position", [100, 100, 1000, 500]);
first = true;
for idx = frames
    clf(fig);
    subplot(1, 2, 1);
    hold on
    for i = 1:n
        xi = squeeze(traj(i, 1, 1:idx));
        yi = squeeze(traj(i, 2, 1:idx));
        team = result.assignment_trace(i, idx);
        if idx < splitIdx
            c = [0.5 0.5 0.5];
        else
            c = palette(team, :);
        end
        plot(xi, yi, "Color", c, "LineWidth", 0.8);
    end
    cur = squeeze(traj(:, :, idx));
    plot(cur(:, 1), cur(:, 2), "ko", "MarkerFaceColor", "w", "MarkerSize", 4);
    for k = 1:size(cfg.sources, 1)
        plot(cfg.sources(k, 1), cfg.sources(k, 2), "p", "MarkerSize", 12, ...
            "MarkerFaceColor", palette(k, :));
    end
    if idx >= splitIdx
        for k = 1:size(result.centroids, 1)
            ck = squeeze(result.centroids(k, :, idx));
            plot(ck(1), ck(2), "kx", "MarkerSize", 10, "LineWidth", 1.5);
        end
    end
    xlim(xlims); ylim(ylims);
    axis equal; grid on
    xlabel("x [m]"); ylabel("y [m]");
    phaseLabel = "unified";
    if idx >= splitIdx
        phaseLabel = "split teams";
    end
    title(sprintf("%s | t=%.2fs (%s)", cfg.config_name, result.times(idx), phaseLabel));

    subplot(2, 2, 2);
    plot(result.times(1:idx), result.formation_error(1:idx), "b", "LineWidth", 1.2);
    hold on
    if idx >= splitIdx
        xline(result.times(splitIdx), "k--");
    end
    grid on; ylabel("formation err"); title("Formation");

    subplot(2, 2, 4);
    plot(result.times(1:idx), mean(result.team_loc_error(1:idx, :), 2), "r", "LineWidth", 1.2);
    hold on
    if idx >= splitIdx
        xline(result.times(splitIdx), "k--");
    end
    grid on; xlabel("time [s]"); ylabel("mean team loc err"); title("Localization");

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

function telemetry = buildGap1Telemetry(result, maxSamples)
[idx, stride] = decimateIndices(numel(result.times), maxSamples);
telemetry = struct();
telemetry.meta = struct("gap", "gap1", "config", result.cfg.config_name, ...
    "stride", stride, "n_samples", numel(idx));
telemetry.times = result.times(idx)';
telemetry.team_loc_error = result.team_loc_error(idx, :);
telemetry.team_separation = result.team_separation(idx)';
telemetry.formation_error = result.formation_error(idx)';
telemetry.phase_tag = result.phase_tag(idx)';
telemetry.summary = result.summary;
end

function saveGapOutputs(result, cfg, runDir, outputOpts)
if ~outputOpts.save_plots && ~outputOpts.save_animation && ~outputOpts.save_telemetry
    return
end

phi = result.phi;
if outputOpts.save_plots
    plotResearchTrajectory(result, cfg, phi, runDir);
    plotSeries(result.times, result.formation_error, "Formation error", ...
        "formation error", [], fullfile(runDir, "formation_error.png"));
    plotFormationErrorPerRobot(result, cfg, phi, runDir);
    plotResearchLocalization(result, cfg, runDir);
end
if outputOpts.save_animation
    writeResearchAnimation(result, cfg, phi, runDir, outputOpts.animation_fps);
end
if outputOpts.save_telemetry
    telemetry = buildResearchTelemetry(result, cfg, phi, outputOpts.telemetry_max_samples);
    writeJson(fullfile(runDir, "telemetry.json"), telemetry);
end
end

function xy = robotTrajectorySlice(result, robotIdx)
if isfield(result, "control_points")
    xy = squeeze(result.control_points(robotIdx, :, :))';
else
    xy = squeeze(result.positions(robotIdx, :, :))';
end
end

function t = trajectoryTensor(result)
if isfield(result, "control_points")
    t = result.control_points;
else
    t = result.positions;
end
end

function plotResearchTrajectory(result, cfg, phi, runDir)
fig = figure("Visible", "off");
hold on
if isfield(cfg, "assignment")
    palette = lines(max(cfg.assignment));
else
    palette = lines(cfg.n);
end
for i = 1:cfg.n
    xy = robotTrajectorySlice(result, i);
    if isfield(cfg, "assignment")
        color = palette(cfg.assignment(i), :);
    else
        color = palette(i, :);
    end
    plot(xy(:, 1), xy(:, 2), "LineWidth", 1.0, "Color", color);
    plot(xy(1, 1), xy(1, 2), "o", "MarkerSize", 5, "Color", color);
    plot(xy(end, 1), xy(end, 2), "s", "MarkerSize", 6, "Color", color);
end

switch cfg.gap
    case "gap1_multi_source"
        for k = 1:size(cfg.sources, 1)
            plot(cfg.sources(k, 1), cfg.sources(k, 2), "rp", "MarkerSize", 14, "MarkerFaceColor", "r");
        end
        if isfield(result, "centroids")
            for k = 1:size(result.centroids, 1)
                ck = squeeze(result.centroids(k, :, end));
                plot(ck(1), ck(2), "kx", "MarkerSize", 10, "LineWidth", 1.5);
            end
        end
        titleStr = sprintf("Gap 1: multi-source trajectories (n=%d, N=%d)", cfg.n, size(cfg.sources, 1));
    case "gap2_moving_source"
        plot(result.source_path(:, 1), result.source_path(:, 2), "r--", "LineWidth", 1.2);
        plot(result.source_path(end, 1), result.source_path(end, 2), "rp", "MarkerSize", 14, "MarkerFaceColor", "r");
        c = result.centroid(end, :);
        circle = c + cfg.R * phi;
        circle = [circle; circle(1, :)];
        plot(circle(:, 1), circle(:, 2), "k:", "LineWidth", 1.2);
        plot(result.centroid(:, 1), result.centroid(:, 2), "k-", "LineWidth", 1.0);
        titleStr = sprintf("Gap 2: moving source (%s)", cfg.motion_type);
    case "gap3_global_maximum"
        for k = 1:size(cfg.peak_centers, 1)
            plot(cfg.peak_centers(k, 1), cfg.peak_centers(k, 2), "k*", "MarkerSize", 12);
        end
        plot(result.centroid(:, 1), result.centroid(:, 2), "b-", "LineWidth", 1.2);
        c = result.centroid(end, :);
        circle = c + cfg.R * phi;
        circle = [circle; circle(1, :)];
        plot(circle(:, 1), circle(:, 2), "k:", "LineWidth", 1.2);
        titleStr = sprintf("Gap 3: global-maximum seeking (%s)", cfg.seek_mode);
    otherwise
        titleStr = "Research gap trajectories";
end

axis equal
grid on
xlabel("x [m]");
ylabel("y [m]");
title(titleStr);
saveas(fig, fullfile(runDir, "trajectory.png"));
close(fig);
end

function plotResearchLocalization(result, cfg, runDir)
fig = figure("Visible", "off");
hold on
switch cfg.gap
    case "gap1_multi_source"
        plot(result.times, result.team_loc_error, "LineWidth", 1.2);
        for k = 1:size(result.team_loc_error, 2)
            if isfield(result.summary.validation, "per_team_epsilon")
                epsK = result.summary.validation.per_team_epsilon{k};
                if ~isempty(epsK)
                    yline(epsK, "--", "LineWidth", 1.0);
                end
            end
        end
        ylabel("||centroid_k - source_k||");
        title("Per-team localization error");
        legend(arrayfun(@(k) sprintf("team %d", k), 1:size(result.team_loc_error, 2), "UniformOutput", false), ...
            "Location", "best");
    case "gap2_moving_source"
        plot(result.times, result.tracking_error, "LineWidth", 1.2);
        issOff = result.summary.validation.iss_predicted_offset;
        yline(issOff, "r--", "ISS offset", "LineWidth", 1.0);
        yline(2 * issOff, "r:", "2x ISS", "LineWidth", 1.0);
        ylabel("||centroid - source(t)||");
        title("Tracking error (moving source)");
        legend({"tracking error", "ISS offset", "2x ISS"}, "Location", "best");
    case "gap3_global_maximum"
        yyaxis left
        plot(result.times, result.field_at_centroid, "b", "LineWidth", 1.2);
        ylabel("f(p^*)");
        yline(cfg.peak_amplitudes(1), "b--", "global max", "LineWidth", 1.0);
        yyaxis right
        plot(result.times, result.localization_error, "r", "LineWidth", 1.0);
        ylabel("dist to nearest peak");
        title("Field value and nearest-peak distance");
        legend({"f(p^*)", "global max", "peak distance"}, "Location", "best");
    otherwise
        plot(result.times, result.localization_error, "LineWidth", 1.2);
        ylabel("localization metric");
        title("Localization error");
end
grid on
xlabel("time [s]");
saveas(fig, fullfile(runDir, "localization_error.png"));
close(fig);
end

function plotFormationErrorPerRobot(result, cfg, phi, runDir)
T = numel(result.times);
traj = trajectoryTensor(result);
z = traj - cfg.R * phi;
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
title("Per-robot formation error");
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

function writeResearchAnimation(result, cfg, phi, runDir, animationFps)
gifPath = fullfile(runDir, "motion.gif");
nFrames = numel(result.times);
stride = max(1, floor(nFrames / 120));
frames = 1:stride:nFrames;
histStride = max(1, floor(nFrames / 1200));
bgT = result.times(1:histStride:end);
bgFe = result.formation_error(1:histStride:end);
meta = sprintf("research/%s  alpha/beta=%g  n=%d", cfg.gap, result.summary.validation.gain_ratio, cfg.n);

if isfield(result, "tracking_error")
    bgLe = result.tracking_error(1:histStride:end);
elseif isfield(result, "team_loc_error")
    bgLe = mean(result.team_loc_error(1:histStride:end, :), 2);
elseif isfield(result, "field_at_centroid")
    bgLe = result.field_at_centroid(1:histStride:end);
else
    bgLe = result.localization_error(1:histStride:end);
end

allXy = reshape(permute(trajectoryTensor(result), [3 1 2]), [], 2);
pad = 1.5;
xlims = [min(allXy(:, 1)) - pad, max(allXy(:, 1)) + pad];
ylims = [min(allXy(:, 2)) - pad, max(allXy(:, 2)) + pad];

if strcmp(cfg.gap, "gap2_moving_source")
    xlims = [min([allXy(:, 1); result.source_path(:, 1)]) - pad, max([allXy(:, 1); result.source_path(:, 1)]) + pad];
    ylims = [min([allXy(:, 2); result.source_path(:, 2)]) - pad, max([allXy(:, 2); result.source_path(:, 2)]) + pad];
elseif strcmp(cfg.gap, "gap1_multi_source")
    xlims = [min([allXy(:, 1); cfg.sources(:, 1)]) - pad, max([allXy(:, 1); cfg.sources(:, 1)]) + pad];
    ylims = [min([allXy(:, 2); cfg.sources(:, 2)]) - pad, max([allXy(:, 2); cfg.sources(:, 2)]) + pad];
elseif strcmp(cfg.gap, "gap3_global_maximum")
    xlims = [min([allXy(:, 1); cfg.peak_centers(:, 1)]) - pad, max([allXy(:, 1); cfg.peak_centers(:, 1)]) + pad];
    ylims = [min([allXy(:, 2); cfg.peak_centers(:, 2)]) - pad, max([allXy(:, 2); cfg.peak_centers(:, 2)]) + pad];
end

fig = figure("Visible", "off", "Position", [100, 100, 1000, 500]);
first = true;
for idx = frames
    clf(fig);
    subplot(1, 2, 1);
    hold on
    hist = unique([1:histStride:idx, idx]);
    traj = trajectoryTensor(result);
    for i = 1:cfg.n
        xi = reshape(traj(i, 1, hist), [], 1);
        yi = reshape(traj(i, 2, hist), [], 1);
        plot(xi, yi, "LineWidth", 0.7);
    end
    current = traj(:, :, idx);
    plot(current(:, 1), current(:, 2), "bo", "MarkerFaceColor", "b", "MarkerSize", 5);

    switch cfg.gap
        case "gap1_multi_source"
            for k = 1:size(cfg.sources, 1)
                plot(cfg.sources(k, 1), cfg.sources(k, 2), "rp", "MarkerSize", 12, "MarkerFaceColor", "r");
            end
            for k = 1:size(result.centroids, 1)
                ck = squeeze(result.centroids(k, :, idx));
                plot(ck(1), ck(2), "kx", "MarkerSize", 10, "LineWidth", 1.5);
            end
            panelTitle = sprintf("Multi-source teams (t = %.2f s)", result.times(idx));
        case "gap2_moving_source"
            ps = result.source_path(idx, :);
            plot(result.source_path(1:idx, 1), result.source_path(1:idx, 2), "r--", "LineWidth", 1.0);
            plot(ps(1), ps(2), "rp", "MarkerSize", 12, "MarkerFaceColor", "r");
            c = result.centroid(idx, :);
            circle = c + cfg.R * phi;
            circle = [circle; circle(1, :)];
            plot(circle(:, 1), circle(:, 2), "k:", "LineWidth", 1.2);
            plot(c(1), c(2), "kx", "MarkerSize", 10, "LineWidth", 1.5);
            panelTitle = sprintf("Moving source (t = %.2f s)", result.times(idx));
        case "gap3_global_maximum"
            for k = 1:size(cfg.peak_centers, 1)
                plot(cfg.peak_centers(k, 1), cfg.peak_centers(k, 2), "k*", "MarkerSize", 10);
            end
            c = result.centroid(idx, :);
            plot(result.centroid(1:idx, 1), result.centroid(1:idx, 2), "b-", "LineWidth", 1.0);
            circle = c + cfg.R * phi;
            circle = [circle; circle(1, :)];
            plot(circle(:, 1), circle(:, 2), "k:", "LineWidth", 1.2);
            plot(c(1), c(2), "kx", "MarkerSize", 10, "LineWidth", 1.5);
            panelTitle = sprintf("Global-max seeking (t = %.2f s)", result.times(idx));
    end

    xlim(xlims); ylim(ylims);
    axis equal
    grid on
    xlabel("x [m]"); ylabel("y [m]");
    title(panelTitle);

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
    if strcmp(cfg.gap, "gap1_multi_source")
        leHist = mean(result.team_loc_error(hist, :), 2);
        leNow = mean(result.team_loc_error(idx, :));
    elseif strcmp(cfg.gap, "gap2_moving_source")
        leHist = result.tracking_error(hist);
        leNow = result.tracking_error(idx);
        yline(result.summary.validation.iss_predicted_offset, "r--", "LineWidth", 1.0);
    elseif strcmp(cfg.gap, "gap3_global_maximum")
        leHist = result.field_at_centroid(hist);
        leNow = result.field_at_centroid(idx);
        yline(cfg.peak_amplitudes(1), "r--", "LineWidth", 1.0);
    else
        leHist = result.localization_error(hist);
        leNow = result.localization_error(idx);
    end
    plot(result.times(hist), leHist, "b", "LineWidth", 1.2);
    plot(result.times(idx), leNow, "ro", "MarkerFaceColor", "r");
    grid on
    xlabel("time [s]");
    if strcmp(cfg.gap, "gap3_global_maximum")
        ylabel("f(p^*)");
        title("Field at centroid");
    else
        ylabel("localization metric");
        title("Localization / tracking error");
    end

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

function telemetry = buildResearchTelemetry(result, cfg, phi, maxSamples)
[idx, stride] = decimateIndices(numel(result.times), maxSamples);
T = numel(idx);
n = cfg.n;

fePerRobot = zeros(n, T);
posOut = zeros(T, n, 2);
for k = 1:T
    t = idx(k);
    p = trajectoryTensor(result);
    p = p(:, :, t);
    posOut(k, :, :) = p;
    z = p - cfg.R * phi;
    zc = mean(z, 1);
    fePerRobot(:, k) = sqrt(sum((z - zc) .^ 2, 2));
end

telemetry = struct();
telemetry.meta = struct( ...
    "model", cfg.gap, ...
    "generated_at", char(datetime("now", "TimeZone", "local", "Format", "yyyy-MM-dd'T'HH:mm:ss")), ...
    "dt", cfg.dt, "duration", cfg.duration, "n_robots", n, ...
    "seed", cfg.seed, "telemetry_stride", stride, "n_samples", T, ...
    "note", "Full-resolution data is in result.mat; JSON may be decimated.");
telemetry.parameters = result.summary.parameters;
telemetry.validation = result.summary.validation;
telemetry.metrics = result.summary.metrics;
telemetry.times = result.times(idx)';
telemetry.centroid = result.centroid(idx, :);
telemetry.formation_error = result.formation_error(idx)';
telemetry.formation_error_per_robot = fePerRobot;
telemetry.positions = posOut;

switch cfg.gap
    case "gap1_multi_source"
        telemetry.team_loc_error = result.team_loc_error(idx, :);
        telemetry.sources = cfg.sources;
        telemetry.assignment = cfg.assignment;
    case "gap2_moving_source"
        telemetry.tracking_error = result.tracking_error(idx)';
        telemetry.source_path = result.source_path(idx, :);
    case "gap3_global_maximum"
        telemetry.field_at_centroid = result.field_at_centroid(idx)';
        telemetry.nearest_peak_distance = result.localization_error(idx)';
        telemetry.peak_centers = cfg.peak_centers;
end
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
