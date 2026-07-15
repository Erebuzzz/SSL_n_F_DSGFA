function result = run_from_config(config_path)
%RUN_FROM_CONFIG Run the MATLAB parity simulator from a shared JSON config.

cfg = sgf_config(config_path);
result = sgf_run(cfg);
sgf_plot_results(result, cfg);
sgf_save_result(result, cfg);

fprintf('MATLAB parity run complete: %s\n', cfg.run_id);
fprintf('Output folder: %s\n', cfg.run_dir);
fprintf('Final localization error: %.6g\n', result.summary.metrics.final_localization_error);
if result.summary.validation.bound_applicable
    fprintf('Inside theorem bound: %d\n', result.summary.validation.inside_bound);
end
end

function result = sgf_run(cfg)
rng(cfg.seed, 'twister');

steps = round(cfg.duration / cfg.dt);
times = (0:steps)' * cfg.dt;
positions = zeros(cfg.n, 2, steps + 1);
centroid = zeros(steps + 1, 2);
formation_error = zeros(steps + 1, 1);
localization_error = zeros(steps + 1, 1);
n_informed = zeros(steps + 1, 1);

positions(:, :, 1) = cfg.initial_positions;
phi = sgf_formation_slots(cfg.n);

for step = 1:(steps + 1)
    current = positions(:, :, step);
    [sigma, informed] = sgf_measurement(current, cfg);
    centroid(step, :) = mean(current, 1);
    formation_error(step) = sgf_formation_error(current, cfg.R, phi);
    localization_error(step) = norm(centroid(step, :) - cfg.source);
    n_informed(step) = sum(informed);

    if step == steps + 1
        break
    end

    u = sgf_control(current, cfg.adjacency, phi, sigma, cfg);
    positions(:, :, step + 1) = current + cfg.dt * u;
end

if min(n_informed) == 0
    warning(['No robot is ever within Dmax of the source (0 informed robots): ' ...
        'there is no source signal, so the centroid cannot localize. Move the ' ...
        'source closer, raise Dmax, or start the robots near the source.']);
end
bounds = sgf_theory_bounds(cfg, min(n_informed));
summary = sgf_summary(cfg, times, formation_error, localization_error, n_informed, bounds);

result = struct();
result.times = times;
result.positions = positions;
result.centroid = centroid;
result.formation_error = formation_error;
result.localization_error = localization_error;
result.n_informed = n_informed;
result.summary = summary;
end

function phi = sgf_formation_slots(n)
theta = 2 * pi * (0:(n - 1))' / n;
phi = [cos(theta), sin(theta)];
end

function value = sgf_formation_error(positions, R, phi)
z = positions - R * phi;
z_centroid = mean(z, 1);
value = sqrt(sum((z - z_centroid) .^ 2, 'all'));
end

function summary = sgf_summary(cfg, times, formation_error, localization_error, n_informed, bounds)
final_formation = formation_error(end);
final_localization = localization_error(end);
inside_bound = [];
if bounds.bound_applicable
    inside_bound = final_localization <= bounds.epsilon;
end

tail_start = max(1, floor(0.75 * numel(localization_error)));
formation_tail = formation_error(tail_start:end);
localization_tail = localization_error(tail_start:end);
formation_threshold = max(0.1, 0.05 * formation_error(1));
if isempty(bounds.epsilon)
    localization_threshold = 0.1;
else
    localization_threshold = bounds.epsilon;
end

summary = struct();
summary.parameters = struct( ...
    'n', cfg.n, ...
    'source', cfg.source, ...
    'kappa', cfg.kappa, ...
    'radius', cfg.R, ...
    'dmax', cfg.Dmax, ...
    'alpha', cfg.alpha, ...
    'beta', cfg.beta, ...
    'duration', cfg.duration, ...
    'dt', cfg.dt, ...
    'seed', cfg.seed, ...
    'noise_model', cfg.noise_model, ...
    'noise_std', cfg.noise_std, ...
    'noise_bound', cfg.noise_bound, ...
    'topology', cfg.topology_name);

summary.validation = struct( ...
    'gain_ratio', cfg.alpha / cfg.beta, ...
    'gain_threshold', bounds.gain_threshold, ...
    'gain_condition_passed', (cfg.alpha / cfg.beta) > bounds.gain_threshold, ...
    'min_n_informed', min(n_informed), ...
    'final_n_informed', n_informed(end), ...
    'epsilon', bounds.epsilon, ...
    'bound_applicable', bounds.bound_applicable, ...
    'inside_bound', inside_bound);

summary.metrics = struct( ...
    'initial_formation_error', formation_error(1), ...
    'final_formation_error', final_formation, ...
    'initial_localization_error', localization_error(1), ...
    'final_localization_error', final_localization, ...
    'formation_error_drop', formation_error(1) - final_formation, ...
    'localization_error_drop', localization_error(1) - final_localization, ...
    'tail_formation_error_span', max(formation_tail) - min(formation_tail), ...
    'tail_formation_error_std', std(formation_tail), ...
    'tail_localization_error_span', max(localization_tail) - min(localization_tail), ...
    'tail_localization_error_std', std(localization_tail), ...
    'formation_threshold', formation_threshold, ...
    'formation_entry_time', first_entry_time(times, formation_error, formation_threshold), ...
    'localization_threshold', localization_threshold, ...
    'localization_entry_time', first_entry_time(times, localization_error, localization_threshold), ...
    'time_inside_localization_threshold_after_entry', time_inside_after_entry(times, localization_error, localization_threshold));
end

function value = first_entry_time(times, values, threshold)
idx = find(values <= threshold, 1, 'first');
if isempty(idx)
    value = [];
else
    value = times(idx);
end
end

function value = time_inside_after_entry(times, values, threshold)
idx = find(values <= threshold, 1, 'first');
if isempty(idx) || numel(times) < 2
    value = 0;
else
    dt = times(2) - times(1);
    value = sum(values(idx:end) <= threshold) * dt;
end
end

function sgf_save_result(result, cfg)
if ~exist(cfg.run_dir, 'dir')
    mkdir(cfg.run_dir);
end
summary = result.summary;
save(fullfile(cfg.run_dir, 'result.mat'), 'result', 'cfg');
write_json(fullfile(cfg.run_dir, 'summary.json'), summary);
end

function write_json(path, data)
try
    text = jsonencode(data, PrettyPrint=true);
catch
    text = jsonencode(data);
end
fid = fopen(path, 'w');
if fid < 0
    error('Could not open %s for writing.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', text);
end
