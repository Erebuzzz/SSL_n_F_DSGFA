function result = run_turtlebot_from_config(config_path)
%RUN_TURTLEBOT_FROM_CONFIG Run the MATLAB TurtleBot simulator from a shared config.
%
%   Usage (from the repository root):
%       addpath('matlab_turtlebot')
%       run_turtlebot_from_config('matlab_turtlebot/configs/turtlebot_working.json')
%
%   Reuses the Phase 1.3 control/measurement/topology helpers in matlab/ so the
%   controller math is identical to the numerical simulator.

here = fileparts(mfilename('fullpath'));
repo_root = fileparts(here);
addpath(here);
addpath(fullfile(here, 'helpers'));
addpath(fullfile(repo_root, 'matlab'));   % reuse sgf_control / sgf_measurement / sgf_topology / sgf_theory_bounds

cfg = tb_config(config_path);
if cfg.mode == "turtlebot_simulink"
    result = run_turtlebot_simulink(cfg);
else
    result = run_turtlebot_simulation(cfg);
end

if ~exist(cfg.run_dir, 'dir')
    mkdir(cfg.run_dir);
end
save_turtlebot_plots(result, cfg);
if cfg.save_animation
    export_turtlebot_animation(result, cfg);
end
save_turtlebot_summary(result, cfg);

fprintf('TurtleBot run complete: %s\n', cfg.run_id);
fprintf('Output folder: %s\n', cfg.run_dir);
fprintf('Final formation error:    %.5f\n', result.summary.metrics.final_formation_error);
fprintf('Final localization error: %.5f\n', result.summary.metrics.final_localization_error);
fprintf('Max |v|=%.3f m/s  Max |omega|=%.3f rad/s\n', ...
    result.summary.metrics.max_commanded_linear_velocity, ...
    result.summary.metrics.max_commanded_angular_velocity);
if result.summary.validation.bound_applicable && ~isempty(result.summary.validation.inside_bound)
    fprintf('Inside theorem bound: %d (epsilon=%.4f)\n', ...
        result.summary.validation.inside_bound, result.summary.validation.epsilon);
end
end


function save_turtlebot_plots(result, cfg)
phi = [cos(2 * pi * (0:(cfg.n - 1))' / cfg.n), sin(2 * pi * (0:(cfg.n - 1))' / cfg.n)];

fig = figure('Visible', 'off');
hold on
for i = 1:cfg.n
    xy = squeeze(result.control_points(i, :, :))';
    plot(xy(:, 1), xy(:, 2), 'LineWidth', 1.0);
    plot(xy(1, 1), xy(1, 2), 'o', 'MarkerSize', 5);
    plot(xy(end, 1), xy(end, 2), 's', 'MarkerSize', 6);
end
final_centroid = result.centroid(end, :);
circle = final_centroid + cfg.R * phi;
circle = [circle; circle(1, :)];
plot(circle(:, 1), circle(:, 2), 'k:', 'LineWidth', 1.2);
plot(result.centroid(:, 1), result.centroid(:, 2), 'k-', 'LineWidth', 1.0);
plot(cfg.source(1), cfg.source(2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
plot(final_centroid(1), final_centroid(2), 'kx', 'MarkerSize', 10, 'LineWidth', 1.5);
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
title(sprintf('TurtleBot trajectories (%s)', cfg.topology_name));
saveas(fig, fullfile(cfg.run_dir, 'trajectory.png')); close(fig);

save_series(result.times, result.formation_error, 'Formation error', ...
    'formation error', [], fullfile(cfg.run_dir, 'formation_error.png'));
epsilon = [];
if result.summary.validation.bound_applicable && ~isempty(result.summary.validation.epsilon)
    epsilon = result.summary.validation.epsilon;
end
save_series(result.times, result.localization_error, 'Localization error', ...
    '||centroid - source||', epsilon, fullfile(cfg.run_dir, 'localization_error.png'));
end


function save_series(times, values, ttl, ylab, bound, path)
fig = figure('Visible', 'off');
plot(times, values, 'LineWidth', 1.2); hold on
if ~isempty(bound)
    yline(bound, 'r--', 'LineWidth', 1.0); legend({'error', 'bound'}, 'Location', 'best');
end
grid on; xlabel('time [s]'); ylabel(ylab); title(ttl);
saveas(fig, path); close(fig);
end


function save_turtlebot_summary(result, cfg)
summary = result.summary;
save(fullfile(cfg.run_dir, 'result.mat'), 'result', 'cfg');
try
    text = jsonencode(summary, PrettyPrint=true);
catch
    text = jsonencode(summary);
end
fid = fopen(fullfile(cfg.run_dir, 'summary.json'), 'w');
if fid < 0
    error('Could not open summary.json for writing.');
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', text);
end
