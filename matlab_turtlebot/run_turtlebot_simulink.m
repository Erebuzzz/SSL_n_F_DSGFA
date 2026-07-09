function [result, model_path] = run_turtlebot_simulink(cfg)
%RUN_TURTLEBOT_SIMULINK Build and simulate the SGF TurtleBot Simulink model.
%
%   Builds models/sgf_turtlebot_swarm.slx via build_turtlebot_simulink_model,
%   runs it through the Simulink fixed-step solver, then reconstructs the standard
%   result struct (control points, error series, commanded velocities, summary)
%   with the shared Phase 1.3 helpers so its outputs are field-compatible with
%   save_turtlebot_plots / export_turtlebot_animation.
%
%   The model is the deterministic (noise=none) continuous reference; compare its
%   metrics against a noise=none numerical unicycle / TurtleBot run.

if ~exist(cfg.run_dir, 'dir')
    mkdir(cfg.run_dir);
end

[model_path, model_name] = build_turtlebot_simulink_model(cfg, fullfile(cfg.run_dir, 'models'));

cleanup = onCleanup(@() close_if_loaded(model_name));

simout = sim(model_name, 'StopTime', num2str(cfg.duration, '%.10g'));
xout = simout.get('xout');   % (3n, 1, T): each column-vector state stacked on dim 3
tout = simout.get('tout');   % (T, 1)

n = cfg.n;
X = reshape(xout, 3 * n, []);          % (3n, T)
T = size(X, 2);

positions = zeros(n, 2, T);
positions(:, 1, :) = reshape(X(1:n, :), [n, 1, T]);
positions(:, 2, :) = reshape(X(n + 1:2 * n, :), [n, 1, T]);
headings = X(2 * n + 1:3 * n, :);      % (n, T)
times = tout(:);

result = turtlebot_result_from_trajectory(cfg, positions, headings, times, "turtlebot_simulink");
result.model_path = model_path;
end


function close_if_loaded(model_name)
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
end
