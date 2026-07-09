function result = turtlebot_result_from_trajectory(cfg, positions, headings, times, model_label)
%TURTLEBOT_RESULT_FROM_TRAJECTORY Build the standard result struct from a pose log.
%
%   Given a logged trajectory (robot centers + headings over time), reconstruct
%   the control points s_i = p_i + r[cos theta; sin theta], compute the formation
%   and localization error series, recover the commanded (v, omega) per step via
%   the shared Phase 1.3 helpers, and assemble the same result/summary struct that
%   run_turtlebot_simulation produces. Used by the Simulink path so its outputs are
%   field-compatible with save_turtlebot_plots / export_turtlebot_animation.
%
%   positions : (n, 2, T)  robot centers
%   headings  : (n, T)     robot headings
%   times     : (T, 1)
%   model_label : string tag stored in summary.model (e.g. "turtlebot_simulink")

if nargin < 5
    model_label = "turtlebot";
end

n = cfg.n;
T = numel(times);
phi = formation_slots(n);

control_pts = zeros(n, 2, T);
centroid = zeros(T, 2);
formation_error = zeros(T, 1);
localization_error = zeros(T, 1);
n_informed = zeros(T, 1);
commanded_v = zeros(n, T);
commanded_omega = zeros(n, T);

for step = 1:T
    p = positions(:, :, step);
    theta = headings(:, step);
    s = p + cfg.offset * [cos(theta), sin(theta)];

    control_pts(:, :, step) = s;
    centroid(step, :) = mean(s, 1);
    formation_error(step) = formation_error_value(s, cfg.R, phi);
    localization_error(step) = norm(centroid(step, :) - cfg.source);

    [sigma, informed] = sgf_measurement(s, cfg);
    n_informed(step) = sum(informed);
    f = sgf_control(s, cfg.adjacency, phi, sigma, cfg);
    [v, omega] = unicycle_feedback_linearization(f, theta, cfg.offset);
    [v, omega] = clip_commands(v, omega, cfg.max_linear_velocity, cfg.max_angular_velocity);
    commanded_v(:, step) = v;
    commanded_omega(:, step) = omega;
end

bounds = sgf_theory_bounds(cfg, min(n_informed));
summary = build_summary(cfg, formation_error, localization_error, n_informed, ...
    commanded_v, commanded_omega, bounds, model_label);

result = struct();
result.times = times;
result.positions = positions;
result.control_points = control_pts;
result.headings = headings;
result.centroid = centroid;
result.formation_error = formation_error;
result.localization_error = localization_error;
result.n_informed = n_informed;
result.commanded_v = commanded_v;
result.commanded_omega = commanded_omega;
result.summary = summary;
end


function phi = formation_slots(n)
theta = 2 * pi * (0:(n - 1))' / n;
phi = [cos(theta), sin(theta)];
end


function value = formation_error_value(s, R, phi)
z = s - R * phi;
zc = mean(z, 1);
value = sqrt(sum((z - zc) .^ 2, 'all'));
end


function [v, omega] = clip_commands(v, omega, max_v, max_omega)
if ~isempty(max_v)
    v = max(min(v, max_v), -max_v);
end
if ~isempty(max_omega)
    omega = max(min(omega, max_omega), -max_omega);
end
end


function summary = build_summary(cfg, formation_error, localization_error, n_informed, cmd_v, cmd_omega, bounds, model_label)
final_formation = formation_error(end);
final_localization = localization_error(end);
inside_bound = [];
if bounds.bound_applicable && ~isempty(bounds.epsilon)
    inside_bound = final_localization <= bounds.epsilon;
end
tail_start = max(1, floor(0.75 * numel(localization_error)));
ft = formation_error(tail_start:end);
lt = localization_error(tail_start:end);

summary = struct();
summary.model = model_label;
summary.parameters = struct('n', cfg.n, 'source', cfg.source, 'kappa', cfg.kappa, ...
    'radius', cfg.R, 'dmax', cfg.Dmax, 'alpha', cfg.alpha, 'beta', cfg.beta, ...
    'duration', cfg.duration, 'dt', cfg.dt, 'seed', cfg.seed, ...
    'noise_model', cfg.noise_model, 'noise_bound', cfg.noise_bound, ...
    'topology', cfg.topology_name, 'control_point_offset', cfg.offset, ...
    'command_period', cfg.command_period, 'wheel_radius', cfg.wheel_radius, ...
    'wheel_separation', cfg.wheel_separation, ...
    'sign_boundary_layer', cfg.sign_boundary_layer);
summary.validation = struct('gain_ratio', cfg.alpha / cfg.beta, ...
    'gain_threshold', bounds.gain_threshold, ...
    'gain_condition_passed', (cfg.alpha / cfg.beta) > bounds.gain_threshold, ...
    'min_n_informed', min(n_informed), 'final_n_informed', n_informed(end), ...
    'epsilon', bounds.epsilon, 'bound_applicable', bounds.bound_applicable, ...
    'inside_bound', inside_bound);
summary.metrics = struct('initial_formation_error', formation_error(1), ...
    'final_formation_error', final_formation, ...
    'initial_localization_error', localization_error(1), ...
    'final_localization_error', final_localization, ...
    'tail_formation_error_span', max(ft) - min(ft), ...
    'tail_localization_error_span', max(lt) - min(lt), ...
    'max_commanded_linear_velocity', max(abs(cmd_v), [], 'all'), ...
    'max_commanded_angular_velocity', max(abs(cmd_omega), [], 'all'));
end
