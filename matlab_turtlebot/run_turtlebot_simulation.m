function result = run_turtlebot_simulation(cfg)
%RUN_TURTLEBOT_SIMULATION Numerical differential-drive TurtleBot swarm sim.
%
%   Applies the paper's Eq. 4 sign gradient-free controller to TurtleBot-style
%   unicycle robots via point-offset feedback linearization, with sampled command
%   updates (command_period) and optional actuator saturation. Reuses the Phase
%   1.3 helpers sgf_control / sgf_measurement / sgf_theory_bounds so the control
%   math is single-sourced with the numerical simulator.
%
%   Metrics are computed on the control points s_i = p_i + r[cos theta; sin theta]
%   (the states the single-integrator law governs), matching Python Phase 2.

rng(cfg.seed, 'twister');

params = struct('wheel_radius', cfg.wheel_radius, 'wheel_separation', cfg.wheel_separation);

steps = round(cfg.duration / cfg.dt);
times = (0:steps)' * cfg.dt;
n = cfg.n;

positions = zeros(n, 2, steps + 1);   % robot centers
headings = zeros(n, steps + 1);
control_pts = zeros(n, 2, steps + 1); % s_i
centroid = zeros(steps + 1, 2);
formation_error = zeros(steps + 1, 1);
localization_error = zeros(steps + 1, 1);
n_informed = zeros(steps + 1, 1);
commanded_v = zeros(n, steps);
commanded_omega = zeros(n, steps);

positions(:, :, 1) = cfg.initial_positions;
headings(:, 1) = cfg.initial_headings;
phi = formation_slots(n);

hold_steps = max(1, round(cfg.command_period / cfg.dt));
v = zeros(n, 1);
omega = zeros(n, 1);

for step = 1:(steps + 1)
    p = positions(:, :, step);
    theta = headings(:, step);
    s = p + cfg.offset * [cos(theta), sin(theta)];

    control_pts(:, :, step) = s;
    centroid(step, :) = mean(s, 1);
    formation_error(step) = formation_error_value(s, cfg.R, phi);
    localization_error(step) = norm(centroid(step, :) - cfg.source);

    if mod(step - 1, hold_steps) == 0
        [sigma, informed] = sgf_measurement(s, cfg);
        n_informed(step) = sum(informed);
        f = sgf_control(s, cfg.adjacency, phi, sigma, cfg);
        [v, omega] = unicycle_feedback_linearization(f, theta, cfg.offset);
        [v, omega] = clip_commands(v, omega, cfg.max_linear_velocity, cfg.max_angular_velocity);
    else
        distances = sqrt(sum((s - cfg.source) .^ 2, 2));
        n_informed(step) = sum((distances < cfg.Dmax) & cfg.informed_mask);
    end

    if step == steps + 1
        break
    end

    commanded_v(:, step) = v;
    commanded_omega(:, step) = omega;
    [p_next, theta_next] = differential_drive_step(p, theta, v, omega, cfg.dt, params);
    positions(:, :, step + 1) = p_next;
    headings(:, step + 1) = theta_next;
end

if min(n_informed) == 0
    warning(['No robot is ever within Dmax of the source (0 informed robots): ' ...
        'there is no source signal, so the centroid cannot localize. Move the ' ...
        'source closer, raise Dmax, or start the robots near the source.']);
end
bounds = sgf_theory_bounds(cfg, min(n_informed));
summary = build_summary(cfg, formation_error, localization_error, n_informed, ...
    commanded_v, commanded_omega, bounds);

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


function summary = build_summary(cfg, formation_error, localization_error, n_informed, cmd_v, cmd_omega, bounds)
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
summary.model = "turtlebot";
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
