function cfg = tb_config(config_path)
%TB_CONFIG Parse a shared JSON config for the MATLAB TurtleBot simulator.
%
%   Accepts the turtlebot / unicycle modes (unlike matlab/sgf_config.m, which is
%   single-integrator only) and builds a cfg struct that is field-compatible with
%   the reused Phase 1.3 helpers (sgf_control, sgf_measurement, sgf_topology,
%   sgf_theory_bounds) plus the TurtleBot differential-drive fields.

raw = jsondecode(fileread(config_path));

mode = string(raw.experiment.mode);
accepted = ["turtlebot_simulink", "turtlebot_numeric", "unicycle"];
if ~ismember(mode, accepted)
    error('tb_config supports modes %s; got "%s".', strjoin(accepted, ", "), mode);
end

cfg = struct();
cfg.config_path = config_path;
cfg.name = string(raw.experiment.name);
cfg.mode = mode;
cfg.seed = double(raw.experiment.seed);
cfg.duration = double(raw.experiment.duration);
cfg.dt = double(raw.experiment.dt);

cfg.n = double(raw.paper_parameters.n);
cfg.source = double(raw.paper_parameters.source(:))';
cfg.kappa = double(raw.paper_parameters.kappa);
cfg.R = double(raw.paper_parameters.R);
cfg.Dmax = double(raw.paper_parameters.Dmax);
cfg.alpha = double(raw.paper_parameters.alpha);
cfg.beta = double(raw.paper_parameters.beta);

cfg.noise_model = string(raw.noise.model);
cfg.noise_std = double(raw.noise.std);
cfg.noise_bound = double(raw.noise.bound);

cfg.topology_name = string(raw.topology.name);
cfg.adjacency = sgf_topology(raw.topology, cfg.n);

rm = raw.robot_model;
cfg.offset = get_field(rm, 'unicycle_shift_r', 0.5);
cfg.wheel_radius = get_field(rm, 'wheel_radius', 0.033);
cfg.wheel_separation = get_field(rm, 'wheel_separation', 0.16);
cfg.max_linear_velocity = get_optional(rm, 'max_linear_velocity');
cfg.max_angular_velocity = get_optional(rm, 'max_angular_velocity');
cfg.command_period = get_field(rm, 'command_period', 0.0);  % 0 = continuous

% Optional boundary-layer sliding-mode width: sgn(x) -> clip(x/eps, +-1).
% 0 = exact paper controller. > 0 removes chattering on the differential-drive
% robot so the control point tracks the ideal single-integrator flow.
if isfield(raw, 'controller')
    cfg.sign_boundary_layer = get_field(raw.controller, 'sign_boundary_layer', 0.0);
else
    cfg.sign_boundary_layer = get_field(rm, 'sign_boundary_layer', 0.0);
end

if isfield(raw, 'simulink')
    cfg.simulink_solver = string(get_char(raw.simulink, 'solver', 'ode4'));
    cfg.simulink_model_name = string(get_char(raw.simulink, 'model_name', 'sgf_turtlebot_swarm'));
else
    cfg.simulink_solver = "ode4";
    cfg.simulink_model_name = "sgf_turtlebot_swarm";
end

% Optional custom initial positions / headings. Absent -> fixed default layout.
% A JSON list of [x, y] pairs decodes to an n-by-2 matrix; headings to a column.
if isfield(raw, 'initial_conditions') && isfield(raw.initial_conditions, 'positions')
    cfg.initial_positions = double(raw.initial_conditions.positions);
else
    cfg.initial_positions = default_initial_positions(cfg.n);
end
if isfield(raw, 'initial_conditions') && isfield(raw.initial_conditions, 'headings')
    cfg.initial_headings = double(raw.initial_conditions.headings(:));
else
    cfg.initial_headings = zeros(cfg.n, 1);
end
% Optional per-robot sensing-capability mask (1 = capable, 0 = forced blind).
if isfield(raw, 'initial_conditions') && isfield(raw.initial_conditions, 'informed')
    cfg.informed_mask = logical(raw.initial_conditions.informed(:));
else
    cfg.informed_mask = true(cfg.n, 1);
end

cfg.output_folder = char(string(raw.outputs.folder));
if isfield(raw.outputs, 'run_id')
    cfg.run_id = char(string(raw.outputs.run_id) + "_turtlebot");
else
    cfg.run_id = char(cfg.name + "_turtlebot");
end
cfg.run_dir = fullfile(cfg.output_folder, cfg.run_id);
cfg.save_plots = get_logical(raw.outputs, 'save_plots', true);
cfg.save_animation = get_logical(raw.outputs, 'save_animation', true);
cfg.animation_fps = get_field(raw.outputs, 'animation_fps', 20);

validate_cfg(cfg);
end


function value = get_field(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    value = double(s.(name));
else
    value = default;
end
end


function value = get_optional(s, name)
if isfield(s, name) && ~isempty(s.(name))
    value = double(s.(name));
else
    value = [];  % [] means "no limit"
end
end


function value = get_char(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    value = char(string(s.(name)));
else
    value = default;
end
end


function value = get_logical(s, name, default)
if isfield(s, name) && ~isempty(s.(name))
    value = logical(s.(name));
else
    value = default;
end
end


function positions = default_initial_positions(n)
if n ~= 6
    error('Default initial positions are defined for n = 6.');
end
positions = [0.0, 0.0; 2.5, -0.5; 5.0, 0.0; 0.5, 3.5; 3.0, 4.0; 5.5, 3.0];
end


function validate_cfg(cfg)
if cfg.n <= 2
    error('n must be > 2.');
end
if cfg.R <= 0 || cfg.Dmax <= 0 || cfg.R > cfg.Dmax
    error('Require 0 < R <= Dmax.');
end
if cfg.alpha <= 0 || cfg.beta <= 0
    error('alpha and beta must be positive.');
end
if cfg.dt <= 0 || cfg.duration <= 0
    error('dt and duration must be positive.');
end
if cfg.offset <= 0
    error('unicycle_shift_r must be positive (feedback linearization singular at 0).');
end
if cfg.command_period < 0
    error('command_period must be non-negative (0 = continuous).');
end
if cfg.sign_boundary_layer < 0
    error('sign_boundary_layer must be non-negative (0 = exact paper sgn).');
end
if any(size(cfg.initial_positions) ~= [cfg.n, 2])
    error('initial_positions must have shape n by 2.');
end
if numel(cfg.initial_headings) ~= cfg.n
    error('initial_headings must have n elements.');
end
if numel(cfg.informed_mask) ~= cfg.n
    error('informed must have n elements.');
end
end
