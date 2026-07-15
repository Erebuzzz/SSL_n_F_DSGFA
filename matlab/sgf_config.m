function cfg = sgf_config(config_path)
%SGF_CONFIG Parse a shared JSON config for the MATLAB parity simulator.

raw_text = fileread(config_path);
raw = jsondecode(raw_text);

mode = string(raw.experiment.mode);
if mode ~= "single_integrator"
    error('Mode %s is planned but not implemented in MATLAB Phase 1.3.', mode);
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
% Optional custom initial positions. Absent -> fixed default layout (n = 6).
% A JSON list of [x, y] pairs decodes to an n-by-2 matrix via jsondecode.
if isfield(raw, 'initial_conditions') && isfield(raw.initial_conditions, 'positions')
    cfg.initial_positions = double(raw.initial_conditions.positions);
else
    cfg.initial_positions = default_initial_positions(cfg.n, cfg.source);
end
% Optional per-robot sensing-capability mask (1 = informed-capable, 0 = forced
% blind). Absent -> all capable. Effective informed status is still gated by Dmax.
if isfield(raw, 'initial_conditions') && isfield(raw.initial_conditions, 'informed')
    cfg.informed_mask = logical(raw.initial_conditions.informed(:));
else
    cfg.informed_mask = true(cfg.n, 1);
end

cfg.output_folder = char(string(raw.outputs.folder));
if isfield(raw.outputs, 'run_id')
    cfg.run_id = char(string(raw.outputs.run_id) + "_matlab");
else
    cfg.run_id = char(cfg.name + "_matlab");
end
cfg.run_dir = fullfile(cfg.output_folder, cfg.run_id);
cfg.save_plots = logical(raw.outputs.save_plots);
cfg.save_animation = logical(raw.outputs.save_animation);
if isfield(raw.outputs, 'animation_fps')
    cfg.animation_fps = double(raw.outputs.animation_fps);
else
    cfg.animation_fps = 20;
end

validate_config(cfg);
end

function positions = default_initial_positions(n, source)
% Deterministic six-robot layout, shifted to keep the paper geometry relative to
% the source so every robot starts within Dmax (source = [5.5 5.5] reproduces the
% original layout exactly).
if n ~= 6
    error('Default initial positions are defined for n = 6. Provide custom support before changing n.');
end
if nargin < 2 || isempty(source)
    source = [5.5, 5.5];
end
base = [
    0.0, 0.0;
    -2.5, -0.5;
    5.0, 1.0;
    0.5, 3.5;
    1.0, 4.0;
    6.0, 2.0
];
shift = source(:)' - [5.5, 5.5];
positions = base + shift;
end

function validate_config(cfg)
if cfg.n <= 2
    error('n must be greater than 2.');
end
if cfg.R <= 0 || cfg.Dmax <= 0 || cfg.R > cfg.Dmax
    error('R and Dmax must be positive and satisfy R <= Dmax.');
end
if cfg.alpha <= 0 || cfg.beta <= 0
    error('alpha and beta must be positive.');
end
if cfg.duration <= 0 || cfg.dt <= 0
    error('duration and dt must be positive.');
end
if ~ismember(cfg.noise_model, ["none", "gaussian", "bounded"])
    error('noise.model must be none, gaussian, or bounded.');
end
if any(size(cfg.adjacency) ~= [cfg.n, cfg.n])
    error('adjacency must have shape n by n.');
end
if any(size(cfg.initial_positions) ~= [cfg.n, 2])
    error('initial_positions must have shape n by 2.');
end
if numel(cfg.informed_mask) ~= cfg.n
    error('informed must have n elements.');
end
if any(diag(cfg.adjacency) ~= 0) || any(any(cfg.adjacency ~= cfg.adjacency'))
    error('adjacency must be undirected with a zero diagonal.');
end
end
