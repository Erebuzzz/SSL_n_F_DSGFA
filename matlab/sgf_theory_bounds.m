function bounds = sgf_theory_bounds(cfg, min_n_informed)
%SGF_THEORY_BOUNDS Compute theorem gain threshold and epsilon metadata.

f_dmax = cfg.kappa * cfg.Dmax * cfg.Dmax + cfg.noise_bound;
gain_threshold = 4 * cfg.n * f_dmax / cfg.R;

epsilon = [];
if min_n_informed > 0
    denominator = cfg.kappa * cfg.R * ( ...
        2 * pi * min_n_informed - cfg.n * abs(sin(2 * pi * min_n_informed / cfg.n)));
    if denominator <= 0
        error('Epsilon denominator must be positive.');
    end
    epsilon = 2 * pi * cfg.n * cfg.noise_bound / denominator;
end

bounds = struct();
bounds.gain_threshold = gain_threshold;
bounds.epsilon = epsilon;
bounds.bound_applicable = ismember(cfg.noise_model, ["bounded", "none"]) && min_n_informed > 0;
end
