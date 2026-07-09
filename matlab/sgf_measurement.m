function [sigma, informed] = sgf_measurement(positions, cfg)
%SGF_MEASUREMENT Evaluate noisy saturated source measurements.

distances = sqrt(sum((positions - cfg.source) .^ 2, 2));
informed = distances < cfg.Dmax;
% A robot senses only if within range AND flagged sensing-capable. The mask is
% optional so older callers (e.g. the golden-parity harness) stay bit-identical.
if isfield(cfg, 'informed_mask')
    informed = informed & cfg.informed_mask;
end
sigma = ones(cfg.n, 1) * (cfg.kappa * cfg.Dmax * cfg.Dmax + cfg.noise_bound);

if any(informed)
    source_delta = positions(informed, :) - cfg.source;
    clean = cfg.kappa * sum(source_delta .^ 2, 2);
    sigma(informed) = clean + sample_noise(cfg, sum(informed));
end
end

function noise = sample_noise(cfg, count)
switch cfg.noise_model
    case "none"
        noise = zeros(count, 1);
    case "gaussian"
        noise = cfg.noise_std * randn(count, 1);
    case "bounded"
        noise = cfg.noise_bound * (2 * rand(count, 1) - 1);
    otherwise
        error('Unsupported noise model: %s', cfg.noise_model);
end
end
