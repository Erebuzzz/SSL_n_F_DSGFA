function controls = sgf_control(positions, adjacency, phi, sigma, cfg)
%SGF_CONTROL Compute the paper Eq. 4 controller for every robot.
%
%   The formation term uses the paper's component-wise signum by default. If the
%   optional field cfg.sign_boundary_layer > 0 is present, sgn(x) is replaced by
%   the saturated approximation sat(x / eps) = clip(x / eps, -1, 1) (a standard
%   boundary-layer sliding-mode modification). eps = 0 (default / absent) is the
%   exact paper controller, so MATLAB<->Python parity is unaffected. A positive
%   boundary layer removes the chattering that otherwise corrupts localization on
%   differential-drive robots (see matlab_turtlebot/README.md).

eps = 0.0;
if isfield(cfg, 'sign_boundary_layer') && ~isempty(cfg.sign_boundary_layer)
    eps = cfg.sign_boundary_layer;
end

z = positions - cfg.R * phi;
controls = zeros(size(positions));

for i = 1:cfg.n
    neighbors = find(adjacency(i, :) ~= 0);
    if ~isempty(neighbors)
        controls(i, :) = controls(i, :) + cfg.alpha * sum(sat_sign(z(neighbors, :) - z(i, :), eps), 1);
    end
    controls(i, :) = controls(i, :) - (2 * cfg.beta / cfg.R) * sigma(i) * phi(i, :);
end
end


function s = sat_sign(d, eps)
%SAT_SIGN Component-wise signum, or its boundary-layer saturation when eps > 0.
if eps > 0
    s = max(min(d / eps, 1.0), -1.0);
else
    s = sign(d);
end
end
