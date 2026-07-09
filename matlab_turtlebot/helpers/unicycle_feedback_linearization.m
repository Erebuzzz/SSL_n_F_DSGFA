function [v, omega] = unicycle_feedback_linearization(commands, headings, offset)
%UNICYCLE_FEEDBACK_LINEARIZATION Convert single-integrator commands to (v, omega).
%
%   [v, omega] = UNICYCLE_FEEDBACK_LINEARIZATION(commands, headings, offset)
%   inverts the control-point map s_i = p_i + r[cos theta; sin theta]:
%
%       [v; omega] = [ cos,      sin;
%                     -sin/r,    cos/r ] * f_i
%
%   commands : (n,2) desired control-point velocities f_i
%   headings : (n,1) robot headings theta_i
%   offset   : feedback-linearization shift r > 0 (singular at r = 0)

if offset <= 0
    error('control-point offset r must be positive (map is singular at r=0).');
end
c = cos(headings(:));
s = sin(headings(:));
fx = commands(:, 1);
fy = commands(:, 2);
v = c .* fx + s .* fy;
omega = (-s .* fx + c .* fy) / offset;
end
