function [pos_next, theta_next, wheel_speeds] = differential_drive_step(pos, theta, v, omega, dt, params)
%DIFFERENTIAL_DRIVE_STEP Advance TurtleBot poses one step via unicycle kinematics.
%
%   Body commands (v, omega) are converted to left/right wheel angular speeds
%   (reported for inspection / a hardware-facing layer) and the pose is
%   integrated with an explicit Euler step:
%
%       x_next     = x + dt * v * cos(theta)
%       y_next     = y + dt * v * sin(theta)
%       theta_next = wrapToPi(theta + dt * omega)
%
%   pos    : (n,2) [x y]
%   theta  : (n,1) heading
%   v,omega: (n,1) body velocities
%   params : struct with wheel_radius, wheel_separation

half_base = 0.5 * params.wheel_separation;
right = (v(:) + omega(:) * half_base) / params.wheel_radius;
left  = (v(:) - omega(:) * half_base) / params.wheel_radius;
wheel_speeds = [left, right];

pos_next = pos + dt * [v(:) .* cos(theta(:)), v(:) .* sin(theta(:))];
theta_next = wrap_to_pi(theta(:) + dt * omega(:));
end

function a = wrap_to_pi(a)
a = mod(a + pi, 2 * pi) - pi;
end
