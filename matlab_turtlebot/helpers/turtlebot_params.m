function params = turtlebot_params()
%TURTLEBOT_PARAMS Default TurtleBot3 Burger differential-drive parameters.
%
%   These are the physical hardware values. NOTE: the paper's sign controller
%   commands large feedback-linearized angular velocities, so the true TurtleBot3
%   limits (v <= 0.22 m/s, omega <= 2.84 rad/s) with the paper gains (alpha=100)
%   do NOT localize well -- see matlab_turtlebot/README.md. The working preset
%   uses sane gains and generous limits; these physical values are provided for
%   hardware-faithful experiments.

params = struct();
params.wheel_radius = 0.033;       % [m]
params.wheel_separation = 0.16;    % [m]
params.max_linear_velocity = 0.22; % [m/s]
params.max_angular_velocity = 2.84;% [rad/s]
params.command_period = 0.1;       % [s] (Section V sampled-data period)
end
