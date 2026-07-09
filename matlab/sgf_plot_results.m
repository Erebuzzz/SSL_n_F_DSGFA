function sgf_plot_results(result, cfg)
%SGF_PLOT_RESULTS Export MATLAB parity plots and optional synchronized animation.
%
%   Produces the same artifact set as the Python simulator so runs are directly
%   comparable:
%     trajectory.png         robot trails, final target circle, centroid path, source
%     formation_error.png    formation error vs time
%     localization_error.png localization error vs time with the epsilon bound
%     motion.gif             (optional) synchronized formation + error panels

if ~cfg.save_plots
    return
end

if ~exist(cfg.run_dir, 'dir')
    mkdir(cfg.run_dir);
end

phi = formation_slots(cfg.n);

plot_trajectory(result, cfg, phi);
plot_series(result.times, result.formation_error, 'Formation error', ...
    'formation error', [], fullfile(cfg.run_dir, 'formation_error.png'));
plot_formation_error_per_robot(result, cfg, phi);

epsilon = localization_bound(result);
plot_series(result.times, result.localization_error, 'Localization error', ...
    '||centroid - source||', epsilon, fullfile(cfg.run_dir, 'localization_error.png'));

if cfg.save_animation
    write_animation(result, cfg, phi, epsilon);
end
end


function phi = formation_slots(n)
theta = 2 * pi * (0:(n - 1))' / n;
phi = [cos(theta), sin(theta)];
end


function epsilon = localization_bound(result)
epsilon = [];
if isfield(result.summary.validation, 'bound_applicable') && ...
        result.summary.validation.bound_applicable && ...
        ~isempty(result.summary.validation.epsilon)
    epsilon = result.summary.validation.epsilon;
end
end


function plot_trajectory(result, cfg, phi)
fig = figure('Visible', 'off');
hold on
for i = 1:cfg.n
    xy = squeeze(result.positions(i, :, :))';
    plot(xy(:, 1), xy(:, 2), 'LineWidth', 1.0);
    plot(xy(1, 1), xy(1, 2), 'o', 'MarkerSize', 5);
    plot(xy(end, 1), xy(end, 2), 's', 'MarkerSize', 6);
end

final_centroid = result.centroid(end, :);
circle = final_centroid + cfg.R * phi;
circle = [circle; circle(1, :)];
plot(circle(:, 1), circle(:, 2), 'k:', 'LineWidth', 1.2);
plot(result.centroid(:, 1), result.centroid(:, 2), 'k-', 'LineWidth', 1.0);
plot(cfg.source(1), cfg.source(2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
plot(final_centroid(1), final_centroid(2), 'kx', 'MarkerSize', 10, 'LineWidth', 1.5);

axis equal
grid on
xlabel('x [m]');
ylabel('y [m]');
title(sprintf('Robot trajectories (MATLAB, %s)', cfg.topology_name));
saveas(fig, fullfile(cfg.run_dir, 'trajectory.png'));
close(fig);
end


function plot_formation_error_per_robot(result, cfg, phi)
%PLOT_FORMATION_ERROR_PER_ROBOT Per-robot formation error e_i = ||z_i - z*||
% (paper Fig. 3). z_i = p_i - R*phi_i is the shifted state; z* = mean_j z_j is
% the shifted centroid. The aggregate formation error equals the L2 norm of these
% per-robot curves stacked. result.positions is (n, 2, steps+1).
T = numel(result.times);
z = result.positions - cfg.R * phi;            % (n,2,T) via implicit expansion
z_star = mean(z, 1);                           % (1,2,T)
diff = z - z_star;                             % (n,2,T)
e = squeeze(sqrt(sum(diff .^ 2, 2)));          % (n,T)
if T == 1
    e = e(:);
end

fig = figure('Visible', 'off');
hold on
for i = 1:cfg.n
    plot(result.times, e(i, :), 'LineWidth', 1.0);
end
grid on
xlabel('time [s]');
ylabel('||z_i - z*||');
title('Per-robot formation error (paper Fig. 3)');
legend(arrayfun(@(i) sprintf('robot %d', i - 1), 1:cfg.n, 'UniformOutput', false), ...
    'Location', 'best', 'FontSize', 7);
saveas(fig, fullfile(cfg.run_dir, 'formation_error_per_robot.png'));
close(fig);
end


function plot_series(times, values, ttl, ylab, bound, path)
fig = figure('Visible', 'off');
plot(times, values, 'LineWidth', 1.2);
hold on
if ~isempty(bound)
    yline(bound, 'r--', 'LineWidth', 1.0);
    legend({'error', 'theoretical bound'}, 'Location', 'best');
end
grid on
xlabel('time [s]');
ylabel(ylab);
title(ttl);
saveas(fig, path);
close(fig);
end


function write_animation(result, cfg, phi, epsilon)
gif_path = fullfile(cfg.run_dir, 'motion.gif');
n_frames = numel(result.times);
stride = max(1, floor(n_frames / 240));
frames = 1:stride:n_frames;

meta = sprintf('MATLAB  topology=%s  alpha/beta=%g  n=%d', ...
    cfg.topology_name, result.summary.validation.gain_ratio, cfg.n);

all_xy = reshape(permute(result.positions, [3 1 2]), [], 2);
pad = 1.0;
xlims = [min([all_xy(:, 1); cfg.source(1)]) - pad, max([all_xy(:, 1); cfg.source(1)]) + pad];
ylims = [min([all_xy(:, 2); cfg.source(2)]) - pad, max([all_xy(:, 2); cfg.source(2)]) + pad];

fig = figure('Visible', 'off', 'Position', [100, 100, 1000, 500]);

first = true;
for idx = frames
    clf(fig);

    % --- left: formation scene ---------------------------------------------
    subplot(1, 2, 1);
    hold on
    for i = 1:cfg.n
        xi = reshape(result.positions(i, 1, 1:idx), [], 1);
        yi = reshape(result.positions(i, 2, 1:idx), [], 1);
        plot(xi, yi, 'LineWidth', 0.7);
    end
    current = result.positions(:, :, idx);
    plot(current(:, 1), current(:, 2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 5);
    centroid = result.centroid(idx, :);
    circle = centroid + cfg.R * phi;
    circle = [circle; circle(1, :)];
    plot(circle(:, 1), circle(:, 2), 'k:', 'LineWidth', 1.2);
    plot(centroid(1), centroid(2), 'kx', 'MarkerSize', 10, 'LineWidth', 1.5);
    plot(cfg.source(1), cfg.source(2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
    xlim(xlims); ylim(ylims);
    axis equal
    grid on
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Formation and source (t = %.2f s)', result.times(idx)));

    % --- right top: formation error ----------------------------------------
    subplot(2, 2, 2);
    plot(result.times, result.formation_error, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
    hold on
    plot(result.times(1:idx), result.formation_error(1:idx), 'b', 'LineWidth', 1.2);
    plot(result.times(idx), result.formation_error(idx), 'ro', 'MarkerFaceColor', 'r');
    grid on
    ylabel('formation error');
    title('Formation error');

    % --- right bottom: localization error ----------------------------------
    subplot(2, 2, 4);
    plot(result.times, result.localization_error, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
    hold on
    plot(result.times(1:idx), result.localization_error(1:idx), 'b', 'LineWidth', 1.2);
    plot(result.times(idx), result.localization_error(idx), 'ro', 'MarkerFaceColor', 'r');
    if ~isempty(epsilon)
        yline(epsilon, 'r--', 'LineWidth', 1.0);
    end
    grid on
    xlabel('time [s]'); ylabel('||centroid - source||');
    title('Localization error');

    sgtitle(meta);

    frame = getframe(fig);
    [image_data, color_map] = rgb2ind(frame2im(frame), 256);
    if first
        imwrite(image_data, color_map, gif_path, 'gif', 'LoopCount', inf, 'DelayTime', 1 / cfg.animation_fps);
        first = false;
    else
        imwrite(image_data, color_map, gif_path, 'gif', 'WriteMode', 'append', 'DelayTime', 1 / cfg.animation_fps);
    end
end

close(fig);
end
