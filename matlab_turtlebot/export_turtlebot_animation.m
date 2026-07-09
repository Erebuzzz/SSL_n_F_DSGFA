function export_turtlebot_animation(result, cfg)
%EXPORT_TURTLEBOT_ANIMATION Write a synchronized 4-panel motion GIF.
%
%   Panels: (left) TurtleBot formation + heading arrows + target circle +
%   centroid + source; (top-right) formation error vs time; (bottom-right)
%   localization error vs time with the epsilon bound. A time-cursor tracks all
%   panels together.

gif_path = fullfile(cfg.run_dir, 'motion.gif');
n_frames = numel(result.times);
stride = max(1, floor(n_frames / 240));
frames = 1:stride:n_frames;

phi = [cos(2 * pi * (0:(cfg.n - 1))' / cfg.n), sin(2 * pi * (0:(cfg.n - 1))' / cfg.n)];
epsilon = [];
if result.summary.validation.bound_applicable && ~isempty(result.summary.validation.epsilon)
    epsilon = result.summary.validation.epsilon;
end

all_xy = reshape(permute(result.control_points, [3 1 2]), [], 2);
pad = 1.0;
xl = [min([all_xy(:,1); cfg.source(1)]) - pad, max([all_xy(:,1); cfg.source(1)]) + pad];
yl = [min([all_xy(:,2); cfg.source(2)]) - pad, max([all_xy(:,2); cfg.source(2)]) + pad];

meta = sprintf('TurtleBot  topology=%s  alpha/beta=%g  T=%.3gs', ...
    cfg.topology_name, cfg.alpha / cfg.beta, cfg.command_period);

fig = figure('Visible', 'off', 'Position', [100, 100, 1000, 500]);
first = true;
for idx = frames
    clf(fig);

    subplot(1, 2, 1); hold on
    for i = 1:cfg.n
        xi = reshape(result.control_points(i, 1, 1:idx), [], 1);
        yi = reshape(result.control_points(i, 2, 1:idx), [], 1);
        plot(xi, yi, 'LineWidth', 0.7);
    end
    s = result.control_points(:, :, idx);
    th = result.headings(:, idx);
    quiver(s(:, 1), s(:, 2), cos(th), sin(th), 0.4, 'b');
    plot(s(:, 1), s(:, 2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 5);
    c = result.centroid(idx, :);
    circle = c + cfg.R * phi; circle = [circle; circle(1, :)];
    plot(circle(:, 1), circle(:, 2), 'k:', 'LineWidth', 1.2);
    plot(c(1), c(2), 'kx', 'MarkerSize', 10, 'LineWidth', 1.5);
    plot(cfg.source(1), cfg.source(2), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');
    xlim(xl); ylim(yl); axis equal; grid on
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('TurtleBots and source (t = %.2f s)', result.times(idx)));

    subplot(2, 2, 2);
    plot(result.times, result.formation_error, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8); hold on
    plot(result.times(1:idx), result.formation_error(1:idx), 'b', 'LineWidth', 1.2);
    plot(result.times(idx), result.formation_error(idx), 'ro', 'MarkerFaceColor', 'r');
    grid on; ylabel('formation error'); title('Formation error');

    subplot(2, 2, 4);
    plot(result.times, result.localization_error, 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8); hold on
    plot(result.times(1:idx), result.localization_error(1:idx), 'b', 'LineWidth', 1.2);
    plot(result.times(idx), result.localization_error(idx), 'ro', 'MarkerFaceColor', 'r');
    if ~isempty(epsilon)
        yline(epsilon, 'r--', 'LineWidth', 1.0);
    end
    grid on; xlabel('time [s]'); ylabel('||centroid - source||'); title('Localization error');

    sgtitle(meta);

    frame = getframe(fig);
    [im, cmap] = rgb2ind(frame2im(frame), 256);
    if first
        imwrite(im, cmap, gif_path, 'gif', 'LoopCount', inf, 'DelayTime', 1 / cfg.animation_fps);
        first = false;
    else
        imwrite(im, cmap, gif_path, 'gif', 'WriteMode', 'append', 'DelayTime', 1 / cfg.animation_fps);
    end
end
close(fig);
end
