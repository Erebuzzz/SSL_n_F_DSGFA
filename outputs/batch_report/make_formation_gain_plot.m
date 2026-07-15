function make_formation_gain_plot()
%MAKE_FORMATION_GAIN_PLOT Overlay figures + entry-time metadata for the sweeps.
%
%   Produces three comparison figures under outputs/batch_report/ and a
%   sweep_meta.json with a consistent formation-entry time (first instant the
%   formation error drops below 5% of its initial value) for every sweep run:
%     formation_gain_sweep.png       single-integrator alpha sweep (fg_*)
%     formation_gain_sweep_ts.png    TurtleBot-Simulink alpha sweep (fgt_*)
%     formation_gain_beta_sweep.png  extreme beta = 10000 probe (fgb_*), log scale
%   The TS summaries omit formation_entry_time, so make_report.py reads it from
%   sweep_meta.json (same definition for both modes -> comparable).

here = fileparts(mfilename('fullpath'));
alphas = [1, 5, 20, 50, 100];
si_dirs = arrayfun(@(a) sprintf('fg_n6_a%d_srcA_none_matlab', a), alphas, 'UniformOutput', false);
ts_dirs = arrayfun(@(a) sprintf('fgt_n6_a%d_srcA_none_turtlebot', a), alphas, 'UniformOutput', false);
b_alphas = [1, 10, 100];
b_dirs = arrayfun(@(a) sprintf('fgb_n6_a%d_srcA_none_turtlebot', a), b_alphas, 'UniformOutput', false);

sweep_overlay(here, si_dirs, alphas, 0.05, 'Single-integrator', ...
    'formation_gain_sweep.png', 5, 40);
sweep_overlay(here, ts_dirs, alphas, 0.05, 'TurtleBot-Simulink', ...
    'formation_gain_sweep_ts.png', 8, 60);
beta_overlay(here, b_dirs, b_alphas, 10000, 'formation_gain_beta_sweep.png');

% --- consistent formation-entry metadata (first t with formation error < 0.1) --
meta = struct();
all_dirs = [si_dirs, ts_dirs, b_dirs];
for k = 1:numel(all_dirs)
    d = all_dirs{k};
    S = load(fullfile(here, 'runs', d, 'result.mat'));
    r = S.result;
    run_id = regexprep(d, '_(matlab|turtlebot)$', '');
    meta.(run_id) = struct( ...
        'formation_entry', first_below(r.times, r.formation_error, 0.05 * r.formation_error(1)), ...
        'final_formation', r.formation_error(end), ...
        'final_localization', r.localization_error(end));
end
fid = fopen(fullfile(here, 'sweep_meta.json'), 'w');
fprintf(fid, '%s', jsonencode(meta, 'PrettyPrint', true));
fclose(fid);
fprintf('wrote sweep_meta.json\n');
end


function sweep_overlay(here, dirs, alphas, beta, modelname, outname, fxmax, lxmax)
colors = lines(numel(alphas));
fig = figure('Visible', 'off', 'Position', [100, 100, 1200, 500]);
labels = strings(numel(alphas), 1);

subplot(1, 2, 1); hold on
for k = 1:numel(alphas)
    r = load_result(here, dirs{k});
    semilogy(r.times, max(r.formation_error, 1e-4), 'LineWidth', 1.6, 'Color', colors(k, :));
    labels(k) = sprintf('\\alpha = %d  (\\alpha/\\beta = %g)', alphas(k), alphas(k) / beta);
end
set(gca, 'YScale', 'log'); xlim([0, fxmax]); grid on
xlabel('time [s]'); ylabel('formation error  (log scale)');
title(sprintf('%s: formation error vs \\alpha', modelname));
legend(labels, 'Location', 'northeast');

subplot(1, 2, 2); hold on
for k = 1:numel(alphas)
    r = load_result(here, dirs{k});
    plot(r.times, r.localization_error, 'LineWidth', 1.6, 'Color', colors(k, :));
end
xlim([0, lxmax]); grid on
xlabel('time [s]'); ylabel('||centroid - source||');
title('Localization error (same runs)');
legend(labels, 'Location', 'northeast');

sgtitle(sprintf('%s formation-gain sweep: only \\alpha varies (\\beta = %g, n = 6)', ...
    modelname, beta));
exportgraphics(fig, fullfile(here, outname), 'Resolution', 130);
close(fig);
fprintf('wrote %s\n', outname);
end


function beta_overlay(here, dirs, alphas, beta, outname)
colors = lines(numel(alphas));
fig = figure('Visible', 'off', 'Position', [100, 100, 1200, 500]);
labels = strings(numel(alphas), 1);

subplot(1, 2, 1); hold on
for k = 1:numel(alphas)
    r = load_result(here, dirs{k});
    semilogy(r.times, max(r.formation_error, 1e-4), 'LineWidth', 1.6, 'Color', colors(k, :));
    labels(k) = sprintf('\\alpha = %d  (\\alpha/\\beta = %g)', alphas(k), alphas(k) / beta);
end
set(gca, 'YScale', 'log'); grid on
xlabel('time [s]'); ylabel('formation error  (log scale)');
title('\beta = 10000: formation error diverges');
legend(labels, 'Location', 'southeast');

subplot(1, 2, 2); hold on
for k = 1:numel(alphas)
    r = load_result(here, dirs{k});
    semilogy(r.times, max(r.localization_error, 1e-4), 'LineWidth', 1.6, 'Color', colors(k, :));
end
set(gca, 'YScale', 'log'); grid on
xlabel('time [s]'); ylabel('||centroid - source||  (log scale)');
title('Localization error also diverges');
legend(labels, 'Location', 'southeast');

sgtitle('Extreme localization gain \beta = 10000 (n = 6, Simulink): the swarm diverges');
exportgraphics(fig, fullfile(here, outname), 'Resolution', 130);
close(fig);
fprintf('wrote %s\n', outname);
end


function r = load_result(here, d)
S = load(fullfile(here, 'runs', d, 'result.mat'));
r = S.result;
end


function t = first_below(times, series, thr)
idx = find(series < thr, 1, 'first');
if isempty(idx)
    t = NaN;
else
    t = times(idx);
end
end
