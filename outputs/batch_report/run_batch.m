function run_batch(pattern)
%RUN_BATCH Run every batch config matching PATTERN, dispatch by mode, and
%aggregate each run's summary into outputs/batch_report/batch_summary.json.
%
%   PATTERN is a glob (without .json) inside outputs/batch_report/configs, e.g.
%   'si_n4_srcA_none' (one case), 'si_*' (all single-integrator), 'ts_*' (all
%   turtlebot_simulink). Results are written incrementally so a partial batch
%   still leaves a usable summary. Each case is wrapped in try/catch so one
%   failure does not abort the rest.

here = fileparts(mfilename('fullpath'));           % outputs/batch_report
repo_root = fileparts(fileparts(here));            % repository root
cd(repo_root);
addpath(fullfile(repo_root, 'matlab'));
addpath(fullfile(repo_root, 'matlab_turtlebot'));
addpath(fullfile(repo_root, 'matlab_turtlebot', 'helpers'));

cfg_dir = fullfile('outputs', 'batch_report', 'configs');
files = dir(fullfile(cfg_dir, [pattern '.json']));
out_json = fullfile('outputs', 'batch_report', 'batch_summary.json');

if isfile(out_json)
    agg = jsondecode(fileread(out_json));
else
    agg = struct();
end

for k = 1:numel(files)
    cfg_path = fullfile(cfg_dir, files(k).name);
    [~, run_id, ~] = fileparts(files(k).name);
    raw = jsondecode(fileread(cfg_path));
    mode = string(raw.experiment.mode);
    fprintf('=== [%d/%d] %s (%s) ===\n', k, numel(files), run_id, mode);

    rec = struct();
    rec.run_id = run_id;
    rec.mode = char(mode);
    rec.config_path = strrep(cfg_path, '\', '/');
    t0 = tic;
    try
        if mode == "turtlebot_simulink"
            res = run_turtlebot_from_config(cfg_path);
            rec.run_dir = ['outputs/batch_report/runs/' run_id '_turtlebot'];
        else
            res = run_from_config(cfg_path);
            rec.run_dir = ['outputs/batch_report/runs/' run_id '_matlab'];
        end
        rec.success = true;
        rec.error = '';
        rec.summary = res.summary;
        rec.elapsed_s = toc(t0);
        fprintf('    OK  ffE=%.4g  flE=%.4g  (%.1fs)\n', ...
            res.summary.metrics.final_formation_error, ...
            res.summary.metrics.final_localization_error, rec.elapsed_s);
    catch ME
        rec.success = false;
        rec.error = ME.message;
        rec.summary = [];
        rec.elapsed_s = toc(t0);
        fprintf('    FAIL: %s\n', ME.message);
    end

    agg.(run_id) = rec;
    fid = fopen(out_json, 'w');
    if fid > 0
        fprintf(fid, '%s', jsonencode(agg, PrettyPrint=true));
        fclose(fid);
    end
end
fprintf('BATCH DONE for pattern "%s" (%d configs)\n', pattern, numel(files));
end
