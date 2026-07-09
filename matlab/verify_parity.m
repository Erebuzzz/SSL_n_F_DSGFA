function passed = verify_parity()
%VERIFY_PARITY Check MATLAB parity against the Python golden reference.
%
%   passed = VERIFY_PARITY() runs the MATLAB parity simulator on the shared
%   deterministic (noise-free) config in matlab/parity/parity_none.json and
%   compares the full error time-series, final poses, and key summary metrics
%   against matlab/parity/golden_parity_none.json, which was produced by the
%   Python simulator (sgf_sim) with:
%
%       python matlab/parity/generate_golden.py
%
%   Because the run is noise-free, both implementations integrate the identical
%   deterministic ODE, so agreement to floating-point tolerance is expected.
%   (Noisy runs are NOT expected to match: NumPy uses PCG64 while MATLAB uses
%   the Mersenne Twister, so the same seed yields different noise streams.)
%
%   Returns true when every deviation is within tolerance.

here = fileparts(mfilename('fullpath'));
repo_root = fileparts(here);
addpath(here);

config_path = fullfile(here, 'parity', 'parity_none.json');
golden_path = fullfile(here, 'parity', 'golden_parity_none.json');

if ~isfile(golden_path)
    error(['Golden reference not found. Generate it first from the repo root:', newline, ...
        '    python matlab/parity/generate_golden.py']);
end

result = run_from_config(config_path);
golden = jsondecode(fileread(golden_path));

tol = 1e-8;

% --- shape check ------------------------------------------------------------
n_steps_matlab = numel(result.times);
if n_steps_matlab ~= golden.n_steps
    error('Step-count mismatch: MATLAB %d vs golden %d.', n_steps_matlab, golden.n_steps);
end

% --- full time-series deviations -------------------------------------------
fe_dev = max(abs(result.formation_error(:) - golden.formation_error(:)));
le_dev = max(abs(result.localization_error(:) - golden.localization_error(:)));

% --- final-state deviations -------------------------------------------------
final_positions = result.positions(:, :, end);            % (n,2)
pos_dev = max(abs(final_positions(:) - golden.final_positions(:)));
cen_dev = max(abs(result.centroid(end, :)' - golden.final_centroid(:)));

% --- scalar summary metrics -------------------------------------------------
gm = golden.summary.metrics;
gv = golden.summary.validation;
fm = result.summary.metrics;
fv = result.summary.validation;

metric_dev = max([ ...
    abs(fm.final_formation_error    - gm.final_formation_error), ...
    abs(fm.final_localization_error - gm.final_localization_error), ...
    abs(fv.gain_ratio               - gv.gain_ratio), ...
    abs(fv.epsilon                  - gv.epsilon)]);

% --- report -----------------------------------------------------------------
fprintf('\n=== Python <-> MATLAB parity check (noise-free) ===\n');
fprintf('  steps compared              : %d\n', n_steps_matlab);
fprintf('  max |formation_error| dev   : %.3e\n', fe_dev);
fprintf('  max |localization_error| dev: %.3e\n', le_dev);
fprintf('  max |final position| dev    : %.3e\n', pos_dev);
fprintf('  max |final centroid| dev    : %.3e\n', cen_dev);
fprintf('  max |summary metric| dev    : %.3e\n', metric_dev);
fprintf('  tolerance                   : %.1e\n', tol);

deviations = [fe_dev, le_dev, pos_dev, cen_dev, metric_dev];
passed = all(deviations <= tol);

if passed
    fprintf('  RESULT: PASS -- MATLAB matches the Python reference.\n\n');
else
    fprintf(2, '  RESULT: FAIL -- deviation exceeds tolerance.\n\n');
end

if nargout == 0
    clear passed
end
end
