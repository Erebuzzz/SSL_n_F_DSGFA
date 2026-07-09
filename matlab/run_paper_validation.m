function results = run_paper_validation()
%RUN_PAPER_VALIDATION Run Gaussian and bounded shared validation configs.

results = struct();
results.gaussian_similarity = run_from_config(fullfile('configs', 'paper_default.json'));
results.bounded_theorem_check = run_from_config(fullfile('configs', 'paper_bounded_validation.json'));
end
