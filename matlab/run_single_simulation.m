function result = run_single_simulation()
%RUN_SINGLE_SIMULATION Run the default shared paper-like config.

result = run_from_config(fullfile('configs', 'paper_default.json'));
end
