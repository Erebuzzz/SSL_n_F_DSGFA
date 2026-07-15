# Experiment Suite: seed-sweep

## Summary

Total runs: 2

| variant | noise | topology | final localization | epsilon | inside bound | localization entry | tail loc span |
|---|---|---|---:|---:|---|---:|---:|
| seed_1 | gaussian | paper_fig1_reconstructed | 4.345735154742109 | 0.1 | None | None | 0.07621880521897317 |
| seed_2 | gaussian | paper_fig1_reconstructed | 4.345872747311928 | 0.1 | None | None | 0.07612184085272045 |

## Interpretation Notes

- Bounded-noise rows are theorem-valid when `bound_applicable` is true.
- Gaussian rows are paper-similarity rows because Gaussian noise is not strictly bounded.
- `localization_entry_time` is the first time the localization error enters the reported threshold.
- `time_inside_localization_threshold_after_entry` measures how persistently the run stays within that threshold after first entry.
