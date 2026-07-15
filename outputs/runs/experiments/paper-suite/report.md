# Experiment Suite: paper-suite

## Summary

Total runs: 2

| variant | noise | topology | final localization | epsilon | inside bound | localization entry | tail loc span |
|---|---|---|---:|---:|---|---:|---:|
| gaussian_similarity | gaussian | paper_fig1_reconstructed | 0.025172537833336117 | 0.1 | None | 39.9295 | 0.03927572617817987 |
| bounded_theorem | bounded | paper_fig1_reconstructed | 0.02819648421304502 | 0.1 | True | 39.918 | 0.03737937151409838 |

## Interpretation Notes

- Bounded-noise rows are theorem-valid when `bound_applicable` is true.
- Gaussian rows are paper-similarity rows because Gaussian noise is not strictly bounded.
- `localization_entry_time` is the first time the localization error enters the reported threshold.
- `time_inside_localization_threshold_after_entry` measures how persistently the run stays within that threshold after first entry.
