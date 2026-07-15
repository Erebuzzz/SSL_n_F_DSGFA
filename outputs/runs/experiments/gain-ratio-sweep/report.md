# Experiment Suite: gain-ratio-sweep

## Summary

Total runs: 5

| variant | noise | topology | final localization | epsilon | inside bound | localization entry | tail loc span |
|---|---|---|---:|---:|---|---:|---:|
| ratio_1000 | bounded | paper_fig1_reconstructed | 3.8974416561999843 | 0.1 | False | None | 0.196466021554643 |
| ratio_1500 | bounded | paper_fig1_reconstructed | 3.8981237663893085 | 0.1 | False | None | 0.1945110670338419 |
| ratio_1730.4 | bounded | paper_fig1_reconstructed | 3.89493541510589 | 0.1 | False | None | 0.1934440549804548 |
| ratio_2000 | bounded | paper_fig1_reconstructed | 3.8967092501395375 | 0.1 | False | None | 0.19366124823230857 |
| ratio_2500 | bounded | paper_fig1_reconstructed | 3.8965935168196397 | 0.1 | False | None | 0.19500103168780125 |

## Interpretation Notes

- Bounded-noise rows are theorem-valid when `bound_applicable` is true.
- Gaussian rows are paper-similarity rows because Gaussian noise is not strictly bounded.
- `localization_entry_time` is the first time the localization error enters the reported threshold.
- `time_inside_localization_threshold_after_entry` measures how persistently the run stays within that threshold after first entry.
