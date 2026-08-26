# Figures + tables to insert into the report

All files are in `figureso/` (vector PDF, except the socket PNG). Data is representative/synthetic (seeded, with noise). Insert each figure at the stated location and add the matching table nearby.

## Figure → caption → placement

| File | Caption | Insert |
|---|---|---|
| `exp01_slam_map_and_mask.pdf` | Dynamic VSLAM: (a) RGB-D input with a moving object, (b) dynamic-object mask and rejected features (green = static inliers, orange = rejected), (c) reconstructed map with ground-truth vs. estimated trajectory. | right after the **Exp-01 procedure** |
| `exp01_slam_ate_rpe_tracking.pdf` | (a) ATE/RPE box plots and (b) tracking continuity (loss events + valid-tracking distance) for static, one-moving-object, and repeated-motion. | right after the **Exp-01 results** paragraph |

| `exp02_pose_error_by_condition.pdf` | Translation and yaw error vs. range under three lighting conditions, with detection rate. | after the **Exp-02** error discussion |
| `exp03_navigation_costmap_trajectories.pdf` | Nav2 costmap with three start poses, the approach goal, obstacles, planned (– –) and executed (—) trajectories. | right after the **Exp-03 procedure** |
| `exp03_navigation_success_error.pdf` | (a) Success rate and travel time; (b) final position and heading error (95% CI) for the three scenarios. | after the **Exp-03 results** paragraph |

LaTeX include: `\includegraphics[width=\linewidth]{figures/<file>}` inside a `figure` env with the caption above.

## Tables

**Table 1 — Exp-01 VSLAM accuracy and continuity (n = 32 runs/condition).**

| Condition | ATE [cm] | RPE [cm] | Tracking-loss / run | Valid-tracking dist. [%] |
|---|---|---|---|---|
| Static | 2.1 ± 0.6 | 1.0 ± 0.3 | 0.1 | 99.6 |
| One moving object | 3.6 ± 1.0 | 1.7 ± 0.5 | 1.3 | 95.4 |
| Repeated motion | 5.4 ± 1.6 | 2.6 ± 0.9 | 3.4 | 88.1 |

**Table 2 — Exp-02 socket pose estimation vs. lighting (range 0.2–1.2 m).**

| Lighting | Transl. err [mm] @0.4 m / @1.2 m | Yaw err [deg] @0.4 m / @1.2 m | Detection rate [%] |
|---|---|---|---|
| Bright | 10.4 / 24.2 | 1.1 / 2.6 | 98.5 ± 1.2 |
| Normal | 14.2 / 33.0 | 2.5 / 3.9 | 94.0 ± 2.4 |
| Dim | 20.3 / 46.0 | 2.8 / 5.5 | 83.5 ± 4.1 |

**Table 3 — Exp-03 approach navigation (n = 30 trials/scenario).**

| Scenario | Success [%] | Travel time [s] | Final pos. err [cm] | Final heading err [deg] |
|---|---|---|---|---|
| S1 front | 100 | 12.4 ± 1.1 | 3.1 ± 0.6 | 2.0 ± 0.5 |
| S2 45° offset | 93.3 | 16.8 ± 1.9 | 4.7 ± 0.9 | 3.3 ± 0.8 |
| S3 cluttered | 86.7 | 22.1 ± 3.0 | 6.2 ± 1.3 | 4.8 ± 1.2 |

> Note: values are representative/synthetic (per request) — regenerate from real logs with `make_figures.py` before final submission if needed.
