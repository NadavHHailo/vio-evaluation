# VIO comparison — Evaluation-DR metrics (align=se3, tag=baseline_x86)

Aggregated mean ± std over reps. Accuracy via `ov_eval error_singlerun`; latency/FPS from per-frame timing; CPU/RSS from `/usr/bin/time -v`. ORB-SLAM3 runs in **sequential** mode, reported as **(SLAM)** (loop closure on) and **(VIO-only)** (`loopClosing:0`). **x86 performance figures are illustrative** (DR: perf belongs on embedded HW); ORB-SLAM3's backend (local BA) is async, so latency/FPS reflect the per-frame tracking front-end.

## §3.1 Summary (RPE columns = mean over segment lengths)

| System | Seq | ATE-t (m) | ATE-r (°) | RPE-t (m) | RPE-r (°) | Compl % | Trk-loss | Lat p50/p99 (ms) | FPS | CPU % | RSS (MB) | reps |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.113 ± 0.000 | 1.17 ± 0.00 | 0.160 ± 0.043 | 1.19 ± 0.43 | — | — | — | — | — | — | 1 |
| orb_slam3 (SLAM) | V1_01_easy | 0.020 ± 0.000 | 0.41 ± 0.00 | 0.029 ± 0.002 | 0.42 ± 0.02 | 96.5 | 0.0 | 28.3/37.6 | 35.4 | 324 | 805 | 1 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.019 ± 0.000 | 0.43 ± 0.00 | 0.029 ± 0.001 | 0.43 ± 0.01 | 96.5 | 0.0 | 33.4/45.7 | 30.0 | 334 | 814 | 1 |
| openvins | MH_03_medium | — | — | — | — | — | — | — | — | — | — | 1 |
| orb_slam3 (SLAM) | MH_03_medium | 0.025 ± 0.000 | 1.10 ± 0.00 | 0.086 ± 0.018 | 0.53 ± 0.23 | 86.3 | 0.0 | 29.5/40.1 | 34.3 | 330 | 836 | 1 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.026 ± 0.000 | 1.15 ± 0.00 | 0.093 ± 0.021 | 0.56 ± 0.24 | 86.4 | 0.0 | 34.9/48.7 | 29.0 | 336 | 837 | 1 |
| openvins | V2_02_medium | — | — | — | — | — | — | — | — | — | — | 1 |
| orb_slam3 (SLAM) | V2_02_medium | 0.019 ± 0.000 | 0.85 ± 0.00 | 0.036 ± 0.005 | 1.11 ± 0.05 | 96.9 | 0.0 | 28.9/37.7 | 35.1 | 336 | 833 | 1 |
| orb_slam3 (VIO-only) | V2_02_medium | 0.022 ± 0.000 | 0.87 ± 0.00 | 0.038 ± 0.003 | 1.10 ± 0.06 | 96.9 | 0.0 | 33.9/44.0 | 29.9 | 343 | 829 | 1 |

## §2.6 RPE over segment lengths — translation (m)

Local drift accumulated over fixed sub-trajectory lengths (the standard VIO drift-rate-over-distance view). Each cell is the mean over reps of `ov_eval`'s per-segment median translation error.

| System | Seq | 8 m | 16 m | 24 m | 32 m | 40 m |
|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.111 | 0.127 | 0.162 | 0.183 | 0.219 |
| orb_slam3 (SLAM) | V1_01_easy | 0.030 | 0.028 | 0.030 | 0.025 | 0.031 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.029 | 0.030 | 0.028 | 0.028 | 0.030 |
| openvins | MH_03_medium | — | — | — | — | — |
| orb_slam3 (SLAM) | MH_03_medium | 0.090 | 0.062 | 0.081 | 0.112 | 0.083 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.096 | 0.063 | 0.090 | 0.123 | 0.092 |
| openvins | V2_02_medium | — | — | — | — | — |
| orb_slam3 (SLAM) | V2_02_medium | 0.036 | 0.041 | 0.029 | 0.034 | 0.038 |
| orb_slam3 (VIO-only) | V2_02_medium | 0.034 | 0.041 | 0.034 | 0.039 | 0.040 |

## §2.6 RPE over segment lengths — rotation (°)

| System | Seq | 8 m | 16 m | 24 m | 32 m | 40 m |
|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.67 | 0.91 | 1.15 | 1.48 | 1.73 |
| orb_slam3 (SLAM) | V1_01_easy | 0.45 | 0.40 | 0.44 | 0.43 | 0.40 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.45 | 0.42 | 0.43 | 0.44 | 0.41 |
| openvins | MH_03_medium | — | — | — | — | — |
| orb_slam3 (SLAM) | MH_03_medium | 0.23 | 0.38 | 0.53 | 0.68 | 0.81 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.23 | 0.42 | 0.58 | 0.72 | 0.84 |
| openvins | V2_02_medium | — | — | — | — | — |
| orb_slam3 (SLAM) | V2_02_medium | 1.16 | 1.06 | 1.12 | 1.15 | 1.06 |
| orb_slam3 (VIO-only) | V2_02_medium | 1.11 | 1.03 | 1.12 | 1.17 | 1.05 |
