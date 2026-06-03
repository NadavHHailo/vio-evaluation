# VIO comparison — Evaluation-DR metrics (align=se3, tag=baseline_x86)

Metric definitions, targets, and methodology follow the **[VIO and SLAM — Evaluation DR](https://hailotech.atlassian.net/wiki/spaces/PhysicalAI/pages/3270180866/VIO+and+SLAM+-+Evaluation+DR)** (§ references throughout).

Aggregated mean ± std over reps. Accuracy (ATE/RPE) via **[evo](https://github.com/MichaelGrupp/evo)** (Umeyama SE3-aligned), one uniform engine for all systems; latency/FPS from per-frame timing; CPU/RSS from `/usr/bin/time -v`. ORB-SLAM3 runs in **sequential** mode, reported as **(SLAM)** (loop closure on) and **(VIO-only)** (`loopClosing:0`). **x86 performance figures are illustrative** (DR: perf belongs on embedded HW); ORB-SLAM3's backend (local BA) is async, so latency/FPS reflect the per-frame tracking front-end. **OpenVINS runs in serial mode, 4 threads**; latency/FPS use its per-frame `total` update time, and CPU%/RSS come from `/usr/bin/time -v` — the same whole-process method as ORB-SLAM3. Caveat: OpenVINS runs via `ros2 launch` + rosbag2 (reading the `.db3`), so its CPU/RSS include that process-tree overhead, whereas ORB-SLAM3 is a bare binary reading PNGs — RSS especially is an upper bound for OpenVINS.

## §3.1 Summary (RPE columns = mean over segment lengths)

*Compl %* = poses ÷ all input frames; *Compl(p-i) %* = poses ÷ frames after the first pose (tracking continuity, excludes the VI-init warm-up); *Init (s)* = time to first pose.

| System | Seq | ATE-t (m) | ATE-r (°) | RPE-t (m) | RPE-r (°) | Compl % | Compl(p-i) % | Init (s) | Trk-loss | Lat p50/p99 (ms) | FPS | CPU % | RSS (MB) | reps |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.038 ± 0.000 | 0.53 ± 0.00 | 0.049 ± 0.005 | 0.48 ± 0.07 | 95.3 | 99.1 | 5.60 | — | 11.6/26.8 | 76.4 | 145 | 2132 | 1 |
| orb_slam3 (SLAM) | V1_01_easy | 0.021 ± 0.000 | 0.40 ± 0.00 | 0.029 ± 0.003 | 0.41 ± 0.02 | 96.5 | 100.0 | 5.15 | 0.0 | 28.7/38.7 | 35.0 | 324 | 804 | 1 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.022 ± 0.000 | 0.41 ± 0.00 | 0.030 ± 0.004 | 0.41 ± 0.03 | 96.5 | 100.0 | 5.15 | 0.0 | 28.7/38.4 | 35.0 | 324 | 798 | 1 |
| basalt | V1_01_easy | 0.030 ± 0.000 | 0.60 ± 0.00 | 0.043 ± 0.003 | 0.49 ± 0.06 | 100.0 | 100.0 | 0.00 | — | 8.7/16.7 | 113.3 | 1337 | 96 | 1 |
| openvins | MH_03_medium | 0.114 ± 0.000 | 1.18 ± 0.00 | 0.126 ± 0.017 | 0.61 ± 0.25 | 85.3 | 99.6 | 19.45 | — | 11.3/27.2 | 80.4 | 149 | 1985 | 1 |
| orb_slam3 (SLAM) | MH_03_medium | 0.027 ± 0.000 | 1.11 ± 0.00 | 0.087 ± 0.018 | 0.54 ± 0.25 | 86.4 | 100.0 | 18.30 | 0.0 | 29.8/39.6 | 34.1 | 332 | 834 | 1 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.027 ± 0.000 | 1.15 ± 0.00 | 0.090 ± 0.021 | 0.55 ± 0.26 | 86.3 | 100.0 | 18.45 | 0.0 | 29.2/39.6 | 34.6 | 328 | 828 | 1 |
| basalt | MH_03_medium | 0.061 ± 0.000 | 1.28 ± 0.00 | 0.117 ± 0.019 | 0.61 ± 0.24 | 100.0 | 100.0 | 0.00 | — | 12.0/25.5 | 80.4 | 1349 | 92 | 1 |
| openvins | V2_02_medium | 0.047 ± 0.000 | 1.20 ± 0.00 | 0.065 ± 0.010 | 1.36 ± 0.13 | 96.3 | 99.7 | 4.04 | — | 11.2/22.6 | 83.8 | 151 | 1741 | 1 |
| orb_slam3 (SLAM) | V2_02_medium | 0.024 ± 0.000 | 0.89 ± 0.00 | 0.042 ± 0.006 | 1.13 ± 0.08 | 96.9 | 100.0 | 3.60 | 0.0 | 28.6/37.1 | 35.2 | 338 | 834 | 1 |
| orb_slam3 (VIO-only) | V2_02_medium | 0.023 ± 0.000 | 0.92 ± 0.00 | 0.042 ± 0.004 | 1.11 ± 0.07 | 96.9 | 100.0 | 3.60 | 0.0 | 27.9/36.4 | 36.4 | 335 | 823 | 1 |
| basalt | V2_02_medium | 0.049 ± 0.000 | 0.81 ± 0.00 | 0.071 ± 0.012 | 1.06 ± 0.14 | 100.0 | 100.0 | 0.00 | — | 9.9/18.3 | 101.5 | 1184 | 90 | 1 |

## §2.6 RPE over segment lengths — translation (m)

Local drift accumulated over fixed sub-trajectory lengths (the standard VIO drift-rate-over-distance view). Each cell is the mean over reps of evo's all-pairs RPE median translation error at the given segment length.

| System | Seq | 8 m | 16 m | 24 m | 32 m | 40 m |
|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.057 | 0.051 | 0.048 | 0.050 | 0.042 |
| orb_slam3 (SLAM) | V1_01_easy | 0.031 | 0.031 | 0.028 | 0.024 | 0.031 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.032 | 0.030 | 0.031 | 0.025 | 0.035 |
| basalt | V1_01_easy | 0.045 | 0.046 | 0.040 | 0.039 | 0.044 |
| openvins | MH_03_medium | 0.149 | 0.106 | 0.114 | 0.128 | 0.131 |
| orb_slam3 (SLAM) | MH_03_medium | 0.093 | 0.063 | 0.087 | 0.113 | 0.077 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.091 | 0.062 | 0.088 | 0.121 | 0.086 |
| basalt | MH_03_medium | 0.106 | 0.089 | 0.121 | 0.136 | 0.132 |
| openvins | V2_02_medium | 0.047 | 0.068 | 0.070 | 0.067 | 0.072 |
| orb_slam3 (SLAM) | V2_02_medium | 0.034 | 0.044 | 0.040 | 0.042 | 0.049 |
| orb_slam3 (VIO-only) | V2_02_medium | 0.037 | 0.042 | 0.039 | 0.048 | 0.045 |
| basalt | V2_02_medium | 0.054 | 0.063 | 0.076 | 0.079 | 0.084 |

## §2.6 RPE over segment lengths — rotation (°)

| System | Seq | 8 m | 16 m | 24 m | 32 m | 40 m |
|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.53 | 0.37 | 0.45 | 0.49 | 0.55 |
| orb_slam3 (SLAM) | V1_01_easy | 0.44 | 0.40 | 0.41 | 0.41 | 0.37 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.46 | 0.40 | 0.41 | 0.42 | 0.36 |
| basalt | V1_01_easy | 0.48 | 0.41 | 0.48 | 0.58 | 0.51 |
| openvins | MH_03_medium | 0.31 | 0.46 | 0.56 | 0.76 | 0.94 |
| orb_slam3 (SLAM) | MH_03_medium | 0.22 | 0.40 | 0.55 | 0.67 | 0.87 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.23 | 0.39 | 0.55 | 0.69 | 0.89 |
| basalt | MH_03_medium | 0.29 | 0.45 | 0.65 | 0.78 | 0.89 |
| openvins | V2_02_medium | 1.20 | 1.26 | 1.35 | 1.48 | 1.50 |
| orb_slam3 (SLAM) | V2_02_medium | 1.12 | 1.00 | 1.15 | 1.16 | 1.22 |
| orb_slam3 (VIO-only) | V2_02_medium | 1.10 | 0.99 | 1.13 | 1.14 | 1.18 |
| basalt | V2_02_medium | 1.09 | 0.87 | 1.01 | 1.10 | 1.24 |

## Conclusions

_Auto-generated by `compare_report.py` from the tables above — refreshed on every run, so they never drift from the data._

Each bullet maps a finding to the relevant section of the **[Evaluation DR](https://hailotech.atlassian.net/wiki/spaces/PhysicalAI/pages/3270180866/VIO+and+SLAM+-+Evaluation+DR)** (the `§x.y` tags — e.g. §2.6 *RPE over segment lengths*, §3.1 *metric table*, §3.3 *robustness*) and checks the measured value against that section's stated target (ATE/RPE/FPS/latency/init-time thresholds), so the verdicts stay tied to the DR's definitions rather than ad-hoc judgement.

- **Accuracy (ATE).** Mean ATE-trans, best→worst: **ORB-SLAM3 0.024 m, Basalt 0.047 m, OpenVINS 0.066 m**. ORB-SLAM3 is the most accurate (~2.8× lower error than OpenVINS). All systems meet the DR ATE targets (V1_01 < 0.10 m, V2_02 < 0.20 m).
- **Loop closure (SLAM vs VIO-only).** Disabling ORB-SLAM3's loop-closure/global-BA stage shifts ATE by **at most 0.001 m — within run-to-run noise, so loop closure makes no meaningful difference here**. Consistent with the DR (§2.1/§2.7): loop closure mainly helps on **revisits**, and these EuRoC sequences are short with little re-observation, so the SLAM back-end adds little global correction.
- **Drift over distance (§2.6 RPE-vs-segment).** On V1_01 over 8→40 m segments, OpenVINS' RPE-trans falls (0.057→0.042 m, range 0.042-0.057) while ORB-SLAM3 stays roughly flat (0.031→0.031 m). ORB-SLAM3's per-segment drift is consistently lower — its local-mapping back-end bounds drift even without loop closure.
- **Robustness (§3.3).** Post-init tracking continuity ~100% with zero track-loss (raw completeness < 100% is the VI-init warm-up, not lost tracking). Init-time meets the DR < 5 s target except MH_03_medium (~19 s), V1_01_easy (~6 s).
- **Compute (§3.1, x86 — illustrative).** OpenVINS is the lighter/faster front-end (~80 FPS, ~1.5 cores) vs ORB-SLAM3 (~35 FPS, ~3.3 cores) — the classic filter-VIO vs optimization-SLAM trade-off. Basalt is the lightest on memory (~93 MB) and fastest (~98 FPS) but the most parallel (~13 cores, TBB). Both clear the DR ≥ 30 FPS bar; OpenVINS meets p99 < 33 ms (~26 ms); ORB-SLAM3 is at/above it (~38 ms). Per the DR, treat these as indicative — real perf profiling belongs on embedded HW.
- **DR coverage & gaps.** Covered: ATE t/r, RPE t/r + per-segment (§2.6), completeness, init-time, track-loss, latency p50/p99, FPS, CPU, RSS. Open items: OpenVINS RSS includes the ros2/rosbag2 process tree (upper bound); loop-closure precision/recall, map-growth, and power are not yet measured; and the last system (SchurVINS) is pending Phase 3.
