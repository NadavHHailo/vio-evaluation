# VIO comparison — Evaluation-DR metrics (align=se3, tag=baseline_x86)

Metric definitions, targets, and methodology follow the **[VIO and SLAM — Evaluation DR](https://hailotech.atlassian.net/wiki/spaces/PhysicalAI/pages/3270180866/VIO+and+SLAM+-+Evaluation+DR)** (§ references throughout).

Aggregated mean ± std over reps. Accuracy (ATE/RPE) via **[evo](https://github.com/MichaelGrupp/evo)** (Umeyama SE3-aligned), one uniform engine for all systems; latency/FPS from per-frame timing; CPU/RSS from `/usr/bin/time -v`. ORB-SLAM3 runs in **sequential** mode, reported as **(SLAM)** (loop closure on) and **(VIO-only)** (`loopClosing:0`). **x86 performance figures are illustrative** (DR: perf belongs on embedded HW); ORB-SLAM3's backend (local BA) is async, so latency/FPS reflect the per-frame tracking front-end. **OpenVINS runs in serial mode, 4 threads**; latency/FPS use its per-frame `total` update time, and CPU%/RSS come from `/usr/bin/time -v` — the same whole-process method as ORB-SLAM3. Caveat: OpenVINS runs via `ros2 launch` + rosbag2 (reading the `.db3`), so its CPU/RSS include that process-tree overhead, whereas ORB-SLAM3 is a bare binary reading PNGs — RSS especially is an upper bound for OpenVINS.

## Dataset previews (camera view)

The left-camera (`cam0`) image stream for each sequence, encoded to MP4 (H.264, 30 fps) straight from the dataset's PNG frames — **no IMU data**, just what the VIO front-ends see. The three EuRoC sequences below are the ones measured in the tables; `indoor_45_2_snapdragon` (UZH-FPV) is included as an additional, harder aggressive-flight preview. Videos play inline on GitHub from the [`dataset-previews` release](https://github.com/NadavHHailo/vio-evaluation/releases/tag/dataset-previews) and are also versioned in-tree under [`videos/`](../videos/) via Git LFS.

| V1_01_easy (EuRoC) | MH_03_medium (EuRoC) |
|---|---|
| <video src="https://github.com/NadavHHailo/vio-evaluation/releases/download/dataset-previews/V1_01_easy.mp4" controls width="380"></video> | <video src="https://github.com/NadavHHailo/vio-evaluation/releases/download/dataset-previews/MH_03_medium.mp4" controls width="380"></video> |
| **V2_02_medium (EuRoC)** | **indoor_45_2_snapdragon (UZH-FPV)** |
| <video src="https://github.com/NadavHHailo/vio-evaluation/releases/download/dataset-previews/V2_02_medium.mp4" controls width="380"></video> | <video src="https://github.com/NadavHHailo/vio-evaluation/releases/download/dataset-previews/indoor_45_2_snapdragon.mp4" controls width="380"></video> |

> If a cell shows a broken link instead of a player, the release assets haven't been uploaded yet — see `videos/` for the local files.

## §3.1 Summary (RPE columns = mean over segment lengths)

*Compl %* = poses ÷ all input frames; *Compl(p-i) %* = poses ÷ frames after the first pose (tracking continuity, excludes the VI-init warm-up); *Init (s)* = time to first pose.

| System | Seq | ATE-t (m) | ATE-r (°) | RPE-t (m) | RPE-r (°) | Compl % | Compl(p-i) % | Init (s) | Trk-loss | Lat p50/p99 (ms) | FPS | CPU % | RSS (MB) | reps |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.038 ± 0.000 | 0.53 ± 0.00 | 0.049 ± 0.005 | 0.48 ± 0.07 | 95.3 | 99.1 | 5.60 | — | 11.0/25.1 | 80.8 | 144 | 2131 | 5 |
| orb_slam3 (SLAM) | V1_01_easy | 0.019 ± 0.001 | 0.41 ± 0.01 | 0.027 ± 0.002 | 0.42 ± 0.03 | 96.5 | 100.0 | 5.15 | 0.0 | 27.7/36.1 | 36.3 | 322 | 807 | 5 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.020 ± 0.001 | 0.42 ± 0.01 | 0.029 ± 0.002 | 0.42 ± 0.03 | 96.5 | 100.0 | 5.15 | 0.0 | 27.9/36.8 | 35.9 | 321 | 801 | 5 |
| basalt | V1_01_easy | 0.030 ± 0.000 | 0.60 ± 0.00 | 0.043 ± 0.003 | 0.49 ± 0.06 | 100.0 | 100.0 | 0.00 | — | 9.5/18.4 | 103.8 | 1335 | 94 | 5 |
| openvins | MH_03_medium | 0.114 ± 0.000 | 1.18 ± 0.00 | 0.126 ± 0.015 | 0.61 ± 0.23 | 85.3 | 99.6 | 19.45 | — | 10.9/26.2 | 83.5 | 150 | 1984 | 5 |
| orb_slam3 (SLAM) | MH_03_medium | 0.028 ± 0.001 | 1.12 ± 0.02 | 0.088 ± 0.018 | 0.54 ± 0.23 | 86.4 | 100.0 | 18.33 | 0.0 | 28.7/38.5 | 35.3 | 329 | 834 | 5 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.028 ± 0.001 | 1.14 ± 0.01 | 0.090 ± 0.018 | 0.55 ± 0.23 | 86.4 | 100.0 | 18.40 | 0.0 | 28.4/38.7 | 35.6 | 325 | 829 | 5 |
| basalt | MH_03_medium | 0.061 ± 0.000 | 1.28 ± 0.00 | 0.117 ± 0.018 | 0.61 ± 0.22 | 100.0 | 100.0 | 0.00 | — | 11.3/22.7 | 86.9 | 1397 | 93 | 5 |
| openvins | V2_02_medium | 0.047 ± 0.000 | 1.20 ± 0.00 | 0.065 ± 0.009 | 1.36 ± 0.12 | 96.3 | 99.7 | 4.04 | — | 11.0/22.5 | 84.9 | 151 | 1741 | 5 |
| orb_slam3 (SLAM) | V2_02_medium | 0.017 ± 0.006 | 0.87 ± 0.02 | 0.032 ± 0.005 | 1.14 ± 0.09 | 96.9 | 100.0 | 3.60 | 0.0 | 28.1/36.2 | 36.1 | 336 | 828 | 5 |
| orb_slam3 (VIO-only) | V2_02_medium | 0.024 ± 0.003 | 0.89 ± 0.02 | 0.041 ± 0.004 | 1.13 ± 0.08 | 96.9 | 100.0 | 3.60 | 0.0 | 27.4/35.0 | 36.9 | 333 | 823 | 5 |
| basalt | V2_02_medium | 0.049 ± 0.000 | 0.81 ± 0.00 | 0.071 ± 0.011 | 1.06 ± 0.13 | 100.0 | 100.0 | 0.00 | — | 8.7/16.7 | 114.4 | 1199 | 92 | 5 |

## §2.6 RPE over segment lengths — translation (m)

Local drift accumulated over fixed sub-trajectory lengths (the standard VIO drift-rate-over-distance view). Each cell is the mean over reps of evo's all-pairs RPE median translation error at the given segment length.

| System | Seq | 8 m | 16 m | 24 m | 32 m | 40 m |
|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.057 | 0.051 | 0.048 | 0.050 | 0.042 |
| orb_slam3 (SLAM) | V1_01_easy | 0.029 | 0.028 | 0.027 | 0.025 | 0.028 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.030 | 0.030 | 0.029 | 0.026 | 0.032 |
| basalt | V1_01_easy | 0.045 | 0.046 | 0.040 | 0.039 | 0.044 |
| openvins | MH_03_medium | 0.149 | 0.106 | 0.114 | 0.128 | 0.131 |
| orb_slam3 (SLAM) | MH_03_medium | 0.094 | 0.063 | 0.086 | 0.117 | 0.079 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.094 | 0.064 | 0.087 | 0.119 | 0.084 |
| basalt | MH_03_medium | 0.106 | 0.089 | 0.121 | 0.136 | 0.132 |
| openvins | V2_02_medium | 0.047 | 0.068 | 0.070 | 0.067 | 0.072 |
| orb_slam3 (SLAM) | V2_02_medium | 0.031 | 0.035 | 0.028 | 0.034 | 0.032 |
| orb_slam3 (VIO-only) | V2_02_medium | 0.036 | 0.041 | 0.038 | 0.044 | 0.045 |
| basalt | V2_02_medium | 0.054 | 0.063 | 0.076 | 0.079 | 0.084 |

## §2.6 RPE over segment lengths — rotation (°)

| System | Seq | 8 m | 16 m | 24 m | 32 m | 40 m |
|---|---|---|---|---|---|---|
| openvins | V1_01_easy | 0.53 | 0.37 | 0.45 | 0.49 | 0.55 |
| orb_slam3 (SLAM) | V1_01_easy | 0.46 | 0.40 | 0.42 | 0.41 | 0.40 |
| orb_slam3 (VIO-only) | V1_01_easy | 0.46 | 0.41 | 0.43 | 0.42 | 0.40 |
| basalt | V1_01_easy | 0.48 | 0.41 | 0.48 | 0.58 | 0.51 |
| openvins | MH_03_medium | 0.31 | 0.46 | 0.56 | 0.76 | 0.94 |
| orb_slam3 (SLAM) | MH_03_medium | 0.23 | 0.39 | 0.55 | 0.68 | 0.87 |
| orb_slam3 (VIO-only) | MH_03_medium | 0.24 | 0.40 | 0.56 | 0.69 | 0.88 |
| basalt | MH_03_medium | 0.29 | 0.45 | 0.65 | 0.78 | 0.89 |
| openvins | V2_02_medium | 1.20 | 1.26 | 1.35 | 1.48 | 1.50 |
| orb_slam3 (SLAM) | V2_02_medium | 1.15 | 0.97 | 1.15 | 1.19 | 1.23 |
| orb_slam3 (VIO-only) | V2_02_medium | 1.12 | 0.99 | 1.15 | 1.18 | 1.22 |
| basalt | V2_02_medium | 1.09 | 0.87 | 1.01 | 1.10 | 1.24 |

## UZH-FPV (fisheye drone) — accuracy + robustness

Aggressive quadrotor flight on the fisheye Snapdragon rig (pipeline + per-rig calib: [docs/uzhfpv.md](uzhfpv.md)). Same evo SE3-aligned engine as the EuRoC tables. **Cov %** = fraction of estimate poses with a GT match — UZH ground truth is partial (even indoors), so ATE/RPE reflect only that subset; read ATE *together with* Cov % and completeness, never alone. Per-segment RPE is omitted here (partial GT makes fixed-length segments unreliable). ORB-SLAM3's near-zero completeness rows are genuine divergence — its small ATE there is computed over the few frames it briefly tracked.

| System | Seq | ATE-t (m) | ATE-r (°) | Compl % | Compl(p-i) % | Cov % | Lat p50/p99 (ms) | FPS | CPU % | RSS (MB) | reps |
|---|---|---|---|---|---|---|---|---|---|---|---|
| openvins | indoor_45_2_snapdragon_with_gt | 0.263 ± 0.000 | 0.94 ± 0.00 | 48.6 | 54.3 | 72.6 | 17.1/40.9 | 50.8 | 179 | 1279 | 5 |
| basalt | indoor_45_2_snapdragon_with_gt | 0.651 ± 0.000 | 2.46 ± 0.00 | 100.0 | 100.0 | 64.4 | 4.8/10.5 | 198.2 | 1127 | 80 | 5 |
| orb_slam3 stereo (SLAM) | indoor_45_2_snapdragon_with_gt | 0.043 ± 0.000 | 0.75 ± 0.00 | 4.4 | 83.3 | 24.7 | 23.0/39.3 | 42.2 | 471 | 837 | 5 |
| orb_slam3 stereo (VIO-only) | indoor_45_2_snapdragon_with_gt | 0.046 ± 0.008 | 2.68 ± 0.17 | 7.8 | 90.0 | 24.7 | 22.7/38.0 | 42.9 | 476 | 837 | 5 |
| orb_slam3 mono (SLAM) | indoor_45_2_snapdragon_with_gt | — | — | 2.2 | 100.0 | — | — | — | 104 | 659 | 5 |
| orb_slam3 mono (VIO-only) | indoor_45_2_snapdragon_with_gt | — | — | 2.1 | 100.0 | — | — | — | 128 | 664 | 5 |

## Conclusions

_Auto-generated by `compare_report.py` from the tables above — refreshed on every run, so they never drift from the data._

Each bullet maps a finding to the relevant section of the **[Evaluation DR](https://hailotech.atlassian.net/wiki/spaces/PhysicalAI/pages/3270180866/VIO+and+SLAM+-+Evaluation+DR)** (the `§x.y` tags — e.g. §2.6 *RPE over segment lengths*, §3.1 *metric table*, §3.3 *robustness*) and checks the measured value against that section's stated target (ATE/RPE/FPS/latency/init-time thresholds), so the verdicts stay tied to the DR's definitions rather than ad-hoc judgement.

- **Accuracy (ATE).** Mean ATE-trans, best→worst: **ORB-SLAM3 0.021 m, Basalt 0.047 m, OpenVINS 0.066 m**. ORB-SLAM3 is the most accurate (~3.1× lower error than OpenVINS). All systems meet the DR ATE targets (V1_01 < 0.10 m, V2_02 < 0.20 m).
- **Loop closure (SLAM vs VIO-only).** Disabling ORB-SLAM3's loop-closure/global-BA stage shifts ATE by **at most 0.007 m — within run-to-run noise, so loop closure makes no meaningful difference here**. Consistent with the DR (§2.1/§2.7): loop closure mainly helps on **revisits**, and these EuRoC sequences are short with little re-observation, so the SLAM back-end adds little global correction.
- **Drift over distance (§2.6 RPE-vs-segment).** On V1_01 over 8→40 m segments, OpenVINS' RPE-trans falls (0.057→0.042 m, range 0.042-0.057) while ORB-SLAM3 stays roughly flat (0.029→0.028 m). ORB-SLAM3's per-segment drift is consistently lower — its local-mapping back-end bounds drift even without loop closure.
- **Robustness (§3.3).** Post-init tracking continuity ~100% with zero track-loss (raw completeness < 100% is the VI-init warm-up, not lost tracking). Init-time meets the DR < 5 s target except MH_03_medium (~19 s), V1_01_easy (~6 s).
- **Compute (§3.1, x86 — illustrative).** OpenVINS is the lighter/faster front-end (~83 FPS, ~1.5 cores) vs ORB-SLAM3 (~36 FPS, ~3.3 cores) — the classic filter-VIO vs optimization-SLAM trade-off. Basalt is the lightest on memory (~93 MB) and fastest (~102 FPS) but the most parallel (~13 cores, TBB). Both clear the DR ≥ 30 FPS bar; OpenVINS meets p99 < 33 ms (~25 ms); ORB-SLAM3 is at/above it (~37 ms). Per the DR, treat these as indicative — real perf profiling belongs on embedded HW.
- **DR coverage & gaps.** Covered: ATE t/r, RPE t/r + per-segment (§2.6), completeness, init-time, track-loss, latency p50/p99, FPS, CPU, RSS. Open items: OpenVINS RSS includes the ros2/rosbag2 process tree (upper bound); loop-closure precision/recall, map-growth, and power are not yet measured; and SchurVINS is deferred — it needs a ROS 1 Melodic container, and no container runtime is available on this host (its ROS 1 bags + submodule are staged for when one is).
