# vio-evaluation

Comparative evaluation of four visual-inertial estimation systems — **OpenVINS**, **Basalt**, **ORB-SLAM3**, **SchurVINS** — on two datasets: **EuRoC MAV** (indoor MAV, pinhole stereo) and the **[UZH-FPV Drone Racing Dataset](https://fpv.ifi.uzh.ch/)** (aggressive drone flight, fisheye stereo). Measures trajectory accuracy (ATE/RPE), per-frame runtime, and resource footprint (CPU%, peak RSS) against the same ground truth per dataset.

- **EuRoC** sequences: V1_01_easy, MH_03_medium, V2_02_medium (5 reps each).
- **UZH-FPV** sequence: indoor_45_2_snapdragon_with_gt (fisheye; see **[docs/uzhfpv.md](docs/uzhfpv.md)**).

Companion to [`NadavHHailo/open_vins`](https://github.com/NadavHHailo/open_vins) and the existing `catkin_ws_ov` ROS 2 workspace. Where `catkin_ws_ov` benchmarks OpenVINS in isolation ([cross-platform.md](https://github.com/NadavHHailo/openvins-ros2-workspace/blob/master-candidate/docs/cross-platform/cross-platform.md)), this repo extends that work to a four-way comparison.

## Why this repo exists (prior art)

A unified, runnable open-source harness comparing OpenVINS, Basalt, ORB-SLAM3, and SchurVINS doesn't exist publicly. What does exist:

- **Academic comparisons in paper form**, not runnable: the canonical [Delmerico & Scaramuzza, ICRA 2018](https://rpg.ifi.uzh.ch/docs/ICRA18_Delmerico.pdf) benchmark compared MSCKF/OKVIS/ROVIO/VINS-Mono/SVO+MSF — all of our four target systems are newer (Basalt 2020, OpenVINS 2020, ORB-SLAM3 2021, SchurVINS 2024) and aren't covered.
- **Each system's own paper** publishes an EuRoC comparison table, but each picks its own competitors and harness. The [SchurVINS CVPR 2024 paper](https://openaccess.thecvf.com/content/CVPR2024/papers/Fan_SchurVINS_Schur_Complement-Based_Lightweight_Visual_Inertial_Navigation_System_CVPR_2024_paper.pdf) publishes ATE/runtime tables against OpenVINS (Table 2), but the [SchurVINS GitHub release](https://github.com/bytedance/SchurVINS) only exposes the code, not the comparison harness that produced those numbers — so the tables can't be independently re-run without rebuilding the evaluation pipeline from scratch (which is what this repo does).
- **Open-source evaluation tooling** is post-hoc only: [`rpg_trajectory_evaluation`](https://github.com/uzh-rpg/rpg_trajectory_evaluation), [`evo`](https://github.com/MichaelGrupp/evo), and [`ov_eval`](https://github.com/rpng/open_vins/tree/master/ov_eval) (which this repo reuses) all compute ATE/RPE from a pair of trajectory files. None of them build, run, or instrument the VIO systems for you.

`vio-evaluation` is the missing harness layer between the system code and the evaluation tools.

## The plan in one paragraph

Treat OpenVINS as the baseline — it already produces per-frame timing CSVs, TUM trajectories, and ATE/RPE via `ov_eval error_singlerun` on x86 and RPi5. Build a sibling harness here that runs each of the other three systems through the same EuRoC sequences against the same ground truth file, normalizes their outputs into the same four files (`<seq>_trajectory.txt`, `<seq>_timing.csv`, `<seq>_proc.csv`, `<seq>_stdout.log`), and aggregates everything into a single side-by-side comparison report. Phased rollout by integration friction: ORB-SLAM3 first (standalone CMake, validates the harness), then Basalt (also standalone CMake), then SchurVINS (ROS 1 Melodic in a Docker container — heaviest friction). x86 only for now; embedded targets reuse the harness later.

Full plan with phase-by-phase steps, file-by-file changes, and verification gates: **[docs/plan.md](docs/plan.md)**.

## Dataset: same data, two on-disk formats

All four systems run on the same EuRoC sensor recordings against the same ground truth, but they consume two different on-disk container formats. Not a fairness issue — same images byte-for-byte, same IMU samples byte-for-byte, different container.

| System | Reads from | Path |
|---|---|---|
| OpenVINS | ROS 2 bag (`.db3`) | `~/datasets/euroc/<seq>/<seq>.db3` |
| ORB-SLAM3 | EuRoC ASL folder (PNG + CSV) | `~/datasets/euroc-asl/<seq>/mav0/...` |
| Basalt | EuRoC ASL folder | same as ORB-SLAM3 |
| SchurVINS | ROS 1 bag (`.bag`) | `~/datasets/euroc-ros1/<seq>.bag` |

Each system's reported wall-ms is the **VIO update time** (frontend + backend), *not* the total including bag/PNG decode — the harness keeps I/O time separate from algorithm time so the comparison stays apples-to-apples. Ground truth is the same `ov_eval`-format file for all four (derived from EuRoC ASL's `state_groundtruth_estimate0/data.csv`).

### UZH-FPV (fisheye drone dataset)

UZH-FPV ships as a single ROS 1 `.bag` per sequence (cameras `/snappy_cam/stereo_{l,r}`, IMU `/snappy_imu`, **ground truth in-bag** on `/groundtruth/pose`) with a **fisheye** (equidistant / Kannala-Brandt) Snapdragon rig — so EuRoC's pinhole calib does not apply. The `scripts/uzh_*` tools (all using the `rosbags` lib, no ROS install needed) extract GT, build a EuRoC-ASL folder + a `.db3`, and generate the per-system fisheye configs from the UZH Kalibr camchain. Full pipeline, per-rig calib, and results: **[docs/uzhfpv.md](docs/uzhfpv.md)**.

## Layout

```
systems/
  orb_slam3/   submodule  → NadavHHailo/ORB_SLAM3  (fork of UZ-SLAMLab/ORB_SLAM3)
  basalt/      submodule  → NadavHHailo/basalt     (mirror of VladyslavUsenko/basalt)
  schurvins/   submodule  → NadavHHailo/SchurVINS  (fork of bytedance/SchurVINS)
scripts/
  run_{openvins,basalt,orb_slam3}.sh       EuRoC runners (per system)
  run_{openvins,basalt,orb_slam3}_uzh.sh   UZH-FPV runners (fisheye; idempotent)
  uzh_bag_gt_to_tum.py          UZH in-bag GT → ov_eval TUM
  uzh_bag_to_asl.py             UZH bag → EuRoC-ASL folder (feeds all systems)
  uzh_calib_to_{basalt,orbslam3,openvins}.py   Kalibr camchain → per-system fisheye configs
  compare_report.py             cross-system aggregator → docs/comparison.md
  adapters/                     per-system trajectory → TUM converters
docs/
  plan.md                       implementation plan (this work)
  comparison.md                 final report (populated by compare_report.py)
  uzhfpv.md                     UZH-FPV integration: pipeline, calib, results
  build-{orb_slam3,basalt,schurvins}.md  per-system build notes
```

OpenVINS is intentionally **not** a submodule here — it builds inside the existing `catkin_ws_ov/` ROS 2 colcon workspace and its results are picked up by `compare_report.py` from `~/results/openvins/...`.

## Status

**EuRoC** (V1_01_easy, MH_03_medium, V2_02_medium) — done, 5 reps each:

| System | State |
|---|---|
| OpenVINS | ✅ done |
| Basalt | ✅ done |
| ORB-SLAM3 (SLAM + VIO-only) | ✅ done |
| SchurVINS | deferred — needs a ROS 1 Melodic container (none on host yet) |

**UZH-FPV** (`indoor_45_2_snapdragon_with_gt`, fisheye) — done, 5 reps each. Headline: on
aggressive fisheye flight OpenVINS (ATE 0.26 m) and Basalt (0.65 m) survive; ORB-SLAM3 fails
in all four configs (stereo/mono × SLAM/VIO-only). Details + caveats: **[docs/uzhfpv.md](docs/uzhfpv.md)**.

Open items: other 3 UZH sequences (`indoor_forward_3/7`, `outdoor_forward_1`); `compare_report.py`
UZH section (coverage%/completeness columns); SchurVINS leg.
