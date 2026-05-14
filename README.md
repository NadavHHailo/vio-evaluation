# vio-evolution

Comparative evaluation harness for visual-inertial estimation systems: **OpenVINS**, **Basalt**, **ORB-SLAM3**, **SchurVINS**.

Companion to [`NadavHHailo/open_vins`](https://github.com/NadavHHailo/open_vins) and the existing `catkin_ws_ov` ROS 2 workspace. Where `catkin_ws_ov` runs OpenVINS, this repo runs the same EuRoC sequences through the other three systems and produces a side-by-side comparison on trajectory accuracy (ATE/RPE), runtime/latency, and resource footprint (CPU%, peak RSS).

## Layout

```
systems/
  orb_slam3/   submodule  → NadavHHailo/ORB_SLAM3  (fork of UZ-SLAMLab/ORB_SLAM3)
  basalt/      submodule  → NadavHHailo/basalt     (mirror of VladyslavUsenko/basalt)
  schurvins/   submodule  → NadavHHailo/SchurVINS  (fork of bytedance/SchurVINS)
scripts/
  run_system.sh                 entrypoint: <system> <seq> [--reps N]
  run_eval.sh                   ATE/RPE via ov_eval error_singlerun (from catkin_ws_ov)
  compare_report.py             cross-system aggregator → docs/comparison.md
  adapters/                     per-system trajectory → TUM converters
docs/
  plan.md                       implementation plan (this work)
  comparison.md                 final report (populated by compare_report.py)
  build-{orb_slam3,basalt,schurvins}.md  per-system build notes
```

OpenVINS is intentionally **not** a submodule here — it builds inside the existing `catkin_ws_ov/` ROS 2 colcon workspace and its results are picked up by `compare_report.py` from `~/results/openvins/...`.

## Status

Bootstrapping. See [docs/plan.md](docs/plan.md) for the full phased rollout.
