# Comparative Evaluation: OpenVINS vs Basalt vs ORB-SLAM3 vs SchurVINS

## Context

The OpenVINS evaluation pipeline in [`catkin_ws_ov/`](/home/hailo/workspace/catkin_ws_ov/) is mature: it produces per-frame timing CSVs, TUM-style trajectory estimates, and ATE/RPE via `ov_eval error_singlerun`, across four host targets (x86-J, x86-H, RPi5-U, RPi5-T) — documented in [`docs/cross-platform/cross-platform.md`](/home/hailo/workspace/catkin_ws_ov/docs/cross-platform/cross-platform.md). To justify (or replace) OpenVINS as the Hailo VIO backbone, we need the same numbers for **Basalt**, **ORB-SLAM3**, and **SchurVINS** on the same EuRoC sequences, measured against the same ground truth.

**Scope of this plan**: x86-only, phased rollout. RPi5/embedded mirroring is a follow-up that will reuse the harness this plan builds.

**Outcome**: A reusable per-system harness (build, run, adapt-output, evaluate) and a cross-system comparison report covering trajectory accuracy (ATE/RPE), runtime/latency, and resource footprint (CPU%, peak RSS) on EuRoC V1_01_easy, MH_03_medium, V2_02_medium.

---

## Design

### Layout (new sibling git repo: `NadavHHailo/vio-evolution`)

Don't fold three new systems into `catkin_ws_ov/src/` — their dep stacks (Pangolin, DBoW2, TBB, custom Sophus) will fight OpenVINS' ROS2 environment. Create a new sibling git repo at `https://github.com/NadavHHailo/vio-evolution`, cloned locally to `/home/hailo/workspace/vio-evolution/`:

```
/home/hailo/workspace/
  catkin_ws_ov/                    # existing — OpenVINS (untouched)
  vio-evolution/                   # NEW git repo (NadavHHailo/vio-evolution)
    .gitmodules                    # pins the three submodules below
    systems/
      orb_slam3/                   # submodule → NadavHHailo/ORB_SLAM3 (fork of UZ-SLAMLab/ORB_SLAM3)
      basalt/                      # submodule → NadavHHailo/basalt    (fork of VladyslavUsenko/basalt)
      schurvins/                   # submodule → NadavHHailo/SchurVINS (fork of bytedance/SchurVINS)
    scripts/
      run_system.sh                # entrypoint: <system> <seq> [--reps N]
      adapters/
        orb_slam3_to_tum.py        # KeyFrameTrajectory.txt → TUM
        basalt_to_tum.py           # basalt traj JSON/txt → TUM
        schurvins_to_tum.py        # ROS topic dump → TUM
      run_eval.sh                  # sources ROS, calls ov_eval error_singlerun
      compare_report.py            # ingests all 4 systems' results, emits table
    docs/
      comparison.md                # final report (mirrors cross-platform.md style)
```

**All three submodules are forked into `NadavHHailo/` first**, then submoduled from the fork (not from upstream). Same pattern as `catkin_ws_ov/src/open_vins` → `NadavHHailo/open_vins`. This is non-optional: at least SchurVINS will almost certainly need timing-instrumentation patches, and likely ORB-SLAM3 will need a small patch to dump per-frame timing as CSV — both require a writable fork. The OpenVINS submodule-management workflow in [`catkin_ws_ov/CLAUDE.md`](/home/hailo/workspace/catkin_ws_ov/CLAUDE.md) (push fork before bumping outer pointer; track branch independently from outer repo) applies verbatim to each new submodule.

**OpenVINS is intentionally *not* a submodule of `vio-evolution/`.** It only builds inside a ROS 2 colcon workspace, which `catkin_ws_ov/` already provides — duplicating that under `vio-evolution/systems/openvins/` would give you a non-buildable tree. Instead, OpenVINS benchmark numbers come from running the existing [`catkin_ws_ov/scripts/run_full_benchmark.sh`](/home/hailo/workspace/catkin_ws_ov/scripts/run_full_benchmark.sh) and are picked up by `compare_report.py` from `~/results/openvins/<arch>/<env>/<tag>/` alongside the new three. The "four-system comparison" framing lives in the report and aggregator, not the directory layout.

### Output convention (mirrors existing OpenVINS layout)

```
~/results/<system>/<arch>/<env>/<tag>/
  <seq>_trajectory.txt   # TUM-format: timestamp tx ty tz qx qy qz qw
  <seq>_timing.csv       # timestamp,frontend,backend,total  (system-normalized)
  <seq>_proc.csv         # /usr/bin/time -v derived: peak_rss_kb, user_s, sys_s, %CPU
  <seq>_stdout.log       # full system output for debugging
```

`<system>` ∈ `{openvins, orb_slam3, basalt, schurvins}`. `<arch>/<env>` and `<tag>` follow the existing scheme in [`bench_lib.sh:arch_results_base()`](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh#L20-L29), so the existing parse logic stays compatible for the OpenVINS rows.

### Reused components

- **Ground truth**: `catkin_ws_ov/src/open_vins/src/ov_data/euroc/{V1_01_easy,MH_03_medium,V2_02_medium}.txt` — the same GT files OpenVINS already uses. No re-export needed.
- **ATE/RPE engine**: `ros2 run ov_eval error_singlerun posyaw <gt> <est> 8 16 24 32 40` — called via [`run_eval.sh`](/home/hailo/workspace/vio-evolution/scripts/run_eval.sh). One-shot per estimate file, no need to reimplement.
- **ROS sourcing pattern**: copy [`bench_lib.sh:source_ros()`](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh#L53-L71) verbatim so `ov_eval` works regardless of the active distro.
- **Sequences + GT alignment**: same three EuRoC sequences as the cross-platform doc. Stereo + IMU (matches OpenVINS benchmark config).

### Per-system runner contract

`run_system.sh <system> <seq>` must produce all four output files. Anything system-specific (build env, EuRoC config path, output post-processing) lives inside a per-system thin wrapper called by `run_system.sh`. The wrapper's job: launch the binary, capture timing, run the adapter, write the four canonical files.

### Resource footprint capture

System-agnostic — wrap each launch in `/usr/bin/time -v` and parse the post-run output for `Maximum resident set size`, `User time`, `System time`, `Percent of CPU this job got`. No need to instrument each codebase. Optional: layer `psrecord --plot` for time-series RAM/CPU graphs if static peaks aren't enough.

### Determinism

All three new systems are non-deterministic by default (multi-threaded frontends, RANSAC without fixed seed). Match OpenVINS' subscribe-mode protocol: **5 reps per sequence**, report mean/std on ATE and wall-ms. Don't chase bit-determinism per system — too much per-system surgery.

---

## Phased rollout (x86)

### Phase 0 — Bootstrap the repo

1. Create empty repo `NadavHHailo/vio-evolution` on GitHub (UI or `gh repo create NadavHHailo/vio-evolution --private`).
2. Fork upstream for each of the three submodules into `NadavHHailo/`:
   - `gh repo fork UZ-SLAMLab/ORB_SLAM3 --clone=false` → `NadavHHailo/ORB_SLAM3`
   - `NadavHHailo/basalt` — upstream is on GitLab, so `gh repo fork` won't work. Mirror it: `git clone --mirror https://gitlab.com/VladyslavUsenko/basalt.git`, then push to a new empty `NadavHHailo/basalt` GitHub repo with `git push --mirror`.
   - `gh repo fork bytedance/SchurVINS --clone=false` → `NadavHHailo/SchurVINS`
3. Locally: `cd /home/hailo/workspace && git clone git@github.com:NadavHHailo/vio-evolution.git && cd vio-evolution`.
4. Create the directory skeleton (`systems/`, `scripts/adapters/`, `docs/`) and an initial `.gitignore` (ignore `build/`, `install/`, `*.log`, local results).
5. First commit: README + empty skeleton, then push.

### Phase 1 — Harness validation on ORB-SLAM3 (smoothest first)

ORB-SLAM3 first because: ships a ready-made [`Examples/Stereo-Inertial/stereo_inertial_euroc`](https://github.com/UZ-SLAMLab/ORB_SLAM3) with EuRoC configs and `EuRoC_TimeStamps/`, has a well-known TUM-format trajectory output (`CameraTrajectory.txt`), and the largest community → least time spent debugging build issues. Validating the full harness end-to-end here de-risks the other two.

Steps:
1. Add ORB-SLAM3 as a submodule at `vio-evolution/systems/orb_slam3/` (`git submodule add <url> systems/orb_slam3`). Pin to a known-good tag. Build natively on x86 (Pangolin, OpenCV ≥3, Eigen3, DBoW2/g2o vendored). Document exact build incantation in `vio-evolution/docs/build-orb-slam3.md`.
2. Write `vio-evolution/systems/orb_slam3/run.sh` — invokes `stereo_inertial_euroc` against the EuRoC bag dirs already at `~/datasets/euroc/`. Note: ORB-SLAM3 wants the *image folder* layout (`mav0/cam0/data/*.png`), not the rosbag — confirm the dataset on disk has both.
3. Adapter `adapters/orb_slam3_to_tum.py`: convert `CameraTrajectory.txt` (already TUM-ish, but timestamps may be in ns vs the GT's seconds) to the canonical `<seq>_trajectory.txt`. Validate timestamp alignment against GT.
4. Timing: ORB-SLAM3 exposes `Tracking::vdTrackTotal_ms` etc. — enable the dump (or patch a one-shot CSV writer onto the example main). Normalize to `timestamp,frontend,backend,total`.
5. End-to-end smoke test: run all three sequences × 5 reps. Verify `run_eval.sh` outputs sensible ATE on V1_01_easy (expect sub-10 cm RMSE — ORB-SLAM3 paper reports ~0.04 m on V1_01).
6. **Gate**: harness is valid if `compare_report.py` emits a table with both `openvins` and `orb_slam3` rows side-by-side, fed from `~/results/{openvins,orb_slam3}/x86/native_jazzy/...`.

### Phase 2 — SchurVINS

Closest to OpenVINS architecturally (Schur-complement filter, paper directly benchmarks against OpenVINS) → most informative head-to-head once the harness exists.

Steps:
1. Add SchurVINS as a submodule at `vio-evolution/systems/schurvins/`. Build (SVO-based stack: Eigen, OpenCV, Sophus, glog, yaml-cpp). Check the upstream README for EuRoC instructions — if it expects ROS1, run it from a ROS1 noetic Docker (parallel to the openvins-humble pattern, but x86-native ROS1 sidecar). If it has a standalone offline runner, prefer that.
2. Write the system-specific runner: launch SchurVINS on each sequence (bag or image folder, depending on what upstream supports), capture pose stream.
3. Adapter `adapters/schurvins_to_tum.py`: convert whatever SchurVINS emits (ROS topic dump if ROS-based, file otherwise) to the canonical TUM trajectory.
4. Timing: instrument the SchurVINS frontend/backend dispatch points if no built-in CSV. Worst case, expose only `total` and accept reduced per-stage breakdown.
5. 5-rep run × 3 sequences; cross-check against numbers in the SchurVINS CVPR 2024 paper Table 2 (ATE on EuRoC).

### Phase 3 — Basalt

Last because: heaviest deps (TBB, Pangolin, custom Sophus fork, ROS bindings are dated), and the offline `basalt_vio` CLI requires its own JSON config conversion from EuRoC's Kalibr YAMLs.

Steps:
1. Add Basalt as a submodule at `vio-evolution/systems/basalt/` (Basalt vendors `basalt-headers` as its own submodule, so a recursive `git submodule update --init --recursive` after adding it picks both up). Build natively (CMake + TBB + Pangolin). Document gotchas in `vio-evolution/docs/build-basalt.md`.
2. Reuse the EuRoC calibration JSON shipped in Basalt's `data/euroc_ds_calib.json` if present, or convert from `kalibr_imucam_chain.yaml` if upstream stopped shipping it.
3. Runner: invoke `basalt_vio --dataset-path ~/datasets/euroc/<seq> --cam-calib <calib.json> --config-path data/euroc_config.json --result-path /tmp/basalt_traj.txt`.
4. Adapter `adapters/basalt_to_tum.py`: Basalt's trajectory output is already TUM-formatted text — just normalize timestamp units and rename.
5. Timing: Basalt's `--show-gui` mode logs per-frame stats; offline mode dumps them to stderr — capture and parse.
6. 5-rep run × 3 sequences; cross-check against Basalt RA-L 2020 Table 2 (ATE on EuRoC).

### Phase 4 — Comparison report

Once all four systems have populated `~/results/<system>/x86/native_jazzy/<tag>/`:

1. `compare_report.py` (new): walk all four `<system>` dirs, for each `(system, seq)` aggregate over reps, call `ov_eval error_singlerun` once per estimate file, and emit:
   - **Headline table**: per-sequence ATE (m, mean±std), wall-ms/frame (mean±std), peak RSS (MB).
   - **Per-stage timing**: where available, side-by-side frontend/backend ms. Note which systems only expose `total` (likely SchurVINS).
   - **Ranking**: vs OpenVINS x86-J as the baseline (matches existing cross-platform.md convention).
2. Render the report to `vio-evolution/docs/comparison.md` following the same style as [`cross-platform.md` §2.1](/home/hailo/workspace/catkin_ws_ov/docs/cross-platform/cross-platform.md#21-subscribe-summary-across-sequences--the-at-a-glance-table) — citations per table, reproducible invocation block at the top, known caveats called out (e.g., ORB-SLAM3's loop closure makes ATE strictly better than pure VIO — flag it).

---

## Critical files to read/modify

**Reused (no edits)**:
- [`catkin_ws_ov/scripts/bench_lib.sh`](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh) — copy `source_ros()` and `detect_ros_distro()` patterns into `vio-evolution/scripts/run_eval.sh`.
- [`catkin_ws_ov/scripts/parse_results.py`](/home/hailo/workspace/catkin_ws_ov/scripts/parse_results.py) — reference for the `error_singlerun` invocation form (lines 69-100); reuse same ANSI strip + RPE regex.
- `catkin_ws_ov/src/open_vins/src/ov_data/euroc/*.txt` — ground truth files.

**New (to be created)**:
- `vio-evolution/` itself — `git init` + initial `.gitignore` (results/build artifacts).
- `vio-evolution/.gitmodules` — three submodules (orb_slam3, basalt, schurvins), pinned to upstream tags or fork commits.
- `vio-evolution/scripts/run_system.sh` — entrypoint dispatcher.
- `vio-evolution/scripts/run_eval.sh` — `ov_eval` wrapper.
- `vio-evolution/scripts/adapters/{orb_slam3,basalt,schurvins}_to_tum.py` — trajectory converters.
- `vio-evolution/scripts/compare_report.py` — cross-system aggregator.
- `vio-evolution/systems/<system>/run.sh` — per-system launcher (3 files).
- `vio-evolution/docs/{build-orb-slam3,build-basalt,build-schurvins,comparison}.md`.

**Existing untouched**:
- All of `catkin_ws_ov/` — OpenVINS pipeline keeps producing baseline numbers from existing `run_full_benchmark.sh` invocations.

---

## Verification

End-to-end correctness gates (run after each phase):

1. **Harness gate** (after Phase 1): `compare_report.py` produces a 2-system table (OpenVINS + ORB-SLAM3) where:
   - ATE on V1_01_easy is sub-15 cm for both (sanity floor).
   - ORB-SLAM3 ATE is in the ballpark of its paper (~0.04 m on V1_01_easy stereo-inertial).
   - Timing CSVs have ≥2000 rows per sequence (every EuRoC bag has >2000 frames; matches `MIN_ROWS_FOR_COMPLETE_RUN` check in [bench_lib.sh:214](/home/hailo/workspace/catkin_ws_ov/scripts/bench_lib.sh#L214)).
2. **Per-system smoke** (after each system added): `run_system.sh <sys> V1_01_easy` produces all four canonical output files; trajectory file passes a TUM-format syntactic check (`python3 -c "import numpy as np; np.loadtxt('<file>'); assert np.loadtxt('<file>').shape[1] == 8"`).
3. **Numbers cross-check**: each system's V1_01_easy ATE is within ~2× of the value in its respective paper. Discrepancies > 2× indicate config or alignment bugs (likely culprit: timestamp units or stereo extrinsics not picked up from EuRoC's `mav0/cam{0,1}/sensor.yaml`).
4. **Final comparison table**: re-render `vio-evolution/docs/comparison.md` from a clean re-run of `compare_report.py` against `~/results/*/x86/native_jazzy/<tag>/`. Spot-check one cell per system against its source CSV.

Reproducible end-to-end check (single command, full sweep):
```bash
for sys in openvins orb_slam3 schurvins basalt; do
  for seq in V1_01_easy MH_03_medium V2_02_medium; do
    bash vio-evolution/scripts/run_system.sh "$sys" "$seq" --reps 5 --tag baseline_x86
  done
done
python3 vio-evolution/scripts/compare_report.py ~/results --tag baseline_x86 --out vio-evolution/docs/comparison.md
```

---

## Out of scope (deferred)

- **RPi5 / Hailo embedded targets** — explicitly Phase-5+. Once `vio-evolution/` works on x86, the same per-system Docker-image pattern that gave OpenVINS its `rpi5-T` (openvins-humble container) can be repeated per system. Don't pre-design the Dockerfiles now; the constraints will be system-specific and easier to address once the x86 harness exists.
- **Loop closure fairness debate** — ORB-SLAM3 will look unfairly good on ATE because of its loop closure. Note it in the report; don't try to disable it (configuration surgery that diverges from "stock" defeats the point of the comparison).
- **Additional sequences** beyond V1_01/MH_03/V2_02 — leave for later. The existing three are what OpenVINS cross-platform.md uses; comparing on the same set is the priority.
- **Multi-rate / threading sweeps** — defer until the four-system baseline is stable. The existing OpenVINS `--rate` and `--threads` sweep dimensions can be added back per-system once each runs cleanly at the default (1.0 rate, stock thread count).
