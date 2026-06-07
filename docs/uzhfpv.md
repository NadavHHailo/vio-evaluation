# UZH-FPV drone dataset integration

Extends the EuRoC comparison to the [UZH-FPV Drone Racing Dataset](https://fpv.ifi.uzh.ch/)
— aggressive quadrotor flight with a **fisheye** Snapdragon stereo rig + IMU. Kept
as a **separate** comparison from EuRoC (different camera model and difficulty
regime); UZH runs use the tag `uzhfpv_x86`.

## Sequences (Snapdragon, `_with_gt` only)

ATE/RPE needs ground truth, so we only use the `_with_gt` sequences (Leica MS50).

| Role | Sequence | GT |
|---|---|---|
| Gentle (≈ V1_01_easy) | `indoor_45_2_snapdragon_with_gt` | dense |
| Medium forward | `indoor_forward_3_snapdragon_with_gt` | dense |
| Hard / divergence | `indoor_forward_7_snapdragon_with_gt` | dense |
| Outdoor stress | `outdoor_forward_1_snapdragon_with_gt` | **partial** (tracker drops lock) |

> Indoor `_with_gt` = the accuracy benchmark (trust the numbers).
> Outdoor `_with_gt` = a robustness/divergence stress test — GT is partial, so ATE/RPE
> cover only a subset. Always report coverage% next to ATE for outdoor (see below).

## The three things that differ from EuRoC

1. **Fisheye, not pinhole.** UZH-FPV (Snapdragon rig) is `pinhole` + `equidistant`
   distortion = Kannala-Brandt **KB4** fisheye. Every system needs a NEW calibration
   from the UZH Kalibr `camchain-imucam-*.yaml`; EuRoC calib does not apply.
   **Each rig has its own calib** — `indoor_45_*`, `indoor_forward_*`, and
   `outdoor_forward_*` use separate calibration zips.
2. **ROS 1 `.bag` distribution.** Confirmed topics (from
   `indoor_45_2_snapdragon_with_gt`, v3): cameras `/snappy_cam/stereo_l` +
   `/snappy_cam/stereo_r` (mono8 640×480, ~26 Hz), IMU `/snappy_imu` (~500 Hz), and
   **GT is IN the bag** on `/groundtruth/pose` (PoseStamped) + `/groundtruth/odometry`
   (Odometry, ~500 Hz) — there is no separate `groundtruth.txt`.
3. **GT is in the Leica world frame** with its own extrinsics + time offset →
   always evaluate with `evo` alignment (`--align se3`, which `compare_report.py`
   already does), and watch coverage (it is partial even indoors — see below).

## Pipeline (all scripts use the `rosbags` lib — no ROS install needed)

1. **GT from bag** → ov_eval TUM:
   [scripts/uzh_bag_gt_to_tum.py](../scripts/uzh_bag_gt_to_tum.py) reads
   `/groundtruth/pose`. ([scripts/uzh_gt_to_tum.py](../scripts/uzh_gt_to_tum.py)
   converts a text-file GT and computes the **coverage metric** via `--est`.)
2. **Bag → EuRoC ASL folder**: [scripts/uzh_bag_to_asl.py](../scripts/uzh_bag_to_asl.py)
   — one extraction feeds Basalt (`--dataset-type euroc`), ORB-SLAM3 (reads ASL
   directly), and OpenVINS (after `rosbags-convert` to `.db3`).
3. **Kalibr → Basalt calib JSON**:
   [scripts/uzh_calib_to_basalt.py](../scripts/uzh_calib_to_basalt.py) inverts
   `T_cam_imu`, maps IMU noise, emits the `kb4` calib (incl. the empty `vignette`
   field Basalt's cereal loader requires).
4. **Run Basalt**: [scripts/run_basalt_uzh.sh](../scripts/run_basalt_uzh.sh) (tag
   `uzhfpv_x86`); records diverged runs as empty trajectories rather than dropping them.

```bash
RAW=~/datasets/uzhfpv-raw; SEQ=indoor_45_2_snapdragon_with_gt
# GT
python3 scripts/uzh_bag_gt_to_tum.py $RAW/$SEQ.bag ~/datasets/uzhfpv-gt/$SEQ.txt
# images + IMU → ASL
python3 scripts/uzh_bag_to_asl.py    $RAW/$SEQ.bag ~/datasets/uzhfpv-asl/$SEQ
# calib (indoor_45 rig) → Basalt JSON
python3 scripts/uzh_calib_to_basalt.py \
  $RAW/indoor_45_calib_snapdragon/camchain-imucam-*.yaml \
  $RAW/indoor_45_calib_snapdragon/imu.yaml \
  systems/basalt/data/uzh_ds_calib.json
# run
./scripts/run_basalt_uzh.sh $SEQ --reps 1
```

> Calib zip per rig: `http://rpg.ifi.uzh.ch/datasets/uzh-fpv/calib/<rig>_calib_snapdragon.zip`
> (follows redirects to `download.ifi.uzh.ch`). The bag/calib `uzh_ds_calib.json`
> lives inside the `systems/basalt` submodule and is a generated artifact —
> regenerate with the command above rather than committing it to the parent repo.

## Status — all three systems run on `indoor_45_2_snapdragon_with_gt` (5 reps each)

| System | ATE-trans (m) | Completeness | Coverage vs GT | Notes |
|---|---|---|---|---|
| **OpenVINS** | **0.263** | 49% | 73% | best; equidistant `uzhfpv_indoor_45` config, ~serial 4-thr |
| **Basalt** | 0.651 | 100% | 64% | sliding-window VIO, ~4.8 ms/frame |
| **ORB-SLAM3** stereo (SLAM / VIO-only) | (0.04) | **3% / 8%** | 25% | ❌ diverges — ATE is over the few tracked frames only |
| **ORB-SLAM3** mono (SLAM / VIO-only) | — | ~2% / ~0% | 0% | ❌ diverges entirely (no GT overlap) |

**Finding:** on aggressive fisheye drone flight, the filter (OpenVINS) and sliding-window
optimizer (Basalt) survive; the feature-based front-end (ORB-SLAM3) fails in **all four**
configurations (stereo/mono × SLAM/VIO-only) — IMU init repeatedly fails and tracking is
lost. The parenthesised ORB-SLAM3 ATEs are a trap: tiny only because computed over the
3–8 % of frames it briefly tracked — **completeness is the honest metric there.**

Runners (idempotent, skip existing reps; diverged reps recorded as empty trajectories):
[run_basalt_uzh.sh](../scripts/run_basalt_uzh.sh),
[run_orb_slam3_uzh.sh](../scripts/run_orb_slam3_uzh.sh) (`--mono` / `--vio-only`),
[run_openvins_uzh.sh](../scripts/run_openvins_uzh.sh). Calib generators:
[uzh_calib_to_basalt.py](../scripts/uzh_calib_to_basalt.py),
[uzh_calib_to_orbslam3.py](../scripts/uzh_calib_to_orbslam3.py) (`--mono`),
[uzh_calib_to_openvins.py](../scripts/uzh_calib_to_openvins.py) (cv::FileStorage-strict layout).

| Leg | State |
|---|---|
| Basalt / OpenVINS / ORB-SLAM3 on `indoor_45_2` | ✅ done (5 reps each) |
| Other 3 UZH seqs (`indoor_forward_3/7`, `outdoor_forward_1`) | pending — download + per-rig calib zips |
| **Report** | pending — `compare_report.py` needs `--gt-dir`/`--frames-root` flags + coverage% / completeness columns |

> **Coverage is partial even indoors** (64–73% on `indoor_45_2`), so report coverage%
> next to ATE for every UZH sequence, not just outdoor.
