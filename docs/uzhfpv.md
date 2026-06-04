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

## Status

| Leg | State |
|---|---|
| **Basalt** (`indoor_45_2`) | ✅ validated end-to-end: ATE-trans **0.651 m**, ATE-rot **2.45°**, GT coverage **64.4%** (GT 49.4 s of the 74.7 s flight), ~4.8 ms/frame |
| Basalt other 3 seqs | pending — download `indoor_forward_3/7` + `outdoor_forward_1` and their calib zips |
| **ORB-SLAM3** | pending — write `UZH-FPV.yaml` (`Camera.type: KannalaBrandt8`) from the same calib numbers, run mono-inertial first on the ASL folder |
| **OpenVINS** | pending — `rosbags-convert` bag→`.db3` remapping to `/cam0/image_raw`,`/cam1/image_raw`,`/imu0`; estimator YAML with `cam0_distortion_model: equidistant` |
| **Report** | pending — `compare_report.py` needs `--gt-dir`/`--frames-root` flags + a coverage% column (EuRoC-specific `conclusions()` bullets self-skip on UZH seq names) |

> **Coverage is partial even indoors** (64% on `indoor_45_2`), so report coverage%
> next to ATE for every UZH sequence, not just outdoor.
