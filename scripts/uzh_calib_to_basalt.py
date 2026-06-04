#!/usr/bin/env python3
"""Generate a Basalt calibration JSON from a UZH-FPV Kalibr calibration.

UZH-FPV calib (e.g. indoor_45_calib_snapdragon/) is `pinhole` + `equidistant`
distortion = Kannala-Brandt fisheye → Basalt's `kb4` camera model. This reads the
Kalibr `camchain-imucam-*.yaml` (intrinsics + T_cam_imu extrinsics) and `imu.yaml`
(noise densities), inverts T_cam_imu → Basalt's T_imu_cam (camera pose in the IMU
frame), and writes the JSON consumed by `--cam-calib`.

Conventions:
  * Kalibr T_cam_imu: p_cam = T_cam_imu · p_imu  → Basalt T_imu_cam = inv(T_cam_imu).
  * Basalt kb4 intrinsics = {fx,fy,cx,cy,k1,k2,k3,k4} (equidistant distortion_coeffs).
  * IMU continuous-time noise densities map directly; accel/gyro random-walk →
    *_bias_std. calib_{accel,gyro}_bias set to zero (identity intrinsic, no bias) —
    Basalt estimates IMU biases online.
  * vignette omitted (photometric, optional).

Usage:
  uzh_calib_to_basalt.py <camchain-imucam-*.yaml> <imu.yaml> <out_calib.json>
"""
import argparse
import json
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def imu_cam_pose(T_cam_imu):
    """inv(T_cam_imu) → (px,py,pz, qx,qy,qz,qw)."""
    T = np.array(T_cam_imu, dtype=float)
    Tinv = np.linalg.inv(T)
    q = Rotation.from_matrix(Tinv[:3, :3]).as_quat()  # (x,y,z,w)
    p = Tinv[:3, 3]
    return dict(px=p[0], py=p[1], pz=p[2], qx=q[0], qy=q[1], qz=q[2], qw=q[3])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("camchain")
    ap.add_argument("imu")
    ap.add_argument("out")
    args = ap.parse_args()

    cc = yaml.safe_load(open(args.camchain))
    imu = yaml.safe_load(open(args.imu))

    cams = [k for k in sorted(cc) if k.startswith("cam")]
    T_imu_cam, intrinsics, resolution = [], [], []
    offsets = []
    for cam in cams:
        c = cc[cam]
        if c.get("distortion_model") != "equidistant":
            sys.exit(f"ERROR: {cam} distortion_model={c.get('distortion_model')!r}; "
                     "this generator only handles equidistant (kb4) fisheye.")
        fx, fy, cx, cy = c["intrinsics"]
        k1, k2, k3, k4 = c["distortion_coeffs"]
        intrinsics.append({"camera_type": "kb4", "intrinsics": {
            "fx": fx, "fy": fy, "cx": cx, "cy": cy,
            "k1": k1, "k2": k2, "k3": k3, "k4": k4}})
        resolution.append(list(c["resolution"]))
        T_imu_cam.append(imu_cam_pose(c["T_cam_imu"]))
        if "timeshift_cam_imu" in c:
            offsets.append(c["timeshift_cam_imu"])

    # imu.yaml may be nested (some Kalibr exports wrap it) — take the leaf dict.
    iv = imu if "accelerometer_noise_density" in imu else next(iter(imu.values()))
    an = float(iv["accelerometer_noise_density"])
    gn = float(iv["gyroscope_noise_density"])
    aw = float(iv["accelerometer_random_walk"])
    gw = float(iv["gyroscope_random_walk"])
    rate = float(iv.get("update_rate", 500.0))
    # one global cam-IMU time offset (mean of per-cam timeshifts), seconds → ns
    toff_ns = int(round((sum(offsets) / len(offsets) if offsets else 0.0) * 1e9))

    calib = {"value0": {
        "T_imu_cam": T_imu_cam,
        "intrinsics": intrinsics,
        "resolution": resolution,
        "calib_accel_bias": [0.0] * 9,    # bias(3) + lower-tri scale (6), identity
        "calib_gyro_bias": [0.0] * 12,    # bias(3) + full 3x3 (9), identity
        "imu_update_rate": rate,
        "accel_noise_std": [an, an, an],
        "gyro_noise_std": [gn, gn, gn],
        "accel_bias_std": [aw, aw, aw],
        "gyro_bias_std": [gw, gw, gw],
        "cam_time_offset_ns": toff_ns,
        "vignette": [],   # NVP required by Basalt's cereal loader (empty = no photometric calib)
    }}

    with open(args.out, "w") as f:
        json.dump(calib, f, indent=4)
    print(f"wrote {args.out}: {len(cams)} cam(s) {cams}, kb4 fisheye, "
          f"res {resolution[0]}, imu {rate:.0f} Hz, cam_time_offset {toff_ns} ns")


if __name__ == "__main__":
    main()
