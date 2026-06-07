#!/usr/bin/env python3
"""Generate an ORB-SLAM3 stereo-inertial YAML (KannalaBrandt8 fisheye) from a
UZH-FPV Kalibr calibration.

UZH-FPV is `pinhole` + `equidistant` = Kannala-Brandt fisheye → ORB-SLAM3's
`KannalaBrandt8` model (same as its TUM-VI fisheye example). This reads the Kalibr
`camchain-imucam-*.yaml` + `imu.yaml` and emits a config matching
Examples/Stereo-Inertial/TUM-VI.yaml, with the two extrinsics ORB-SLAM3 needs:

  IMU.T_b_c1   = T_imu_cam0 = inv(cam0.T_cam_imu)   (camera0 pose in the IMU/body frame)
  Stereo.T_c1_c2 = inv(cam1.T_cn_cnm1)              (camera1 pose in the camera0 frame)

Usage:
  uzh_calib_to_orbslam3.py <camchain-imucam-*.yaml> <imu.yaml> <out.yaml> [--fps 30]
"""
import argparse
import sys

import numpy as np
import yaml


def mat(name, M, comment=None):
    rows = "\n".join("         " + ", ".join(f"{v:.12g}" for v in r) + ("," if i < 3 else "")
                     for i, r in enumerate(M))
    head = f"# {comment}\n" if comment else ""
    return (f"{head}{name}: !!opencv-matrix\n"
            f"  rows: 4\n  cols: 4\n  dt: f\n  data: [{rows.strip()}]\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("camchain")
    ap.add_argument("imu")
    ap.add_argument("out")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--mono", action="store_true",
                    help="emit a monocular-inertial config (cam0 only, no stereo)")
    args = ap.parse_args()

    cc = yaml.safe_load(open(args.camchain))
    imu = yaml.safe_load(open(args.imu))
    iv = imu if "accelerometer_noise_density" in imu else next(iter(imu.values()))

    c0, c1 = cc["cam0"], cc["cam1"]
    for c, n in ((c0, "cam0"), (c1, "cam1")):
        if c.get("distortion_model") != "equidistant":
            sys.exit(f"ERROR: {n} is not equidistant (KannalaBrandt8); got "
                     f"{c.get('distortion_model')!r}")
    w, h = c0["resolution"]
    f0 = c0["intrinsics"]; d0 = c0["distortion_coeffs"]
    f1 = c1["intrinsics"]; d1 = c1["distortion_coeffs"]

    T_b_c1 = np.linalg.inv(np.array(c0["T_cam_imu"], float))      # cam0 pose in IMU frame
    T_c1_c2 = np.linalg.inv(np.array(c1["T_cn_cnm1"], float))     # cam1 pose in cam0 frame

    lines = [
        "%YAML:1.0", "",
        '# Generated from UZH-FPV Kalibr calib by uzh_calib_to_orbslam3.py', "",
        'File.version: "1.0"', "",
        'Camera.type: "KannalaBrandt8"', "",
        f"Camera1.fx: {f0[0]}", f"Camera1.fy: {f0[1]}",
        f"Camera1.cx: {f0[2]}", f"Camera1.cy: {f0[3]}",
        f"Camera1.k1: {d0[0]}", f"Camera1.k2: {d0[1]}",
        f"Camera1.k3: {d0[2]}", f"Camera1.k4: {d0[3]}", "",
    ]
    if not args.mono:
        lines += [
            f"Camera2.fx: {f1[0]}", f"Camera2.fy: {f1[1]}",
            f"Camera2.cx: {f1[2]}", f"Camera2.cy: {f1[3]}",
            f"Camera2.k1: {d1[0]}", f"Camera2.k2: {d1[1]}",
            f"Camera2.k3: {d1[2]}", f"Camera2.k4: {d1[3]}", "",
            mat("Stereo.T_c1_c2", T_c1_c2, "cam1 pose in cam0 frame = inv(T_cn_cnm1)"),
            "# Lapping area between images (full width — fisheye stereo overlap)",
            "Camera1.overlappingBegin: 0", f"Camera1.overlappingEnd: {w-1}",
            "Camera2.overlappingBegin: 0", f"Camera2.overlappingEnd: {w-1}", "",
        ]
    lines += [
        f"Camera.width: {w}", f"Camera.height: {h}",
        f"Camera.fps: {args.fps}", "Camera.RGB: 1", "",
        "Stereo.ThDepth: 40.0", "",
        mat("IMU.T_b_c1", T_b_c1, "cam0 pose in IMU/body frame = inv(T_cam_imu)"),
        f"IMU.NoiseGyro: {iv['gyroscope_noise_density']}",
        f"IMU.NoiseAcc: {iv['accelerometer_noise_density']}",
        f"IMU.GyroWalk: {iv['gyroscope_random_walk']}",
        f"IMU.AccWalk: {iv['accelerometer_random_walk']}",
        f"IMU.Frequency: {iv.get('update_rate', 500.0)}", "",
        "# ORB Parameters", "ORBextractor.nFeatures: 1500",
        "ORBextractor.scaleFactor: 1.2", "ORBextractor.nLevels: 8",
        "ORBextractor.iniThFAST: 20", "ORBextractor.minThFAST: 7", "",
        "# Viewer (headless; values copied from EuRoC.yaml)",
        "Viewer.KeyFrameSize: 0.05", "Viewer.KeyFrameLineWidth: 1.0",
        "Viewer.GraphLineWidth: 0.9", "Viewer.PointSize: 2.0",
        "Viewer.CameraSize: 0.08", "Viewer.CameraLineWidth: 3.0",
        "Viewer.ViewpointX: 0.0", "Viewer.ViewpointY: -0.7",
        "Viewer.ViewpointZ: -1.8", "Viewer.ViewpointF: 500.0",
        "Viewer.imageViewScale: 1.0",
    ]
    with open(args.out, "w") as fo:
        fo.write("\n".join(lines) + "\n")
    print(f"wrote {args.out}: KannalaBrandt8 {w}x{h}, fps {args.fps}, "
          f"baseline {abs(T_c1_c2[0,3])*100:.1f} cm, imu {iv.get('update_rate',500):.0f} Hz")


if __name__ == "__main__":
    main()
