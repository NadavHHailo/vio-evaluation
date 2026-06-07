#!/usr/bin/env python3
"""Extract a UZH-FPV ROS 1 bag into the EuRoC ASL folder layout.

One extraction feeds all three systems: Basalt (`--dataset-type euroc`),
ORB-SLAM3 (reads ASL directly), and — after `rosbags-convert` to `.db3` — the
OpenVINS leg. Uses the `rosbags` library (no ROS install needed); mirrors the
output of db3_to_asl.py so the rest of the harness is unchanged.

UZH-FPV topics (Snapdragon rig): /snappy_cam/stereo_l, /snappy_cam/stereo_r
(mono8 640x480), /snappy_imu (~500 Hz). Timestamps come from each message header.

Output (EuRoC ASL):
  <out>/mav0/cam0/data/<ts_ns>.png  + cam0/data.csv  (#timestamp [ns],filename)
  <out>/mav0/cam1/data/<ts_ns>.png  + cam1/data.csv
  <out>/mav0/imu0/data.csv          (#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z)

Usage:
  uzh_bag_to_asl.py <bag> <out_seq_dir>
    [--cam0 /snappy_cam/stereo_l] [--cam1 /snappy_cam/stereo_r] [--imu /snappy_imu]
"""
import argparse
import csv
import os
import sys

import cv2
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

TS = get_typestore(Stores.ROS1_NOETIC)


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def img_to_array(m):
    """sensor_msgs/Image (mono8 / bgr8 / rgb8) → 2-D uint8 (mono) ndarray."""
    buf = np.frombuffer(m.data, dtype=np.uint8)
    if m.encoding == "mono8":
        return buf.reshape(m.height, m.step)[:, : m.width]
    if m.encoding in ("bgr8", "rgb8"):
        arr = buf.reshape(m.height, m.step // 3, 3)[:, : m.width, :]
        # cv2 wants BGR; rgb8 needs a swap. Convert to gray for VIO.
        if m.encoding == "rgb8":
            arr = arr[:, :, ::-1]
        return cv2.cvtColor(np.ascontiguousarray(arr), cv2.COLOR_BGR2GRAY)
    sys.exit(f"ERROR: unhandled image encoding {m.encoding!r}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--cam0", default="/snappy_cam/stereo_l")
    ap.add_argument("--cam1", default="/snappy_cam/stereo_r")
    ap.add_argument("--imu", default="/snappy_imu")
    args = ap.parse_args()

    cam_topics = {args.cam0: "cam0", args.cam1: "cam1"}
    mav0 = os.path.join(args.out, "mav0")
    for cam in cam_topics.values():
        os.makedirs(os.path.join(mav0, cam, "data"), exist_ok=True)
    os.makedirs(os.path.join(mav0, "imu0"), exist_ok=True)

    cam_rows = {cam: [] for cam in cam_topics.values()}
    imu_rows = []
    n_img = 0

    with Reader(args.bag) as r:
        present = {c.topic for c in r.connections}
        for t in (*cam_topics, args.imu):
            if t not in present:
                sys.exit(f"ERROR: bag has no topic {t}. Present: {sorted(present)}")
        wanted = [c for c in r.connections if c.topic in cam_topics or c.topic == args.imu]
        for conn, _, raw in r.messages(connections=wanted):
            m = TS.deserialize_ros1(raw, conn.msgtype)
            ts = stamp_ns(m.header)
            if conn.topic in cam_topics:
                cam = cam_topics[conn.topic]
                fname = f"{ts}.png"
                cv2.imwrite(os.path.join(mav0, cam, "data", fname), img_to_array(m))
                cam_rows[cam].append((ts, fname))
                n_img += 1
                if n_img % 1000 == 0:
                    print(f"  ...{n_img} images", flush=True)
            else:
                w, a = m.angular_velocity, m.linear_acceleration
                imu_rows.append((ts, w.x, w.y, w.z, a.x, a.y, a.z))

    for cam, rows in cam_rows.items():
        rows.sort()
        with open(os.path.join(mav0, cam, "data.csv"), "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["#timestamp [ns]", "filename"])
            wr.writerows(rows)
    imu_rows.sort()
    with open(os.path.join(mav0, "imu0", "data.csv"), "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]",
                     "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]",
                     "a_RS_S_z [m s^-2]"])
        wr.writerows(imu_rows)

    c0, c1 = (len(cam_rows[c]) for c in ("cam0", "cam1"))
    print(f"DONE {args.out}: cam0={c0} cam1={c1} imu0={len(imu_rows)}")
    if c0 != c1:
        print(f"  WARNING: cam0/cam1 frame count mismatch ({c0} vs {c1})")


if __name__ == "__main__":
    main()
