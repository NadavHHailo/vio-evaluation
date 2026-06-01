#!/usr/bin/env python3
"""Reconstruct the EuRoC ASL (folder) layout from a ROS 2 .db3 bag.

The canonical ETH download host (robotics.ethz.ch) that serves the ASL .zip
archives is dead. ORB-SLAM3 and Basalt both consume the ASL folder layout,
not the ROS 2 bag — so we regenerate that layout from the SAME bag OpenVINS
already benchmarks against. Same camera frames, same IMU samples, same
timestamps: the cross-system comparison stays fair (arguably more so, since
every system is fed bytes that trace back to one recording).

Output (per sequence), matching EuRoC ASL:
  <out>/mav0/cam0/data/<ts_ns>.png   + cam0/data.csv  (#timestamp [ns],filename)
  <out>/mav0/cam1/data/<ts_ns>.png   + cam1/data.csv
  <out>/mav0/imu0/data.csv           (#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z)

Calibration yamls (cam{0,1}/sensor.yaml, imu0/sensor.yaml) are NOT in the bag;
ORB-SLAM3 ships its own EuRoC.yaml and Basalt takes --cam-calib JSON, so they
are not required here.

Usage:
  python3 db3_to_asl.py <bag_dir_or_db3> <out_seq_dir>
  # e.g. python3 db3_to_asl.py ~/datasets/euroc/V1_01_easy ~/datasets/euroc-asl/V1_01_easy
"""
import csv
import os
import sys

import cv2
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CAM_TOPICS = {"/cam0/image_raw": "cam0", "/cam1/image_raw": "cam1"}
IMU_TOPIC = "/imu0"


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def open_reader(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, out = sys.argv[1], sys.argv[2]
    # rosbag2 wants the bag directory (the one holding metadata.yaml).
    uri = src if os.path.isdir(src) else os.path.dirname(src)

    bridge = CvBridge()
    reader = open_reader(uri)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (*CAM_TOPICS, IMU_TOPIC):
        if topic not in type_map:
            sys.exit(f"ERROR: bag {uri} has no topic {topic}")

    mav0 = os.path.join(out, "mav0")
    for cam in CAM_TOPICS.values():
        os.makedirs(os.path.join(mav0, cam, "data"), exist_ok=True)
    os.makedirs(os.path.join(mav0, "imu0"), exist_ok=True)

    cam_rows = {cam: [] for cam in CAM_TOPICS.values()}
    imu_rows = []
    n_img = 0

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in CAM_TOPICS:
            cam = CAM_TOPICS[topic]
            msg = deserialize_message(data, get_message(type_map[topic]))
            ts = stamp_ns(msg.header)
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            fname = f"{ts}.png"
            cv2.imwrite(os.path.join(mav0, cam, "data", fname), img)
            cam_rows[cam].append((ts, fname))
            n_img += 1
            if n_img % 1000 == 0:
                print(f"  ...{n_img} images", flush=True)
        elif topic == IMU_TOPIC:
            msg = deserialize_message(data, get_message(type_map[topic]))
            ts = stamp_ns(msg.header)
            w, a = msg.angular_velocity, msg.linear_acceleration
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
    print(f"DONE {out}: cam0={c0} cam1={c1} imu0={len(imu_rows)}")
    if c0 != c1:
        print(f"  WARNING: cam0/cam1 frame count mismatch ({c0} vs {c1})")


if __name__ == "__main__":
    main()
