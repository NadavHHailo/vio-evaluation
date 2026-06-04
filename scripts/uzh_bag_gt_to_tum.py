#!/usr/bin/env python3
"""Extract UZH-FPV ground truth from the ROS 1 bag into ov_eval / EuRoC TUM format.

UZH-FPV ships GT *inside* the bag (not as a separate groundtruth.txt) on
`/groundtruth/pose` (geometry_msgs/PoseStamped) and `/groundtruth/odometry`
(nav_msgs/Odometry). This reads the pose topic (no ROS install needed — uses the
`rosbags` library) and writes `# timestamp(s) tx ty tz qx qy qz qw`, the same
format every system in this repo is scored against.

The camera/IMU/GT timestamps share one clock in the bag, so the extracted GT
aligns in time with each system's trajectory; evo's SE3 alignment
(compare_report.py) then handles the Leica-world → VIO-body frame offset.

Usage:
  uzh_bag_gt_to_tum.py <bag> <out_seq.txt> [--topic /groundtruth/pose]
"""
import argparse
import sys

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

HEADER = "# timestamp(s) tx ty tz qx qy qz qw"


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("out")
    ap.add_argument("--topic", default="/groundtruth/pose",
                    help="PoseStamped or Odometry GT topic (default /groundtruth/pose)")
    args = ap.parse_args()

    ts = get_typestore(Stores.ROS1_NOETIC)
    rows = []
    with Reader(args.bag) as r:
        conns = [c for c in r.connections if c.topic == args.topic]
        if not conns:
            avail = sorted({c.topic for c in r.connections})
            sys.exit(f"ERROR: topic {args.topic} not in bag. Available: {avail}")
        msgtype = conns[0].msgtype
        for conn, _, raw in r.messages(connections=conns):
            m = ts.deserialize_ros1(raw, conn.msgtype)
            # PoseStamped: m.pose.{position,orientation}; Odometry: m.pose.pose.{...}
            p = m.pose.pose if "Odometry" in msgtype else m.pose
            t = stamp_s(m.header)
            pos, q = p.position, p.orientation
            rows.append((t, pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w))

    if not rows:
        sys.exit(f"ERROR: no GT messages read from {args.topic}")
    rows.sort(key=lambda r: r[0])
    with open(args.out, "w") as f:
        f.write(HEADER + "\n")
        for r in rows:
            f.write(f"{r[0]:.9f} " + " ".join(f"{v:.6f}" for v in r[1:]) + "\n")

    span = rows[-1][0] - rows[0][0]
    print(f"wrote {args.out}: {len(rows)} GT poses from {args.topic}, "
          f"{span:.1f}s span ({rows[0][0]:.3f}..{rows[-1][0]:.3f}), "
          f"~{len(rows)/span:.0f} Hz")


if __name__ == "__main__":
    main()
