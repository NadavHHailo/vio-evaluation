#!/usr/bin/env python3
"""Convert ORB-SLAM3 EuRoC trajectory output to the canonical TUM file.

ORB-SLAM3's `f_<name>.txt` (full per-frame trajectory, SaveTrajectoryEuRoC) is
already TUM-ordered (`timestamp tx ty tz qx qy qz qw`) BUT writes the timestamp
in NANOSECONDS, while our ground truth (ov_data/euroc_mav/<seq>.txt) and the rest
of the harness use SECONDS. This adapter divides the timestamp by 1e9, drops any
non-finite/short rows, and writes `<seq>_trajectory.txt`.

It asserts the output has 8 columns and (when a GT file is given) that the
trajectory's time range overlaps the GT — the cheapest guard against a units or
sequence mix-up.

Usage:
  orb_slam3_to_tum.py <f_traj.txt> <out_trajectory.txt> [gt.txt]
"""
import sys


def main():
    if len(sys.argv) not in (3, 4):
        sys.exit(__doc__)
    src, out = sys.argv[1], sys.argv[2]
    gt = sys.argv[3] if len(sys.argv) == 4 else None

    rows = []
    with open(src) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            t = float(p[0]) / 1e9  # ns -> s
            rows.append((t, *p[1:8]))
    if not rows:
        sys.exit(f"ERROR: no pose rows parsed from {src}")

    with open(out, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for r in rows:
            f.write(f"{r[0]:.9f} {' '.join(r[1:])}\n")

    t0, t1 = rows[0][0], rows[-1][0]
    print(f"wrote {len(rows)} poses to {out}  (t: {t0:.3f}..{t1:.3f} s)")

    if gt:
        g = []
        with open(gt) as f:
            for line in f:
                if line.startswith("#") or not line.strip():
                    continue
                g.append(float(line.split()[0]))
        if g:
            g0, g1 = min(g), max(g)
            overlap = min(t1, g1) - max(t0, g0)
            if overlap <= 0:
                sys.exit(f"ERROR: trajectory ({t0:.1f}..{t1:.1f}) does not overlap "
                         f"GT ({g0:.1f}..{g1:.1f}) — timestamp units/sequence mismatch?")
            print(f"  GT overlap: {overlap:.1f} s — OK")


if __name__ == "__main__":
    main()
