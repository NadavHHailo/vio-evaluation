#!/usr/bin/env python3
"""Normalize Basalt's `--save-trajectory tum` output to the canonical TUM file.

Basalt already writes `# timestamp tx ty tz qx qy qz qw` with the timestamp in
SECONDS (`t_ns * 1e-9`), so unlike ORB-SLAM3 (ns) no unit conversion is needed —
this just rewrites it with fixed precision, drops comments, asserts 8 columns,
and (when a GT file is given) checks the trajectory's time range overlaps the GT.

Usage:
  basalt_to_tum.py <trajectory.txt> <out_trajectory.txt> [gt.txt]
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
            rows.append((float(p[0]), *p[1:8]))
    if not rows:
        sys.exit(f"ERROR: no pose rows parsed from {src}")
    rows.sort()

    with open(out, "w") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for r in rows:
            f.write(f"{r[0]:.9f} {' '.join(r[1:])}\n")

    t0, t1 = rows[0][0], rows[-1][0]
    print(f"wrote {len(rows)} poses to {out}  (t: {t0:.3f}..{t1:.3f} s)")

    if gt:
        g = [float(ln.split()[0]) for ln in open(gt)
             if ln.strip() and not ln.startswith("#")]
        if g:
            overlap = min(t1, max(g)) - max(t0, min(g))
            if overlap <= 0:
                sys.exit(f"ERROR: trajectory ({t0:.1f}..{t1:.1f}) does not overlap "
                         f"GT ({min(g):.1f}..{max(g):.1f}) — unit/sequence mismatch?")
            print(f"  GT overlap: {overlap:.1f} s — OK")


if __name__ == "__main__":
    main()
