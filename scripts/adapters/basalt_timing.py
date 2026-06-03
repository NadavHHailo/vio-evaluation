#!/usr/bin/env python3
"""Extract per-frame VIO timing from Basalt's stats_sums.ubjson → canonical timing.csv.

Basalt logs ExecutionStats (UBJSON), not a CSV. `stats_sums.ubjson` holds per-frame
arrays:
  frame_id  : image timestamp in ns      (one per frame)
  measure   : total per-frame VIO time (s) — the full update latency
  optimize  : backend optimization time (s) — fewer entries (skips the first frames)

We emit `timestamp,frontend,backend,total` (ms) like the other systems. Basalt's
front-/back-end aren't cleanly separable per frame the way the harness column implies,
so the reported per-frame latency uses `measure` (full VIO update) in the `frontend`
slot — consistent with ORB-SLAM3, whose `frontend` is likewise the full per-frame
tracking time. `backend` is left nan (optimize runs as part of measure / async).

Usage:
  basalt_timing.py <stats_sums.ubjson> <out_timing.csv>
"""
import sys

import ubjson


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, out = sys.argv[1], sys.argv[2]
    with open(src, "rb") as f:
        d = ubjson.load(f)
    fid = d.get("frame_id", [])
    measure = d.get("measure", [])
    if not fid or not measure:
        sys.exit(f"ERROR: stats has no frame_id/measure ({src})")
    n = min(len(fid), len(measure))
    with open(out, "w") as f:
        f.write("timestamp,frontend,backend,total\n")
        for i in range(n):
            ts = fid[i] / 1e9          # ns -> s
            ms_ = measure[i] * 1e3     # s -> ms (full per-frame VIO latency)
            f.write(f"{ts:.9f},{ms_:.6f},nan,{ms_:.6f}\n")
    print(f"wrote {n} frames to {out}  (median ~{sorted(measure)[n // 2] * 1e3:.1f} ms)")


if __name__ == "__main__":
    main()
