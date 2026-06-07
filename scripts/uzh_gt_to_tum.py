#!/usr/bin/env python3
"""Convert a UZH-FPV ground-truth file to the ov_eval / EuRoC TUM format used by
the rest of this harness, and (optionally) report GROUND-TRUTH COVERAGE against a
VIO estimate.

Why this exists
---------------
The four systems are scored against a single TUM ground-truth file per sequence
(`# timestamp(s) tx ty tz qx qy qz qw`, seconds), the same layout EuRoC uses
(see ~/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav/<seq>.txt). UZH-FPV
ships its GT in the UZH-RPG `groundtruth.txt` layout (Leica MS50). This script
normalizes that into our target format so UZH sequences flow through
compare_report.py's evo engine unchanged.

Coverage (the important UZH-specific caveat)
--------------------------------------------
UZH-FPV ground truth is NOT full-trajectory:
  * indoor `_with_gt` sequences: dense (Leica has line-of-sight throughout).
  * outdoor `_with_gt` sequences: PARTIAL — the tracker drops lock, so GT covers
    only part of the flight. ATE/RPE then reflect a subset, not the whole run.
Given a VIO estimate (TUM), `--est` reports coverage =
  (# estimate poses with a GT pose within --max-diff s) / (# estimate poses).
Report this next to ATE so a partial-GT outdoor run is never read as a
full-trajectory result.

Input format (auto-detected)
----------------------------
Whitespace- or comma-separated, `#`-comment lines skipped. Accepts either:
  8 cols:  t  tx ty tz  qx qy qz qw                (UZH-RPG stamped_groundtruth)
  9 cols:  id  t  tx ty tz  qx qy qz qw             (leading index column)
Timestamps are assumed to be in SECONDS (UZH-RPG convention). If the values look
like nanoseconds (> 1e16) they are divided by 1e9; pass --ts-scale to override.

Usage
-----
  uzh_gt_to_tum.py <uzh_groundtruth.txt> <out_seq.txt>
  uzh_gt_to_tum.py <uzh_groundtruth.txt> <out_seq.txt> --est <vio_traj_tum.txt>
                   [--max-diff 0.02] [--ts-scale auto|s|ns]
"""
import argparse
import sys

HEADER = "# timestamp(s) tx ty tz qx qy qz qw"


def read_rows(path):
    """Return list of (t, tx,ty,tz, qx,qy,qz,qw) floats from a UZH GT file."""
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.replace(",", " ").split()
            if len(parts) == 9:        # leading id column → drop it
                parts = parts[1:]
            if len(parts) != 8:
                continue               # malformed / unexpected width → skip
            try:
                rows.append(tuple(float(x) for x in parts))
            except ValueError:
                continue
    return rows


def scale_factor(rows, mode):
    if mode == "s":
        return 1.0
    if mode == "ns":
        return 1e-9
    # auto: UZH GT is seconds; guard against an accidental ns dump.
    return 1e-9 if rows and rows[0][0] > 1e16 else 1.0


def est_stamps(path):
    """First-column timestamps (s) of a TUM trajectory (comments skipped)."""
    ts = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            try:
                ts.append(float(ln.split()[0]))
            except (ValueError, IndexError):
                continue
    return ts


def coverage(est_ts, gt_ts, max_diff):
    """Fraction of est_ts within max_diff seconds of some gt_ts (two-pointer over
    sorted lists). Returns (matched, total, fraction)."""
    if not est_ts:
        return 0, 0, 0.0
    g = sorted(gt_ts)
    matched = 0
    j = 0
    for t in sorted(est_ts):
        while j + 1 < len(g) and abs(g[j + 1] - t) <= abs(g[j] - t):
            j += 1
        if g and abs(g[j] - t) <= max_diff:
            matched += 1
    return matched, len(est_ts), matched / len(est_ts)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", help="UZH-FPV ground-truth file")
    ap.add_argument("out", help="output TUM file (ov_eval format)")
    ap.add_argument("--est", help="VIO estimate (TUM) to report coverage against")
    ap.add_argument("--max-diff", type=float, default=0.02,
                    help="max |t_est - t_gt| (s) counted as covered (default 0.02)")
    ap.add_argument("--ts-scale", choices=("auto", "s", "ns"), default="auto",
                    help="input timestamp unit (default auto-detect)")
    args = ap.parse_args()

    rows = read_rows(args.src)
    if not rows:
        sys.exit(f"ERROR: no valid GT rows parsed from {args.src} "
                 f"(expected 8 or 9 whitespace/comma columns)")
    rows.sort(key=lambda r: r[0])
    sf = scale_factor(rows, args.ts_scale)

    with open(args.out, "w") as f:
        f.write(HEADER + "\n")
        for r in rows:
            t = r[0] * sf
            f.write(f"{t:.9f} " + " ".join(f"{v:.6f}" for v in r[1:]) + "\n")

    span = (rows[-1][0] - rows[0][0]) * sf
    print(f"wrote {args.out}: {len(rows)} GT poses, "
          f"{span:.1f}s span ({rows[0][0]*sf:.3f}..{rows[-1][0]*sf:.3f})")

    if args.est:
        gt_ts = [r[0] * sf for r in rows]
        m, n, frac = coverage(est_stamps(args.est), gt_ts, args.max_diff)
        verdict = "FULL" if frac >= 0.95 else "PARTIAL" if frac >= 0.05 else "NONE"
        print(f"coverage vs {args.est}: {m}/{n} estimate poses have GT "
              f"within {args.max_diff}s = {100*frac:.1f}%  [{verdict}]")
        if verdict != "FULL":
            print("  NOTE: partial GT — ATE/RPE reflect only the covered subset. "
                  "Report this coverage% alongside accuracy.")


if __name__ == "__main__":
    main()
