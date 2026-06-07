#!/usr/bin/env bash
# ORB-SLAM3 runner for UZH-FPV (stereo-inertial, KannalaBrandt8 fisheye).
#
# Same plumbing as run_orb_slam3.sh, but points at the UZH ASL folder
# (uzh_bag_to_asl.py output), the fisheye UZH-FPV.yaml (uzh_calib_to_orbslam3.py),
# and the bag-extracted GT. Tag uzhfpv_x86. Idempotent: a rep whose trajectory
# already exists is skipped. Divergence (no f_<seq>.txt) is recorded as an empty
# trajectory so compare_report.py reports 0% completeness instead of crashing.
set -euo pipefail

usage() { echo "usage: run_orb_slam3_uzh.sh <seq> [--reps N] [--tag TAG] [--realtime] [--vio-only] [--mono]"; exit 1; }
[ $# -ge 1 ] || usage
SEQ="$1"; shift
REPS=1; TAG=""; SEQUENTIAL=1; VIO_ONLY=0; MONO=0
while [ $# -gt 0 ]; do
  case "$1" in
    --reps)     REPS="$2"; shift 2;;
    --tag)      TAG="$2";  shift 2;;
    --realtime) SEQUENTIAL=0; shift;;
    --vio-only) VIO_ONLY=1; shift;;
    --mono)     MONO=1; shift;;       # monocular-inertial (cam0 only) instead of stereo
    *) usage;;
  esac
done
# Default tag keeps mono and stereo results in separate dirs.
[ -n "$TAG" ] || TAG=$([ "$MONO" = 1 ] && echo "uzhfpv_mono_x86" || echo "uzhfpv_x86")

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
ORB="$REPO/systems/orb_slam3"
if [ "$MONO" = 1 ]; then
  BIN="$ORB/Examples/Monocular-Inertial/mono_inertial_euroc"
  YAML="$ORB/Examples/Monocular-Inertial/UZH-FPV.yaml"
else
  BIN="$ORB/Examples/Stereo-Inertial/stereo_inertial_euroc"
  YAML="$ORB/Examples/Stereo-Inertial/UZH-FPV.yaml"
fi
VOC="$ORB/Vocabulary/ORBvoc.txt"
ASL="$HOME/datasets/uzhfpv-asl/$SEQ"
GT="$HOME/datasets/uzhfpv-gt/$SEQ.txt"
ADAPTER="$HERE/adapters/orb_slam3_to_tum.py"

for p in "$BIN" "$VOC" "$YAML" "$ASL/mav0/cam0/data" "$GT" "$ADAPTER"; do
  [ -e "$p" ] || { echo "ERROR: missing $p" >&2; exit 1; }
done

if [ "$VIO_ONLY" = 1 ]; then
  TAG="${TAG}_vioonly"
  YAML_EFF="$(mktemp --suffix=.yaml)"
  cp "$YAML" "$YAML_EFF"
  printf '\nloopClosing: 0\n' >> "$YAML_EFF"
  YAML="$YAML_EFF"
  SLAM_MODE="vio-only (loop closure OFF)"
else
  SLAM_MODE="full SLAM (loop closure ON)"
fi

OUT="$HOME/results/orb_slam3/x86/native_jazzy/$TAG"
mkdir -p "$OUT"
export LD_LIBRARY_PATH="$HOME/opt/pangolin/lib:$ORB/lib:$ORB/Thirdparty/DBoW2/lib:$ORB/Thirdparty/g2o/lib:${LD_LIBRARY_PATH:-}"
if [ "$SEQUENTIAL" = 1 ]; then export VIO_EVAL_SEQUENTIAL=1; MODE=sequential; else unset VIO_EVAL_SEQUENTIAL; MODE=realtime; fi

TIMES="$OUT/${SEQ}_times.txt"
ls "$ASL/mav0/cam0/data/" | sed 's/\.png$//' | sort -n > "$TIMES"
echo "[ORB-UZH $SEQ] $(wc -l < "$TIMES") frames; reps=$REPS; mode=$MODE; $SLAM_MODE; out=$OUT"

for i in $(seq 0 $((REPS-1))); do
  if [ -f "$OUT/${SEQ}_rep${i}_trajectory.txt" ]; then
    echo "[ORB-UZH $SEQ] rep $i already present — skipping (idempotent)"; continue
  fi
  WORK="$(mktemp -d)"
  echo "[ORB-UZH $SEQ] rep $i ..."
  ( cd "$WORK" && /usr/bin/time -v -o time.log \
      "$BIN" "$VOC" "$YAML" "$ASL" "$TIMES" "$SEQ" >stdout.log 2>&1 ) \
    || echo "[ORB-UZH $SEQ] rep $i: binary exited nonzero (may have diverged)"
  if [ -f "$WORK/f_$SEQ.txt" ]; then
    # The adapter writes the trajectory before its GT-overlap sanity check, so a
    # diverged rep (few poses, outside the GT window) still produces a file but the
    # adapter exits nonzero. Tolerate that — keep the partial trajectory, don't abort
    # the rep loop (UZH-FPV divergence is an expected, reportable outcome).
    python3 "$ADAPTER" "$WORK/f_$SEQ.txt" "$OUT/${SEQ}_rep${i}_trajectory.txt" "$GT" \
      || echo "[ORB-UZH $SEQ] rep $i: adapter warned (diverged / little-to-no GT overlap) — keeping partial trajectory"
  else
    echo "[ORB-UZH $SEQ] rep $i: NO trajectory (f_$SEQ.txt missing) — writing empty result"
    : > "$OUT/${SEQ}_rep${i}_trajectory.txt"
  fi
  [ -f "$WORK/${SEQ}_timing.csv" ] && cp "$WORK/${SEQ}_timing.csv" "$OUT/${SEQ}_rep${i}_timing.csv"
  awk '
    /Maximum resident set size/ {rss=$NF}
    /User time/                 {usr=$NF}
    /System time/               {sys=$NF}
    /Percent of CPU/            {cpu=$NF; gsub(/%/,"",cpu)}
    /Elapsed .wall clock/       {split($NF,a,":"); wall=(length(a)==3)?a[1]*3600+a[2]*60+a[3]:a[1]*60+a[2]}
    END {printf "peak_rss_kb,user_s,sys_s,pct_cpu,wall_s\n%s,%s,%s,%s,%s\n",rss,usr,sys,cpu,wall}
  ' "$WORK/time.log" > "$OUT/${SEQ}_rep${i}_proc.csv"
  cp "$WORK/stdout.log" "$OUT/${SEQ}_rep${i}_stdout.log"
  rm -rf "$WORK"
done
[ "$VIO_ONLY" = 1 ] && rm -f "$YAML_EFF"
echo "[ORB-UZH $SEQ] done."
