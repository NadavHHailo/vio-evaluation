#!/usr/bin/env bash
# OpenVINS runner for UZH-FPV (fisheye, equidistant) — serial mode.
#
# Same measurement method as run_openvins.sh (wraps the estimator in
# /usr/bin/time -v, reuses bench_lib.sh's make_bench_config), but points at:
#   * the UZH ROS 2 .db3 bag (rosbags-convert of the UZH .bag; topics kept as
#     /snappy_cam/stereo_{l,r} + /snappy_imu — OpenVINS subscribes to whatever the
#     config names, so NO topic remap is needed)
#   * the uzhfpv_indoor_45 config (equidistant fisheye, built from the UZH Kalibr
#     camchain). OpenVINS reads the Kalibr chain format natively.
# Tag uzhfpv_x86. Idempotent: a rep whose _est.txt already exists is skipped.
set -euo pipefail

usage() { echo "usage: run_openvins_uzh.sh <seq> [--tag TAG] [--thr N] [--reps N] [--config DIR]"; exit 1; }
[ $# -ge 1 ] || usage
SEQ="$1"; shift
TAG="uzhfpv_x86"; THR=4; REPS=1; CFGNAME="uzhfpv_indoor_45"
while [ $# -gt 0 ]; do
  case "$1" in
    --tag)    TAG="$2"; shift 2;;
    --thr)    THR="$2"; shift 2;;
    --reps)   REPS="$2"; shift 2;;
    --config) CFGNAME="$2"; shift 2;;
    *) usage;;
  esac
done

CATKIN="$HOME/workspace/catkin_ws_ov"
export WS_DIR="$CATKIN"
# Override bench_lib defaults to the UZH dataset + config (bench_lib uses :- so
# these env vars win). Must be set BEFORE sourcing bench_lib.sh.
export DATASETS_DIR="$HOME/datasets/uzhfpv-db3"
export CONFIG_DIR="$CATKIN/src/open_vins/config/$CFGNAME"
export BASE_CONFIG="$CONFIG_DIR/estimator_config.yaml"

set +u
source "$CATKIN/scripts/bench_lib.sh"
source_ros
set -u

BAG="$DATASETS_DIR/$SEQ"                 # ROS 2 .db3 dir
[ -d "$BAG" ] || { echo "ERROR: db3 bag dir $BAG missing (run rosbags-convert first)" >&2; exit 1; }
[ -f "$BASE_CONFIG" ] || { echo "ERROR: config $BASE_CONFIG missing" >&2; exit 1; }

CFG="$CONFIG_DIR/vioeval_ov_${THR}thr.yaml"
trap 'rm -f "$CFG"' EXIT
make_bench_config "$CFG" "" "$THR"

OUT="$HOME/results/openvins/x86/native_jazzy/$TAG"
mkdir -p "$OUT"
echo "[OV-UZH $SEQ] serial, threads=$THR; reps=$REPS; cfg=$CFGNAME; out=$OUT"

for i in $(seq 0 $((REPS-1))); do
  if [ -f "$OUT/${SEQ}_rep${i}_est.txt" ]; then
    echo "[OV-UZH $SEQ] rep $i already present — skipping (idempotent)"; continue
  fi
  rm -f "$TIMING_WALL_TMP" "$TIMING_CPU_TMP" "$TIMING_THREAD_TMP" "$FEATS_TMP" "$EST_TMP" "$STD_TMP"
  echo "[OV-UZH $SEQ] rep $i ..."
  /usr/bin/time -v -o /tmp/vioeval_ov_uzh_time.log \
    ros2 launch ov_msckf serial.launch.py \
      config_path:="$CFG" path_bag:="$BAG" \
      max_cameras:=2 use_stereo:=true save_total_state:=true \
      filepath_est:="$EST_TMP" filepath_std:="$STD_TMP" \
    > "$OUT/${SEQ}_rep${i}_stdout.log" 2>&1 \
    || echo "[OV-UZH $SEQ] rep $i: launch exited nonzero (may have diverged)"
  if [ -f "$EST_TMP" ]; then
    cp "$EST_TMP" "$OUT/${SEQ}_rep${i}_est.txt"
    [ -f "$TIMING_WALL_TMP" ] && cp "$TIMING_WALL_TMP" "$OUT/${SEQ}_rep${i}_wall.txt"
    [ -f "$TIMING_CPU_TMP" ]  && cp "$TIMING_CPU_TMP"  "$OUT/${SEQ}_rep${i}_cpu.txt"
  else
    echo "[OV-UZH $SEQ] rep $i: NO estimate produced — writing empty result"
    : > "$OUT/${SEQ}_rep${i}_est.txt"
  fi
  awk '
    /Maximum resident set size/ {rss=$NF}
    /User time/                 {usr=$NF}
    /System time/               {sys=$NF}
    /Percent of CPU/            {cpu=$NF; gsub(/%/,"",cpu)}
    /Elapsed .wall clock/       {split($NF,a,":"); wall=(length(a)==3)?a[1]*3600+a[2]*60+a[3]:a[1]*60+a[2]}
    END {printf "peak_rss_kb,user_s,sys_s,pct_cpu,wall_s\n%s,%s,%s,%s,%s\n",rss,usr,sys,cpu,wall}
  ' /tmp/vioeval_ov_uzh_time.log > "$OUT/${SEQ}_rep${i}_proc.csv"
done
echo "[OV-UZH $SEQ] done ($REPS reps)."
