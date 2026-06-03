#!/usr/bin/env bash
# OpenVINS runner for vio-eval (serial mode, configurable threads).
#
# Unlike the catkin run_full_benchmark.sh (which derives CPU% from OpenVINS' own
# per-frame CPU CSV), this wraps the estimator launch in `/usr/bin/time -v` — the
# SAME whole-process measurement used by run_orb_slam3.sh — so CPU% and peak RSS
# are directly comparable across systems. Reuses bench_lib.sh for source_ros,
# make_bench_config, and the fixed /tmp output paths.
#
# Writes ~/results/openvins/x86/native_jazzy/<tag>/ :
#   <seq>_est.txt   OpenVINS state-dump trajectory (accuracy via _est_to_tum)
#   <seq>_wall.txt  per-frame wall timing (latency/FPS; OpenVINS column format)
#   <seq>_cpu.txt   per-frame CPU timing (kept for reference)
#   <seq>_proc.csv  /usr/bin/time -v: peak_rss_kb,user_s,sys_s,pct_cpu,wall_s
#   <seq>_stdout.log
set -euo pipefail

usage() { echo "usage: run_openvins.sh <seq> [--tag TAG] [--thr N] [--reps N]"; exit 1; }
[ $# -ge 1 ] || usage
SEQ="$1"; shift
TAG="baseline_x86"; THR=4; REPS=1
while [ $# -gt 0 ]; do
  case "$1" in
    --tag)  TAG="$2"; shift 2;;
    --thr)  THR="$2"; shift 2;;
    --reps) REPS="$2"; shift 2;;
    *) usage;;
  esac
done

CATKIN="$HOME/workspace/catkin_ws_ov"
export WS_DIR="$CATKIN"
# bench_lib.sh / ROS setup touch unset vars; relax -u while sourcing.
set +u
source "$CATKIN/scripts/bench_lib.sh"
source_ros
set -u

BAG="$DATASETS_DIR/$SEQ"                 # ~/datasets/euroc/<seq> (.db3 dir)
[ -d "$BAG" ] || { echo "ERROR: bag dir $BAG missing" >&2; exit 1; }

# Generate the bench config IN CONFIG_DIR (it references sibling calib YAMLs by
# relative path, so it cannot live in /tmp). Timing recording + thread count are
# set by make_bench_config. Cleaned up on exit.
CFG="$CONFIG_DIR/vioeval_ov_${THR}thr.yaml"
trap 'rm -f "$CFG"' EXIT
make_bench_config "$CFG" "" "$THR"

OUT="$HOME/results/openvins/x86/native_jazzy/$TAG"
mkdir -p "$OUT"
echo "[OV $SEQ] serial, threads=$THR; reps=$REPS; out=$OUT"

for i in $(seq 0 $((REPS-1))); do
  rm -f "$TIMING_WALL_TMP" "$TIMING_CPU_TMP" "$TIMING_THREAD_TMP" "$FEATS_TMP" "$EST_TMP" "$STD_TMP"
  echo "[OV $SEQ] rep $i ..."
  /usr/bin/time -v -o /tmp/vioeval_ov_time.log \
    ros2 launch ov_msckf serial.launch.py \
      config_path:="$CFG" path_bag:="$BAG" \
      max_cameras:=2 use_stereo:=true save_total_state:=true \
      filepath_est:="$EST_TMP" filepath_std:="$STD_TMP" \
    > "$OUT/${SEQ}_rep${i}_stdout.log" 2>&1
  [ -f "$EST_TMP" ] || { echo "ERROR: no estimate produced (see ${SEQ}_rep${i}_stdout.log)" >&2; exit 1; }
  cp "$EST_TMP" "$OUT/${SEQ}_rep${i}_est.txt"
  cp "$TIMING_WALL_TMP" "$OUT/${SEQ}_rep${i}_wall.txt"
  cp "$TIMING_CPU_TMP"  "$OUT/${SEQ}_rep${i}_cpu.txt"
  awk '
    /Maximum resident set size/ {rss=$NF}
    /User time/                 {usr=$NF}
    /System time/               {sys=$NF}
    /Percent of CPU/            {cpu=$NF; gsub(/%/,"",cpu)}
    /Elapsed .wall clock/       {split($NF,a,":"); wall=(length(a)==3)?a[1]*3600+a[2]*60+a[3]:a[1]*60+a[2]}
    END {printf "peak_rss_kb,user_s,sys_s,pct_cpu,wall_s\n%s,%s,%s,%s,%s\n",rss,usr,sys,cpu,wall}
  ' /tmp/vioeval_ov_time.log > "$OUT/${SEQ}_rep${i}_proc.csv"
done
echo "[OV $SEQ] done ($REPS reps)."
