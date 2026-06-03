#!/usr/bin/env bash
# Basalt VIO runner for vio-eval (EuRoC ASL, offline, headless).
#
# Wraps basalt_vio in /usr/bin/time -v (same whole-process method as the other
# systems) for CPU% / peak RSS. Writes ~/results/basalt/x86/native_jazzy/<tag>/ :
#   <seq>_rep<i>_trajectory.txt  canonical TUM (seconds) via basalt_to_tum.py
#   <seq>_rep<i>_proc.csv        peak_rss_kb,user_s,sys_s,pct_cpu,wall_s
#   <seq>_rep<i>_stdout.log      full Basalt output
#
# Basalt is pure VIO (sliding-window, no loop closure). Build is vcpkg-based;
# its shared deps live under build/release/vcpkg_installed/x64-linux/lib.
set -euo pipefail

usage() { echo "usage: run_basalt.sh <seq> [--reps N] [--tag TAG] [--threads N]"; exit 1; }
[ $# -ge 1 ] || usage
SEQ="$1"; shift
REPS=1; TAG="baseline_x86"; THREADS=0   # 0 = Basalt default
while [ $# -gt 0 ]; do
  case "$1" in
    --reps)    REPS="$2"; shift 2;;
    --tag)     TAG="$2";  shift 2;;
    --threads) THREADS="$2"; shift 2;;
    *) usage;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # scripts/
REPO="$(cd "$HERE/.." && pwd)"                                # vio-evaluation
BAS="$REPO/systems/basalt"                                    # submodule
BIN="$BAS/build/release/basalt_vio"
CALIB="$BAS/data/euroc_ds_calib.json"
CONFIG="$BAS/data/euroc_config.json"
ASL="$HOME/datasets/euroc-asl/$SEQ"
GT="$HOME/workspace/catkin_ws_ov/src/open_vins/ov_data/euroc_mav/$SEQ.txt"
ADAPTER="$HERE/adapters/basalt_to_tum.py"
TIMING="$HERE/adapters/basalt_timing.py"

for p in "$BIN" "$CALIB" "$CONFIG" "$ASL/mav0/cam0/data" "$GT" "$ADAPTER" "$TIMING"; do
  [ -e "$p" ] || { echo "ERROR: missing $p" >&2; exit 1; }
done

OUT="$HOME/results/basalt/x86/native_jazzy/$TAG"
mkdir -p "$OUT"
export LD_LIBRARY_PATH="$BAS/build/release/vcpkg_installed/x64-linux/lib:${LD_LIBRARY_PATH:-}"

thr_args=(); [ "$THREADS" != 0 ] && thr_args=(--num-threads "$THREADS")
echo "[BASALT $SEQ] reps=$REPS; threads=${THREADS:-default}; out=$OUT"

for i in $(seq 0 $((REPS-1))); do
  WORK="$(mktemp -d)"
  echo "[BASALT $SEQ] rep $i ..."
  ( cd "$WORK" && /usr/bin/time -v -o time.log \
      "$BIN" --dataset-path "$ASL" --dataset-type euroc \
             --cam-calib "$CALIB" --config-path "$CONFIG" \
             --show-gui 0 --save-trajectory tum "${thr_args[@]}" >stdout.log 2>&1 )
  [ -f "$WORK/trajectory.txt" ] || { echo "ERROR: no trajectory (see stdout.log)" >&2; cat "$WORK/stdout.log" | tail -15; rm -rf "$WORK"; exit 1; }
  python3 "$ADAPTER" "$WORK/trajectory.txt" "$OUT/${SEQ}_rep${i}_trajectory.txt" "$GT"
  [ -f "$WORK/stats_sums.ubjson" ] && python3 "$TIMING" "$WORK/stats_sums.ubjson" "$OUT/${SEQ}_rep${i}_timing.csv"
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
echo "[BASALT $SEQ] done."
