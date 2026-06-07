#!/usr/bin/env bash
# Basalt VIO runner for the UZH-FPV drone dataset (fisheye), offline, headless.
#
# Same measurement method as run_basalt.sh (wraps basalt_vio in /usr/bin/time -v
# for CPU% / peak RSS; reuses basalt_to_tum.py + basalt_timing.py adapters). The
# only differences from the EuRoC runner are dataset-specific:
#   * the UZH bag is extracted to a EuRoC ASL folder (uzh_bag_to_asl.py), read with
#     --dataset-type euroc — the same folder also feeds ORB-SLAM3 / OpenVINS.
#   * a FISHEYE calibration (--cam-calib): EuRoC's pinhole calib does NOT apply.
#     UZH-FPV (Snapdragon rig) is equidistant/double-sphere — supply a kb4/ds/eucm
#     calib JSON (model after data/t265_kb4_calib.json from the UZH Kalibr file).
#   * ground truth is the UZH GT converted to TUM via scripts/uzh_gt_to_tum.py.
#
# Writes ~/results/basalt/x86/native_jazzy/<tag>/ (default tag uzhfpv_x86):
#   <seq>_rep<i>_trajectory.txt  canonical TUM (seconds)
#   <seq>_rep<i>_timing.csv      per-frame VIO update time
#   <seq>_rep<i>_proc.csv        peak_rss_kb,user_s,sys_s,pct_cpu,wall_s
#   <seq>_rep<i>_stdout.log
set -euo pipefail

usage() {
  echo "usage: run_basalt_uzh.sh <seq> [--reps N] [--tag TAG] [--threads N]"
  echo "                         [--dataset-path DIR] [--calib JSON] [--config JSON] [--gt FILE]"
  echo "  <seq>  e.g. indoor_45_2_snapdragon_with_gt"
  exit 1
}
[ $# -ge 1 ] || usage
SEQ="$1"; shift
REPS=1; TAG="uzhfpv_x86"; THREADS=0

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # scripts/
REPO="$(cd "$HERE/.." && pwd)"                                # vio-evaluation
BAS="$REPO/systems/basalt"                                    # submodule

# Defaults are overridable; the calib MUST be a fisheye calib you create from the
# UZH Kalibr camchain (see docs/uzhfpv.md). euroc_config.json is camera-agnostic
# (algorithm params only), so it is a reasonable starting config.
DSPATH="$HOME/datasets/uzhfpv-asl/$SEQ"
CALIB="$BAS/data/uzh_ds_calib.json"
CONFIG="$BAS/data/euroc_config.json"
GT="$HOME/datasets/uzhfpv-gt/$SEQ.txt"

while [ $# -gt 0 ]; do
  case "$1" in
    --reps)         REPS="$2"; shift 2;;
    --tag)          TAG="$2";  shift 2;;
    --threads)      THREADS="$2"; shift 2;;
    --dataset-path) DSPATH="$2"; shift 2;;
    --calib)        CALIB="$2"; shift 2;;
    --config)       CONFIG="$2"; shift 2;;
    --gt)           GT="$2"; shift 2;;
    *) usage;;
  esac
done

BIN="$BAS/build/release/basalt_vio"
ADAPTER="$HERE/adapters/basalt_to_tum.py"
TIMING="$HERE/adapters/basalt_timing.py"

for p in "$BIN" "$CALIB" "$CONFIG" "$DSPATH" "$GT" "$ADAPTER" "$TIMING"; do
  [ -e "$p" ] || { echo "ERROR: missing $p" >&2; exit 1; }
done

OUT="$HOME/results/basalt/x86/native_jazzy/$TAG"
mkdir -p "$OUT"
export LD_LIBRARY_PATH="$BAS/build/release/vcpkg_installed/x64-linux/lib:${LD_LIBRARY_PATH:-}"

thr_args=(); [ "$THREADS" != 0 ] && thr_args=(--num-threads "$THREADS")
echo "[BASALT-UZH $SEQ] reps=$REPS; threads=${THREADS:-default}; calib=$CALIB; out=$OUT"

for i in $(seq 0 $((REPS-1))); do
  if [ -f "$OUT/${SEQ}_rep${i}_trajectory.txt" ]; then
    echo "[BASALT-UZH $SEQ] rep $i already present — skipping (idempotent)"; continue
  fi
  WORK="$(mktemp -d)"
  echo "[BASALT-UZH $SEQ] rep $i ..."
  ( cd "$WORK" && /usr/bin/time -v -o time.log \
      "$BIN" --dataset-path "$DSPATH" --dataset-type euroc \
             --cam-calib "$CALIB" --config-path "$CONFIG" \
             --show-gui 0 --save-trajectory tum "${thr_args[@]}" >stdout.log 2>&1 ) \
    || { echo "[BASALT-UZH $SEQ] rep $i: basalt_vio exited nonzero (may have diverged)"; }
  if [ -f "$WORK/trajectory.txt" ]; then
    python3 "$ADAPTER" "$WORK/trajectory.txt" "$OUT/${SEQ}_rep${i}_trajectory.txt" "$GT"
    [ -f "$WORK/stats_sums.ubjson" ] && python3 "$TIMING" "$WORK/stats_sums.ubjson" "$OUT/${SEQ}_rep${i}_timing.csv"
  else
    # UZH-FPV is aggressive — divergence/no-trajectory is an expected outcome on the
    # hard sequences. Record it as an empty trajectory so compare_report.py reports
    # 0% completeness rather than silently dropping the run.
    echo "[BASALT-UZH $SEQ] rep $i: NO trajectory produced (diverged?) — writing empty result"
    : > "$OUT/${SEQ}_rep${i}_trajectory.txt"
  fi
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
echo "[BASALT-UZH $SEQ] done."
