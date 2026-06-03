# Building Basalt (x86, Ubuntu 24.04 / gcc-13, vcpkg)

Pinned to `NadavHHailo/basalt` `master` (`0f3b2b5`), built natively. Patches live on
branch `vio-eval-build`. Modern Basalt uses **vcpkg manifest mode** (all deps built from
source) — not the older system-deps + submodule build. Build is heavy (~30–60 min, several
GB) but fully reproducible and self-contained.

## Prerequisites (already present on this machine)
gcc 13.3, cmake ≥ 3.24 (have 3.28), **ninja**, X11/GL dev libs (libgl-dev, libglew-dev,
libx11-dev, libxrandr/xinerama/cursor/i-dev). No `libudev-dev` (see realsense patch). vcpkg
auto-fetches nasm/yasm itself; no sudo needed.

## Patches (branch `vio-eval-build`, off `master`)
- **`vcpkg.json`** — remove the **`realsense2`** dependency (and its version override).
  RealSense is only for live cameras (EuRoC is offline); Basalt's CMake already guards it
  with `find_package(realsense2 QUIET)`. It pulled `libusb[udev]`, which fails to build
  without system `libudev-dev` (no sudo) — removing realsense2 drops that whole chain.
- **`CMakeLists.txt`** — drop `-Werror` from `BASALT_CXX_FLAGS` (gcc-13 raises new
  `-Wunused-variable` warnings in `dataset_io_*.h`).

## Build
```bash
cd systems/basalt
git submodule update --init --recursive          # clones thirdparty/vcpkg
./thirdparty/vcpkg/bootstrap-vcpkg.sh -disableMetrics
cmake --preset release                            # vcpkg builds all deps, then configures Basalt
cmake --build build/release -j8                   # builds basalt_vio (+ tools)
```
vcpkg installs into `build/release/vcpkg_installed/x64-linux/` (OpenCV 4.12, Boost 1.90,
Pangolin 0.9.4, opengv, TBB, fmt, lz4, …). Artifact: `build/release/basalt_vio` (~6 MB).

## Running (headless, EuRoC ASL)
```
basalt_vio --dataset-path ~/datasets/euroc-asl/<seq> --dataset-type euroc \
  --cam-calib data/euroc_ds_calib.json --config-path data/euroc_config.json \
  --show-gui 0 --save-trajectory tum
```
- Writes `trajectory.txt` (TUM, timestamps already in **seconds** → adapter is a no-op rename).
- Set `LD_LIBRARY_PATH` to include `build/release/vcpkg_installed/x64-linux/lib`.
- Per-frame timing is logged to `stats_sums.ubjson` (key `measure` = full per-frame VIO
  latency, `optimize` = backend). [`scripts/adapters/basalt_timing.py`](../scripts/adapters/basalt_timing.py)
  converts it to the canonical `timing.csv` (needs `py-ubjson`).
- See [`scripts/run_basalt.sh`](../scripts/run_basalt.sh) for the full invocation
  (wrapped in `/usr/bin/time -v` for CPU%/RSS, same method as the other systems).

## Validation (V1_01_easy, evo SE3 align)
ATE-trans = **0.030 m** (evo + ov_eval agree). Basalt is pure VIO (no loop closure),
emits a pose for every frame (completeness 100%, init ~0 s), is the lightest on memory
(~90 MB) and fastest (~110 FPS) but the most parallel (~13 cores, TBB).
