#!/usr/bin/env bash
# Publish the per-dataset camera-view MP4 previews:
#   1. push the branch + LFS objects (over SSH)
#   2. create the `dataset-previews` GitHub release and upload the 4 MP4s
# Prereq (one time): gh auth login   — or   export GH_TOKEN=<pat>
set -euo pipefail

REPO="NadavHHailo/vio-evaluation"
TAG="dataset-previews"
BRANCH="videos-dataset-previews"
cd "$(git rev-parse --show-toplevel)"

echo ">> pushing $BRANCH (LFS objects transfer over SSH)…"
git push -u origin "$BRANCH"

echo ">> creating release $TAG and uploading assets…"
gh release create "$TAG" \
  videos/V1_01_easy.mp4 \
  videos/MH_03_medium.mp4 \
  videos/V2_02_medium.mp4 \
  videos/indoor_45_2_snapdragon.mp4 \
  -R "$REPO" \
  --title "Dataset camera-view previews" \
  --notes "Left-camera (cam0) MP4 previews for the EuRoC sequences (V1_01_easy, MH_03_medium, V2_02_medium) and UZH-FPV indoor_45_2_snapdragon. Encoded from PNG frames only (no IMU), H.264 30 fps. Embedded as inline players in docs/comparison.md." \
  || gh release upload "$TAG" \
       videos/V1_01_easy.mp4 videos/MH_03_medium.mp4 \
       videos/V2_02_medium.mp4 videos/indoor_45_2_snapdragon.mp4 \
       -R "$REPO" --clobber

echo ">> done. Asset URLs:"
gh release view "$TAG" -R "$REPO" --json assets --jq '.assets[].url'
