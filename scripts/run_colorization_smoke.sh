#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 ../CCIE2026_Crack3D_demo_data [PointCloudProcessor_binary]" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${1%/}/colorization_smoke"
BIN_ARG=()

if [[ $# -ge 2 ]]; then
  BIN_ARG=(--binary "$2")
fi

bash "$REPO_ROOT/04_pointcloud_colorization/run_pcd_colorization.sh" \
  --data-root "$DATA_ROOT" \
  --point-cloud "$DATA_ROOT/scans-crop.pcd" \
  --odom "$DATA_ROOT/vo_interpolated_odom.txt" \
  --images "$DATA_ROOT/raw_images" \
  --output "$DATA_ROOT/output" \
  --enable-mls 0 \
  --enable-nid 0 \
  --enable-manual 0 \
  "${BIN_ARG[@]}"
