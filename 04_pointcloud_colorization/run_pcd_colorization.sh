#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_pcd_colorization.sh --data-root DATASET_DIR [options]

Options:
  --data-root DIR       Dataset folder containing scans-crop.pcd, raw_images/, and odometry.
  --point-cloud FILE    Input point cloud. Default: DATASET_DIR/scans-crop.pcd, then ronghe.pcd, then scans.pcd.
  --odom FILE           Interpolated odometry txt. Default: DATASET_DIR/vo_interpolated_odom.txt.
  --images DIR          Timestamp-named JPG image folder. Default: DATASET_DIR/raw_images.
  --masks DIR           Optional timestamp-named PNG crack-mask folder. Empty disables mask projection.
  --output DIR          Output folder. Default: DATASET_DIR.
  --binary FILE         Built PointCloudProcessor binary. Default: 04_pointcloud_colorization/build/PointCloudProcessor.
  --enable-mls 0|1      Enable MLS smoothing. Default: 0.
  --enable-nid 0|1      Enable NID pose optimization. Default: 0.
  --enable-manual 0|1   Enable manual initial-guess GUI. Default: 0.
  --help                Show this help.

If --odom is missing and DATASET_DIR has visual_odom.txt plus visual_odom_in_lidar_ts.txt,
the script runs PointCloudProcessor/scripts/make_vo_odom_for_fastlio.py first.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DATA_ROOT=""
POINT_CLOUD_PATH=""
ODOMETRY_PATH=""
IMAGES_FOLDER=""
MASK_IMAGE_FOLDER=""
OUTPUT_PATH=""
BIN_PATH="$SCRIPT_DIR/build/PointCloudProcessor"
ENABLE_MLS=0
ENABLE_NID_OPTIMIZE=0
ENABLE_INITIAL_GUESS_MANUAL=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --data-root) DATA_ROOT="$2"; shift 2 ;;
    --point-cloud) POINT_CLOUD_PATH="$2"; shift 2 ;;
    --odom) ODOMETRY_PATH="$2"; shift 2 ;;
    --images) IMAGES_FOLDER="$2"; shift 2 ;;
    --masks) MASK_IMAGE_FOLDER="$2"; shift 2 ;;
    --output) OUTPUT_PATH="$2"; shift 2 ;;
    --binary) BIN_PATH="$2"; shift 2 ;;
    --enable-mls) ENABLE_MLS="$2"; shift 2 ;;
    --enable-nid) ENABLE_NID_OPTIMIZE="$2"; shift 2 ;;
    --enable-manual) ENABLE_INITIAL_GUESS_MANUAL="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
done

if [[ -z "$DATA_ROOT" ]]; then
  echo "Missing required --data-root" >&2
  usage >&2
  exit 1
fi

DATA_ROOT="${DATA_ROOT%/}"
IMAGES_FOLDER="${IMAGES_FOLDER:-$DATA_ROOT/raw_images}"
ODOMETRY_PATH="${ODOMETRY_PATH:-$DATA_ROOT/vo_interpolated_odom.txt}"
OUTPUT_PATH="${OUTPUT_PATH:-$DATA_ROOT}"

if [[ -z "$POINT_CLOUD_PATH" ]]; then
  for candidate in "$DATA_ROOT/scans-crop.pcd" "$DATA_ROOT/ronghe.pcd" "$DATA_ROOT/scans.pcd"; do
    if [[ -f "$candidate" ]]; then
      POINT_CLOUD_PATH="$candidate"
      break
    fi
  done
fi

if [[ -z "$MASK_IMAGE_FOLDER" && -d "$DATA_ROOT/masks" ]]; then
  MASK_IMAGE_FOLDER="$DATA_ROOT/masks"
fi

for path in "$DATA_ROOT" "$IMAGES_FOLDER" "$OUTPUT_PATH"; do
  mkdir -p "$path"
done

if [[ -z "$POINT_CLOUD_PATH" || ! -f "$POINT_CLOUD_PATH" ]]; then
  echo "Point cloud not found. Pass --point-cloud or provide scans-crop.pcd/ronghe.pcd/scans.pcd under --data-root." >&2
  exit 1
fi

ODOM_SCRIPT="$SCRIPT_DIR/scripts/make_vo_odom_for_fastlio.py"
if [[ ! -f "$ODOMETRY_PATH" ]]; then
  if [[ -f "$ODOM_SCRIPT" && -f "$DATA_ROOT/visual_odom.txt" && -f "$DATA_ROOT/visual_odom_in_lidar_ts.txt" ]]; then
    python3 "$ODOM_SCRIPT" --root_dir "$DATA_ROOT"
  else
    echo "Odometry not found: $ODOMETRY_PATH" >&2
    echo "Provide --odom or place visual_odom.txt and visual_odom_in_lidar_ts.txt under --data-root." >&2
    exit 1
  fi
fi

if [[ ! -x "$BIN_PATH" ]]; then
  echo "PointCloudProcessor binary not found or not executable: $BIN_PATH" >&2
  echo "Build it first with: cd 04_pointcloud_colorization && mkdir -p build && cd build && cmake .. && make -j\$(nproc)" >&2
  exit 1
fi

cmd=(
  "$BIN_PATH"
  --point_cloud_path "$POINT_CLOUD_PATH"
  --odometry_path "$ODOMETRY_PATH"
  --images_folder "${IMAGES_FOLDER%/}/"
  --output_path "${OUTPUT_PATH%/}/"
  --enableMLS "$ENABLE_MLS"
  --enableNIDOptimize "$ENABLE_NID_OPTIMIZE"
  --enableInitialGuessManual "$ENABLE_INITIAL_GUESS_MANUAL"
)

if [[ -n "$MASK_IMAGE_FOLDER" ]]; then
  cmd+=(--mask_image_folder "${MASK_IMAGE_FOLDER%/}/")
fi

echo "Running PointCloudProcessor:"
printf '  %q' "${cmd[@]}"
echo
"${cmd[@]}"
