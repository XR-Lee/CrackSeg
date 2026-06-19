#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 ../CCIE2026_Crack3D_demo_data [python] [device]" >&2
  echo "Example: $0 ../CCIE2026_Crack3D_demo_data python3 cuda:0" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO_ROOT="${1%/}"
PYTHON_BIN="${2:-python3}"
DEVICE="${3:-cuda:0}"

DATA_ROOT="$DEMO_ROOT/segmentation_demo/data"
DEEPLAB_CHECKPOINT="$DEMO_ROOT/models/checkpoint35.pth.tar"
SAM_CHECKPOINT="$DEMO_ROOT/models/sam_vit_h_4b8939.pth"

if [[ ! -f "$DEEPLAB_CHECKPOINT" ]]; then
  echo "DeepLab checkpoint not found: $DEEPLAB_CHECKPOINT" >&2
  exit 1
fi

if [[ ! -f "$SAM_CHECKPOINT" ]]; then
  echo "SAM checkpoint not found: $SAM_CHECKPOINT" >&2
  exit 1
fi

"$PYTHON_BIN" "$REPO_ROOT/01_segmentation/DeepLab/eval_test.py" \
  --checkpoint "$DEEPLAB_CHECKPOINT" \
  --input-dir "$DATA_ROOT/images" \
  --output-dir "$DATA_ROOT/mask35_rerun" \
  --device "$DEVICE"

"$PYTHON_BIN" "$REPO_ROOT/01_segmentation/demodata/run_sam_refine.py" \
  --data-root "$DATA_ROOT" \
  --mask-folder mask35_rerun \
  --output-mask-folder refine35_rerun \
  --output-vis-folder refinevis35_rerun \
  --sam-checkpoint "$SAM_CHECKPOINT" \
  --device "$DEVICE"

"$PYTHON_BIN" "$REPO_ROOT/01_segmentation/demodata/visualize_comparison.py" \
  --data-root "$DATA_ROOT" \
  --mask-folder mask35_rerun \
  --refine-folder refine35_rerun \
  --output "$DEMO_ROOT/segmentation_demo/outputs/comparison_grid_rerun.png"
