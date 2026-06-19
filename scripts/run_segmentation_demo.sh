#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 ../CCIE2026_Crack3D_demo_data [python]" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${1%/}/segmentation_demo/data"
PYTHON_BIN="${2:-python3}"

"$PYTHON_BIN" "$REPO_ROOT/01_segmentation/demodata/visualize_comparison.py" \
  --data-root "$DATA_ROOT" \
  --output "$DATA_ROOT/../outputs/comparison_grid.png"
