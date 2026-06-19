#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 ../CCIE2026_Crack3D_demo_data [python]" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${1%/}/width_measurement_demo"
PYTHON_BIN="${2:-python3}"

"$PYTHON_BIN" "$REPO_ROOT/05_crack_width_measurement/genNormAndDistanceMask.py" \
  --data-root "$DATA_ROOT" \
  --camera-config "$REPO_ROOT/configs/camera_4096x3000_paper.json" \
  --stage all \
  --selected-points-json "$DATA_ROOT/selected_points.json" \
  --log-level INFO
