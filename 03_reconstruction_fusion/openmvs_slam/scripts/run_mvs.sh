#!/bin/bash
set -euo pipefail

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    echo "Usage: $0 /mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/MVS_Workspace /opt/openMVS/build/bin [mvs_frame_result.txt]"
    exit 1
fi

WORK_DIR="$1"
CMD_DIR="$2"
INPUT_MVS_FRAME="${3:-$WORK_DIR/mvs_frame_result.txt}"

DENSIFY_OUTPUT="$WORK_DIR/mvs_densify.mvs"
RECONSTRUCT_OUTPUT="$WORK_DIR/mvs_reconstruct.mvs"
REFINE_OUTPUT="$WORK_DIR/mvs_refine.mvs"
TEXTURE_OUTPUT="$WORK_DIR/mvs_texture.mvs"

echo "Running DensifyPointCloud..."
"$CMD_DIR/DensifyPointCloud" -w "$WORK_DIR" -i "$INPUT_MVS_FRAME" -o "$DENSIFY_OUTPUT"

echo "Running ReconstructMesh..."
"$CMD_DIR/ReconstructMesh" -w "$WORK_DIR" -i "$DENSIFY_OUTPUT" -o "$RECONSTRUCT_OUTPUT"

echo "Running RefineMesh..."
"$CMD_DIR/RefineMesh" -w "$WORK_DIR" -i "$RECONSTRUCT_OUTPUT" -o "$REFINE_OUTPUT"

echo "Running TextureMesh..."
"$CMD_DIR/TextureMesh" -w "$WORK_DIR" -i "$REFINE_OUTPUT" -o "$TEXTURE_OUTPUT"

echo "All OpenMVS stages completed."
