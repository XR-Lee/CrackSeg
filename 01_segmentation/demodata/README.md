# Crack Segmentation Demo Scripts

This folder contains lightweight demo scripts for the crack segmentation stage. In the public release, the demo images, DeepLabV3+ masks, refined masks, and optional SAM checkpoint live in the companion data package:

```text
CCIE2026_Crack3D_demo_data/
|-- segmentation_demo/data/
|   |-- images/
|   |-- mask35/
|   `-- refine35/
|-- models/checkpoint35.pth.tar
`-- models/sam_vit_h_4b8939.pth
```

The SAM refinement script mirrors `../SAM_refine/refine_masks.py`: it dilates the coarse DeepLabV3+ mask, finds connected crack clusters, crops local image patches with a 200 px margin, prompts SAM with EDT-derived points, and rejects over-wide SAM masks by EDT width evaluation.

The demo uses three representative samples:

- `1710490542.876314`
- `1710833369.347823`
- `1715852650.974332`

## Environment

Use Python 3.9 or 3.10.

```bash
conda create -n crackseg python=3.10 -y
conda activate crackseg
```

For viewing the included results only:

```bash
pip install -r 01_segmentation/demodata/requirements_visual.txt
```

For re-running SAM refinement, install PyTorch first. Choose the command matching your machine:

```bash
# CUDA 11.8
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

```bash
# CUDA 12.1
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

```bash
# CPU only
pip install torch torchvision
```

Then install the remaining demo dependencies:

```bash
pip install -r 01_segmentation/demodata/requirements_demo.txt
```

## Visualize Included Results

From the repository root:

```bash
python 01_segmentation/demodata/visualize_comparison.py \
  --data-root ../CCIE2026_Crack3D_demo_data/segmentation_demo/data \
  --output ../CCIE2026_Crack3D_demo_data/segmentation_demo/outputs/comparison_grid.png
```

Or use the wrapper:

```bash
bash scripts/run_segmentation_demo.sh ../CCIE2026_Crack3D_demo_data
```

The figure columns are:

1. Original RGB image.
2. DeepLabV3+ coarse mask overlay.
3. SAM-refined mask overlay.
4. Pixels added by SAM refinement.

Expected console output:

```text
sample,deeplab_pixels,refined_pixels,added_pixels,removed_pixels
1710490542.876314,85974,204226,118252,0
1710833369.347823,24330,43408,19078,0
1715852650.974332,26782,56239,29457,0
```

## Re-run SAM Refinement

If the checkpoint is already in the companion data package, run:

```bash
python 01_segmentation/demodata/run_sam_refine.py \
  --data-root ../CCIE2026_Crack3D_demo_data/segmentation_demo/data \
  --sam-checkpoint ../CCIE2026_Crack3D_demo_data/models/sam_vit_h_4b8939.pth \
  --device cuda:0
```

Outputs are written under the data package:

```text
CCIE2026_Crack3D_demo_data/segmentation_demo/data/refine35_rerun/
CCIE2026_Crack3D_demo_data/segmentation_demo/data/refinevis35_rerun/
```

If the checkpoint is missing, download it with:

```bash
python 01_segmentation/demodata/download_sam_checkpoint.py \
  --model-type vit_h \
  --output-dir ../CCIE2026_Crack3D_demo_data/models
```

The checkpoint is large and should be distributed through GitHub Releases, Zenodo, Git LFS, or another dataset host instead of being committed as a normal Git file.

## Troubleshooting

If `segment_anything` is missing:

```bash
pip install segment-anything
```

If no matching samples are found, check that filenames share the same stem:

```text
images/1710490542.876314.jpg
mask35/1710490542.876314.png
refine35/1710490542.876314.png
```

The bundled `mask35/` and `refine35/` are generated from the release DeepLab checkpoint
`models/checkpoint35.pth.tar` and the SAM ViT-H checkpoint. Small differences from
`refine35/` may occur across SAM, PyTorch, and CUDA versions.

## Re-run the Full Segmentation Chain

From the repository root:

```bash
bash scripts/run_segmentation_full_demo.sh ../CCIE2026_Crack3D_demo_data python3 cuda:0
```

This runs:

1. `01_segmentation/DeepLab/eval_test.py` on `segmentation_demo/data/images/`.
2. `01_segmentation/demodata/run_sam_refine.py` on the generated `mask35_rerun/`.
3. `01_segmentation/demodata/visualize_comparison.py` on the generated `refine35_rerun/`.

Generated outputs are written to:

```text
CCIE2026_Crack3D_demo_data/segmentation_demo/data/mask35_rerun/
CCIE2026_Crack3D_demo_data/segmentation_demo/data/refine35_rerun/
CCIE2026_Crack3D_demo_data/segmentation_demo/data/refinevis35_rerun/
CCIE2026_Crack3D_demo_data/segmentation_demo/outputs/comparison_grid_rerun.png
```
