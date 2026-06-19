# CrackModel0823

Code for the crack image segmentation stage of:

> 3D modeling and automated measurement of concrete cracks via segment anything refinement and visual inertial LiDAR fusion

The segmentation pipeline first produces coarse crack masks with DeepLabV3+, then refines the masks with Segment Anything Model (SAM) prompts generated from the coarse masks.

## Repository Layout

```text
01_segmentation/
  DeepLab/            DeepLabV3+ training and folder inference code
  SAM_refine/         SAM-based coarse-mask refinement code
  demodata/           Self-contained SAM-refinement demo + visualization
```

Large files such as datasets, checkpoints, TensorBoard logs, and generated outputs are intentionally excluded by `.gitignore`. Publish trained weights separately with GitHub Releases, Zenodo, or another artifact host.

The public demo assumes the companion data folder is next to this repository:

```bash
DEMO_DATA=../CCIE2026_Crack3D_demo_data
```

Commands below are written from the repository root unless a `cd` command is shown.

## Installation

Create a Python environment and install the dependencies:

```bash
pip install -r requirements-segmentation.txt
```

Install PyTorch for your CUDA version from the official PyTorch instructions if the default pip package does not match your GPU environment.

Download the external model weights separately:

- DeepLab crack checkpoint: use the paper demo checkpoint, `CCIE2026_Crack3D_demo_data/models/checkpoint35.pth.tar`, or your own trained checkpoint.
- SAM checkpoint: download from the official Segment Anything release, for example `sam_vit_h_4b8939.pth`.

## Quick Demo

A lightweight, self-contained demo with three resized samples is provided in `demodata/`.
It visualizes the bundled DeepLabV3+ coarse masks and the corresponding SAM-refined masks without requiring a SAM checkpoint:

```bash
python demodata/visualize_comparison.py
```

The comparison figure is saved to:

```text
demodata/outputs/comparison_grid.png
```

See `demodata/README.md` for the full demo environment setup and optional SAM re-run command.

To rerun the complete demo from raw images to coarse DeepLab masks and SAM-refined masks:

```bash
bash scripts/run_segmentation_full_demo.sh ../CCIE2026_Crack3D_demo_data python3 cuda:0
```

## Dataset Format

For DeepLab training, set `CRACK_DATASET_ROOT` to a folder with this structure:

```text
${CRACK_DATASET_ROOT}/
  Dataset/
    Images/
      image_0001.png
    Masks/
      image_0001.png
    train.txt
    val.txt
```

Each line in `train.txt` and `val.txt` should be a filename shared by `Images/` and `Masks/`.

## DeepLab Inference

Run coarse crack segmentation on a folder of RGB images:

```bash
python 01_segmentation/DeepLab/eval_test.py \
  --checkpoint "$DEMO_DATA/models/checkpoint35.pth.tar" \
  --input-dir "$DEMO_DATA/segmentation_demo/data/images" \
  --output-dir "$DEMO_DATA/segmentation_demo/data/mask35_rerun" \
  --device cuda:0
```

The script writes one binary PNG mask per input image.

## SAM Refinement

Prepare a refinement folder containing images and their coarse masks:

```text
data/refine_case/
  crop/       RGB images
  maskcrop/   coarse masks from DeepLab
```

For the public demo data, run SAM refinement from `mask35_rerun/`:

```bash
python 01_segmentation/demodata/run_sam_refine.py \
  --data-root "$DEMO_DATA/segmentation_demo/data" \
  --mask-folder mask35_rerun \
  --output-mask-folder refine35_rerun \
  --output-vis-folder refinevis35_rerun \
  --sam-checkpoint "$DEMO_DATA/models/sam_vit_h_4b8939.pth" \
  --device cuda:0
```

Outputs:

- `refine35_rerun/`: refined binary crack masks.
- `refinevis35_rerun/`: four-panel visualizations for quality inspection.

## Training

Set the dataset root and run the DeepLab training script:

```bash
export CRACK_DATASET_ROOT="$(pwd)/../CCIE2026_Crack3D_demo_data/training_dataset_example"
cd 01_segmentation/DeepLab
bash train_crack.sh
```

For finetuning from an existing checkpoint:

```bash
export CRACK_DATASET_ROOT="$(pwd)/../CCIE2026_Crack3D_demo_data/training_dataset_example"
export RESUME_CHECKPOINT="$(pwd)/../CCIE2026_Crack3D_demo_data/models/checkpoint35.pth.tar"
cd 01_segmentation/DeepLab
bash train_crack_finetuning.sh
```

## Citation

If you use this code, please cite:

```bibtex
@article{deng2026crack3d,
  title = {3D modeling and automated measurement of concrete cracks via segment anything refinement and visual inertial LiDAR fusion},
  author = {Deng, Pengru and Yao, Jiapeng and Li, Chun and Wang, Su and Li, Xinrun and Ojha, Varun and He, Xuhui},
  journal = {Computer-Aided Civil and Infrastructure Engineering},
  volume = {45},
  pages = {100019},
  year = {2026},
  doi = {10.1016/j.cacaie.2026.100019}
}
```

## Acknowledgements

The DeepLab implementation is adapted from PyTorch DeepLabV3+ codebases, including `pytorch-deeplab-xception`. SAM refinement depends on Meta AI's Segment Anything implementation.

## License

This code release uses the Apache License, Version 2.0 (see the repository-root `LICENSE`). Check third-party dependencies and pretrained weights for their own licenses before redistribution.
