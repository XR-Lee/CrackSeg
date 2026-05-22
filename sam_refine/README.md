# Crack Segmentation Demo Data

This folder is a self-contained demo for the crack segmentation stage. You can copy only this `sam_refine/` folder to another machine and run the demo scripts here without importing code from the rest of the repository.

The SAM refinement script mirrors the method in `SAM_refine/refine_masks.py`: it dilates the coarse DeepLabV3+ mask, finds connected crack clusters, crops local image patches with a 200 px margin, prompts SAM with EDT-derived points, and rejects over-wide SAM masks by EDT width evaluation.

The demo contains three representative samples:

- `1710490542.876314`
- `1710833369.347823`
- `1715852650.974332`

## Folder Structure

```text
sam_refine/
  checkpoints/  local SAM checkpoint, ignored by Git
  data/
    images/      RGB crack images
    mask35/      DeepLabV3+ 0-shot masks
    refine35/    provided SAM-refined masks
  visualize_comparison.py
  run_sam_refine.py
  download_sam_checkpoint.py
  requirements_visual.txt
  requirements_demo.txt
```

The images are the original 4096 x 3000 JPG samples from `test20241011new/images`; they are not resized or recompressed. Generated outputs and SAM checkpoints are ignored by Git because the checkpoint file is large.
For a local reproducibility package, keep `checkpoints/sam_vit_h_4b8939.pth` inside this folder. For GitHub, publish the checkpoint with GitHub Releases, Git LFS, Zenodo, or another artifact host instead of committing it as a normal Git file.

## 1. Create Environment

Use Python 3.9 or 3.10.

```bash
conda create -n crackdemo python=3.10 -y
conda activate crackdemo
```

## 2. Install Dependencies

For viewing the included results only:

```bash
pip install -r requirements_visual.txt
```

For re-running SAM refinement, install PyTorch first. Choose one command that matches your machine:

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

Then install the remaining demo dependencies, including Segment Anything:

```bash
pip install -r requirements_demo.txt
```

The official SAM repository states that SAM requires PyTorch and TorchVision, and recommends CUDA support for inference.

## 3. Visualize Included Results

From this `sam_refine/` folder:

```bash
python visualize_comparison.py
```

Or from the repository root:

```bash
python sam_refine/visualize_comparison.py
```

The script writes:

```text
sam_refine/outputs/comparison_grid.png
```

The figure columns are:

1. Original RGB image.
2. DeepLabV3+ 0-shot mask overlay.
3. SAM-refined mask overlay.
4. Pixels added by SAM refinement.

Expected console output:

```text
sample,deeplab_pixels,refined_pixels,added_pixels,removed_pixels
1710490542.876314,69953,181721,111768,0
1710833369.347823,21023,41322,20299,0
1715852650.974332,22535,51666,29131,0
```

## 4. Check SAM Checkpoint

This local demo is expected to use:

```text
checkpoints/sam_vit_h_4b8939.pth
```

If the checkpoint file is already present, you can skip this section and run the refinement command directly.

If it is missing, download the ViT-H checkpoint with the helper script:


```bash
python download_sam_checkpoint.py --model-type vit_h
```

This saves:

```text
sam_refine/checkpoints/sam_vit_h_4b8939.pth
```

From the repository root, run:

```bash
python sam_refine/download_sam_checkpoint.py --model-type vit_h
```

The checkpoint is large and is not committed to Git. The helper uses Meta's official SAM checkpoint URL for `vit_h`.

## 5. Re-run SAM Refinement

From this `sam_refine/` folder:

```bash
python run_sam_refine.py --device cuda:0
```

From the repository root:

```bash
python sam_refine/run_sam_refine.py --device cuda:0
```

Outputs are written to:

```text
sam_refine/data/refine35_rerun/
sam_refine/data/refinevis35_rerun/
```

If you do not have a CUDA GPU, use `--device cpu`.

## 6. Visualize Re-run Results

```bash
python visualize_comparison.py \
  --refine-folder refine35_rerun \
  --output outputs/comparison_grid_rerun.png
```

Small differences from `data/refine35/` may occur across SAM, PyTorch, and CUDA versions.

## 7. Troubleshooting

If `torch` or `torchvision` fails to import, reinstall PyTorch for your CUDA version and keep `numpy<2`.

If `segment_anything` is missing:

```bash
pip install segment-anything
```

If no matching samples are found, check that the image and mask filenames share the same stem:

```text
data/images/1710490542.876314.jpg
data/mask35/1710490542.876314.png
data/refine35/1710490542.876314.png
```

## References

- Official Segment Anything repository: <https://github.com/facebookresearch/segment-anything>
- ViT-H SAM checkpoint: <https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth>
