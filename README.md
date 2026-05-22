# CrackSeg

**3D Modeling and Automated Measurement of Concrete Cracks via Segment Anything Refinement and Visual Inertial LiDAR Fusion**

Reference implementation for [Deng et al., 2025 (arXiv:2501.09203)](https://arxiv.org/abs/2501.09203). The pipeline turns hand-held RGB + LiDAR + IMU captures of a concrete surface into a semantically segmented, real-world-scale 3D crack model with sub-millimetre width measurements — no manual post-processing.

## Method overview

The full pipeline has three stages:

1. **Crack-aware SAM refinement.** A DeepLabV3+ baseline produces a coarse 2D crack mask; we generate point prompts from the Euclidean distance-transform (EDT) skeleton, run SAM on local crops, and reject over-wide masks via width comparison. Gives ~6% IoU improvement over the zero-shot DeepLabV3+ baseline.
2. **Visual–Inertial–LiDAR (VIL) SLAM fusion.** FastLIO2 fuses LiDAR scans and IMU into a dense, drift-corrected point cloud; multi-frame RGB masks are projected into 3D to form a semantically-enriched crack point cloud at real-world scale.
3. **Automated 3D measurement.** Crack width and spatial location are extracted directly from the 3D crack point cloud. Validated against a Dino-Lite AF4915 microscope with mean width error < 0.1 mm.

## Repository status

| Stage | Folder | Status |
| --- | --- | --- |
| Crack-aware SAM refinement | [`sam_refine/`](sam_refine/) | Demo released (3 field samples, reproducible); full pipeline & training planned |
| VIL-SLAM fusion (FastLIO2 + multi-frame projection) | [`vil_slam/`](vil_slam/) | Planned (placeholder) |
| Automated 3D crack measurement | [`measurement/`](measurement/) | Planned (placeholder) |
| Training & evaluation scripts (0/10/110-shot) | [`training/`](training/) | Planned (placeholder) |

Only stage 1 has runnable code today, packaged as a self-contained demo under `sam_refine/`. The remaining stages have placeholder folders with brief READMEs explaining what will land there.

## Quickstart

Run the SAM refinement demo on the three bundled 4096×3000 field samples. See [`sam_refine/README.md`](sam_refine/README.md) for the full tutorial; the short version:

```bash
conda create -n crackdemo python=3.10 -y
conda activate crackdemo

# Install PyTorch first, matching your CUDA version (or CPU). Then:
pip install -r sam_refine/requirements_demo.txt

python sam_refine/download_sam_checkpoint.py --model-type vit_h
python sam_refine/run_sam_refine.py --device cuda:0
python sam_refine/visualize_comparison.py
```

Outputs (refined masks, visualisations, comparison grid) are written under `sam_refine/data/` and `sam_refine/outputs/`.

## Datasets

The paper aggregates ten public crack-segmentation datasets plus one in-house field dataset (9,260 annotated images in total).

| Dataset | Notes |
| --- | --- |
| Ceramic-Cracks | Ceramic-tile cracks |
| CFD | Crack Forest Dataset (pavement) |
| Crack500 | Pavement-crack benchmark |
| CrackTree200 | Pavement-crack benchmark |
| DeepCrack | Multi-scene crack benchmark |
| GAPS | German Asphalt Pavement distress |
| Masonry | Masonry-surface cracks |
| Rissbilder | Concrete-crack images |
| Volker | Concrete-crack images |
| CCSS-DATA | Crack semantic-segmentation dataset |
| **Field (in-house)** | 150 images @ 4096×3000 of real concrete structures, plus paired LiDAR+IMU sequences for the VIL-SLAM experiments — to be released via [GitHub Releases](https://github.com/XR-Lee/CrackSeg/releases) |

Public datasets should be obtained from their original sources; the paper cites each. The in-house field dataset and LiDAR/IMU sequences will be published as a GitHub Release once cleared for distribution; the URL above will resolve once the release is live.

## Model checkpoints

- **SAM weights.** Meta's official Segment Anything checkpoints; `sam_refine/download_sam_checkpoint.py` pulls `sam_vit_h_4b8939.pth` directly from `dl.fbaipublicfiles.com`.
- **Fine-tuned DeepLabV3+** (0/10/110-shot configurations from the paper) will be published with the stage-2 release via [GitHub Releases](https://github.com/XR-Lee/CrackSeg/releases).

## Citation

If you use this code or build on the method, please cite:

```bibtex
@article{deng2025crackseg,
  title   = {3D Modeling and Automated Measurement of Concrete Cracks via
             Segment Anything Refinement and Visual Inertial LiDAR Fusion},
  author  = {Deng, Pengru and Yao, Jiapeng and Li, Chun and Wang, Su and
             Li, Xinrun and Ojha, Varun and He, Xuhui},
  journal = {arXiv preprint arXiv:2501.09203},
  year    = {2025}
}
```

## License

Licensed under the Apache License, Version 2.0 — see [LICENSE](LICENSE).

The SAM weights downloaded by `sam_refine/download_sam_checkpoint.py` are released by Meta under the Apache 2.0 License; see the [Segment Anything repository](https://github.com/facebookresearch/segment-anything) for terms. Each public dataset listed above is governed by its own license; consult the original source before redistribution.
