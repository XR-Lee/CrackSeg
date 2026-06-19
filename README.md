# CrackSeg

**3D Modeling and Automated Measurement of Concrete Cracks via Segment Anything Refinement and Visual Inertial LiDAR Fusion**

Reference implementation for Deng et al., *Computer-Aided Civil and Infrastructure Engineering* (2026) — [arXiv:2501.09203](https://arxiv.org/abs/2501.09203). The pipeline turns hand-held RGB + LiDAR + IMU captures of a concrete surface into a semantically segmented, real-world-scale 3D crack model with sub-millimetre width measurements — no manual post-processing.

## Method overview

Conceptually the pipeline has three stages:

1. **Crack-aware SAM refinement.** A DeepLabV3+ baseline produces a coarse 2D crack mask; we generate point prompts from the Euclidean distance-transform (EDT) skeleton, run SAM on local crops, and reject over-wide masks via width comparison. Gives ~6% IoU improvement over the zero-shot DeepLabV3+ baseline.
2. **Visual–Inertial–LiDAR (VIL) SLAM fusion.** FastLIO2 fuses LiDAR scans and IMU into a dense, drift-corrected point cloud; multi-frame RGB masks are projected into 3D to form a semantically-enriched crack point cloud at real-world scale.
3. **Automated 3D measurement.** Crack width and spatial location are extracted directly from the 3D crack point cloud. Validated against a Dino-Lite AF4915 microscope with mean width error < 0.1 mm.

## Repository layout

The code is organized into five numbered stages in pipeline order:

| # | Folder | What it does | Runs where |
| --- | --- | --- | --- |
| 1 | [`01_segmentation/`](01_segmentation/) | DeepLabV3+ coarse masks + crack-aware SAM refinement | Python (CPU/GPU) |
| 2 | [`02_rosbag_extraction/`](02_rosbag_extraction/) | ROS bag → images / LiDAR PCD / IMU·location·mag CSV | Python (ROS optional) |
| 3 | [`03_reconstruction_fusion/`](03_reconstruction_fusion/) | Visual-inertial-LiDAR reconstruction (FAST-LIO / R3LIVE / OpenMVS) | Linux + ROS Noetic |
| 4 | [`04_pointcloud_colorization/`](04_pointcloud_colorization/) | Point-cloud crop / denoise / colorize + semantic mask projection | C++ (Linux + ROS + Ceres 2.1) |
| 5 | [`05_crack_width_measurement/`](05_crack_width_measurement/) | Automated 3D crack-width measurement | Python (CPU/GPU) |

Stages 1 and 5 run on a plain workstation (verified on macOS/CPU). Stages 2–4 target a Linux/ROS Noetic server; see [`docs/REPRODUCIBILITY.md`](docs/REPRODUCIBILITY.md) for the full environment matrix and [`docs/PIPELINE_IO.md`](docs/PIPELINE_IO.md) for stage-by-stage input/output contracts.

## Demo data

Large inputs (model checkpoints, sample images, point clouds) are hosted off-repo, not in Git:

> **Demo data (Google Drive): https://drive.google.com/drive/folders/1gFtSRWuKS7C0BrCifCmJpb7bshKcNFeG**

Download the whole folder and place it **next to** this repository so the demo scripts resolve it via `../CCIE2026_Crack3D_demo_data`:

```text
parent/
|-- CrackSeg/                       # this repository
`-- CCIE2026_Crack3D_demo_data/     # downloaded demo data (this Drive folder)
    |-- README.md
    |-- checksums.sha256
    |-- models/                     # checkpoint35.pth.tar (DeepLabV3+); SAM weight via download script
    |-- segmentation_demo/          # 3 field images + mask35/refine35 + comparison grid   (Stage 1)
    |-- colorization_smoke/         # scans-crop.pcd + odometry + 3 raw_images             (Stage 4)
    |-- width_measurement_demo/     # filtered_pcd / mask_select / raw_images + expected/   (Stage 5)
    `-- full_dataset_note/          # note on obtaining the full 34 GB raw ROS bag
```

After downloading, optionally verify integrity (the checklist uses CRLF line endings, so strip them first):

```bash
cd CCIE2026_Crack3D_demo_data
tr -d '\r' < checksums.sha256 | shasum -a 256 -c -    # Linux: ... | sha256sum -c -
```

See [`docs/DATASETS.md`](docs/DATASETS.md) for the full folder breakdown and per-file checksums.

## Quickstart

### Stage 1 — Segmentation (SAM refinement)

```bash
conda create -n crackdemo python=3.10 -y
conda activate crackdemo
pip install -r 01_segmentation/requirements-segmentation.txt
pip install -r 01_segmentation/demodata/requirements_demo.txt

# Visualization-only demo on the three bundled field samples:
bash scripts/run_segmentation_demo.sh ../CCIE2026_Crack3D_demo_data

# Full re-run: DeepLabV3+ coarse masks -> SAM-refined masks:
python 01_segmentation/demodata/download_sam_checkpoint.py --model-type vit_h
bash scripts/run_segmentation_full_demo.sh ../CCIE2026_Crack3D_demo_data python3 cuda:0
```

### Stage 2 — ROS bag extraction

Extract synchronized camera / LiDAR / IMU streams from a raw ROS bag. Runs even without a full ROS install (falls back to the pure-Python `rosbags` backend).

```bash
cd 02_rosbag_extraction
conda env create -f environment.yml
conda activate rosbag-extractor
pip install -e .            # installs the bag_info / *_extractor commands
cd -

RAW_BAG=../CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag
OUT=../CCIE2026_Crack3D_demo_data/full_dataset/extracted

bag_info               "$RAW_BAG"                                                    # list topics
image_extractor        "$RAW_BAG" --output_folder "$OUT/images" --topic /hikrobot_camera/rgb/compressed
point_cloud_extractor  "$RAW_BAG" "$OUT/pcds"                   --topic /Pandar/XT32_M2X/pandar
imu_csv_extractor      "$RAW_BAG" --output_csv "$OUT/csv/imu.csv"      --topic /wit/imu
location_csv_extractor "$RAW_BAG" --output_csv "$OUT/csv/location.csv" --topic /wit/location
```

Full extractor list and output layout: [`02_rosbag_extraction/README.md`](02_rosbag_extraction/README.md).

### Stage 3 — Visual-inertial-LiDAR reconstruction

Requires Ubuntu 20.04 + ROS Noetic + a catkin workspace (PCL, OpenCV, Eigen, Ceres 2.1+, GTSAM). Build the stage-3 SLAM packages into your workspace, then run FAST-LIO color mapping:

```bash
cd ~/ccie_crack3d_ws        # your catkin workspace with the stage-3 packages built
bash <repo>/03_reconstruction_fusion/fast_lio_color_mapping/run_slam_and_colorization.sh \
  ../CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag \
  devel/setup.bash \
  mapping_zhongnan.launch
```

Produces `{bag}_reconstruct/` with the fused `scans.pcd`, camera odometry (`visual_odom*.txt`), and `raw_images/`. Workspace setup: [`03_reconstruction_fusion/README.md`](03_reconstruction_fusion/README.md) and [`03_reconstruction_fusion/RECONSTRUCTION_IO_WORKFLOW.md`](03_reconstruction_fusion/RECONSTRUCTION_IO_WORKFLOW.md).

### Stage 4 — Point-cloud colorization & semantic projection

C++ tool; needs ROS catkin, Iridescence, and Ceres 2.1+ (see [`docs/REPRODUCIBILITY.md`](docs/REPRODUCIBILITY.md)). Build once, then run the 3-frame smoke demo:

```bash
cd 04_pointcloud_colorization
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j"$(nproc)"
cd -

bash scripts/run_colorization_smoke.sh ../CCIE2026_Crack3D_demo_data \
  04_pointcloud_colorization/build/PointCloudProcessor
```

Outputs `cloudInWorldWithRGB.pcd`, `cloudInWorldWithRGBandMask.pcd`, and per-frame `filtered_pcd/` consumed by Stage 5. Full options: [`04_pointcloud_colorization/README.md`](04_pointcloud_colorization/README.md).

### Stage 5 — 3D crack-width measurement

```bash
pip install -r 05_crack_width_measurement/requirements-width.txt
bash scripts/run_width_measurement_demo.sh ../CCIE2026_Crack3D_demo_data
```

Outputs include `crack_width_3d_results.json` plus per-frame skeleton/edge visualizations.

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
| **Field (in-house)** | 150 images @ 4096×3000 of real concrete structures, plus paired LiDAR+IMU sequences for the VIL-SLAM experiments |

Public datasets should be obtained from their original sources; the paper cites each. The in-house field dataset, LiDAR/IMU sequences, and the runnable demo bundle are distributed via the demo-data link above.

## Model checkpoints

- **SAM weights.** Meta's official Segment Anything checkpoints; `01_segmentation/demodata/download_sam_checkpoint.py` pulls `sam_vit_h_4b8939.pth` directly from `dl.fbaipublicfiles.com`.
- **Fine-tuned DeepLabV3+** (`checkpoint35.pth.tar`, 0/10/110-shot configurations from the paper) is distributed with the demo data above.

## Citation

If you use this code or build on the method, please cite:

```bibtex
@article{deng20263d,
  title={3D modeling and automated measurement of concrete cracks via segment anything refinement and visual inertial LiDAR fusion},
  author={Deng, Pengru and Yao, Jiapeng and Li, Chun and Wang, Su and Li, Xinrun and Ojha, Varun and He, Xuhui},
  journal={Computer-Aided Civil and Infrastructure Engineering},
  pages={100019},
  year={2026},
  publisher={Elsevier}
}
```

## License

Licensed under the Apache License, Version 2.0 — see [LICENSE](LICENSE).

The SAM weights downloaded by `01_segmentation/demodata/download_sam_checkpoint.py` are released by Meta under the Apache 2.0 License; see the [Segment Anything repository](https://github.com/facebookresearch/segment-anything) for terms. Vendored third-party SLAM components under `03_reconstruction_fusion/` (FAST-LIO, R3LIVE, OpenMVS, Livox driver, etc.) retain their own upstream licenses; see the license file inside each subfolder. Each public dataset listed above is governed by its own license; consult the original source before redistribution.
