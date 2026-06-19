# Reproducibility Notes

## Tested Demo Stages

The following stages were checked with the companion demo data:

- Segmentation comparison on three representative crack images.
- Point-cloud colorization smoke test on three frames.
- Non-interactive crack-width measurement on two frames.

The full reconstruction and full 39-frame colorization run are intended for a larger Linux/ROS server environment.

## Recommended Environments

Segmentation:

```bash
conda create -n crack-seg python=3.10 -y
conda activate crack-seg
pip install -r 01_segmentation/requirements-segmentation.txt
pip install -r 01_segmentation/demodata/requirements_visual.txt
```

SAM refinement requires PyTorch and Segment Anything:

```bash
pip install -r 01_segmentation/demodata/requirements_demo.txt
```

Reconstruction:

- Ubuntu 20.04
- ROS Noetic
- Catkin workspace
- PCL, OpenCV, Eigen, Ceres 2.1+, GTSAM, Qhull

PointCloudProcessor C++:

```bash
sudo apt update
sudo apt install -y cmake make git libpcl-dev libopencv-dev libeigen3-dev libboost-all-dev libomp-dev libqhull-dev libfmt-dev
```

`PointCloudProcessor` requires Ceres headers available in Ceres 2.1+ (`ceres/manifold.h` and `ceres/autodiff_first_order_function.h`). Ubuntu 20.04's default `libceres-dev` is Ceres 1.14, so build Ceres 2.1+ from source or use a newer package source before compiling the C++ processor.

Width measurement Python:

```bash
python3 -m venv .venv-width
source .venv-width/bin/activate
pip install -r 05_crack_width_measurement/requirements-width.txt
```

On Python 3.8, these versions were validated locally:

- `open3d==0.18.0`
- `opencv-python==4.8.1.78`
- `plantcv==3.14.3`
- `numpy==1.22.4`
- `scipy==1.10.1`

## Demo Commands

From the repository root:

```bash
bash scripts/run_segmentation_demo.sh ../CCIE2026_Crack3D_demo_data
bash scripts/run_segmentation_full_demo.sh ../CCIE2026_Crack3D_demo_data .venv-seg/bin/python cuda:0
bash scripts/run_width_measurement_demo.sh ../CCIE2026_Crack3D_demo_data .venv-width/bin/python
```

After building the C++ binary:

```bash
bash scripts/run_colorization_smoke.sh ../CCIE2026_Crack3D_demo_data 04_pointcloud_colorization/build/PointCloudProcessor
```

## Known Constraints

- The complete 39-frame colorization accumulates multi-view RGB scores in memory and can exceed workstation memory.
- `--manual-select` requires an OpenCV GUI session.
- Camera intrinsics are dataset-specific. Replace `configs/camera_4096x3000_paper.json` for another camera.
- Raw bags and large point clouds should be hosted outside Git and referenced by URL/checksum.
