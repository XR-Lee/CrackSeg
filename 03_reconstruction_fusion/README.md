# Reconstruction and Visual-LiDAR Fusion

This folder contains the source-only release of the reconstruction and semantic/visual fusion code used after ROS bag extraction and crack-image segmentation.

## Included Modules

```text
reconstruction_and_fusion/
|-- direct_visual_lidar_calibration/   # LiDAR-camera extrinsic calibration toolbox
|-- fast_lio_slam/                     # FAST-LIO + scan-context pose-graph mapping source
|-- fast_lio_color_mapping/            # FAST-LIO based RGB/color point-cloud fusion
|-- livox_ros_driver/                  # Livox ROS driver source used by FAST-LIO variants
|-- openmvs_slam/                      # OpenMVS fork and scripts for mesh reconstruction
`-- r3live/                            # R3LIVE RGB LiDAR-inertial-visual mapping source
```

Build outputs, ROS `devel/` folders, raw bags, point clouds, images, PDFs, notebooks, binary libraries, and local IDE files are intentionally excluded.

## Pipeline Position

1. Extract ROS bag data with `02_rosbag_extraction`.
2. Calibrate LiDAR-camera extrinsics with `direct_visual_lidar_calibration` or your existing calibration files.
3. Run LiDAR-inertial reconstruction with `fast_lio_slam`, `fast_lio_color_mapping`, or `r3live`.
4. Generate keyframe image and OpenMVS frame metadata with `openmvs_slam/scripts/generate_mvs_workspace.py`.
5. Run OpenMVS densification, mesh reconstruction, refinement, and texturing with `openmvs_slam/scripts/run_mvs.sh`.
6. Project refined crack masks and measure crack width with `PointCloudProcessor`.

## ROS Build Layout

Create a Catkin workspace and copy or symlink the required packages:

```bash
RELEASE_ROOT=/mnt/e/crackmeasurement/CCIE2026_Crack3D_pipeline_release

mkdir -p ~/ccie_crack3d_ws/src
cd ~/ccie_crack3d_ws/src

ln -s "$RELEASE_ROOT/03_reconstruction_fusion/fast_lio_color_mapping" .
ln -s "$RELEASE_ROOT/03_reconstruction_fusion/livox_ros_driver/livox_ros_driver" .

cd ~/ccie_crack3d_ws
catkin_make
source devel/setup.bash
```

For R3LIVE:

```bash
RELEASE_ROOT=/mnt/e/crackmeasurement/CCIE2026_Crack3D_pipeline_release
ln -s "$RELEASE_ROOT/03_reconstruction_fusion/r3live/r3live" ~/ccie_crack3d_ws/src/r3live
catkin_make
```

Use ROS Noetic on Ubuntu 20.04 unless you have already validated another ROS distribution.

## FAST-LIO Color Mapping

The paper-specific configuration is:

```text
fast_lio_color_mapping/config/zhongnan.yaml
fast_lio_color_mapping/launch/mapping_zhongnan.launch
```

Run:

```bash
cd ~/ccie_crack3d_ws

RELEASE_ROOT=/mnt/e/crackmeasurement/CCIE2026_Crack3D_pipeline_release
RAW_BAG=/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag

bash "$RELEASE_ROOT/03_reconstruction_fusion/fast_lio_color_mapping/run_slam_and_colorization.sh" \
  "$RAW_BAG" \
  devel/setup.bash \
  mapping_zhongnan.launch
```

Expected reconstruction outputs include colorized point clouds and visual/LiDAR odometry text files under the generated reconstruction directory.

For a developer-facing description of the reconstruction backend input contract, output files, processing flow, and software integration notes, see `RECONSTRUCTION_IO_WORKFLOW.md`.

## OpenMVS Metadata Generation

First interpolate FAST-LIO scan-state odometry to image timestamps:

```bash
WORK_DIR=/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct

python openmvs_slam/scripts/make_vo_odom_for_fastlio.py \
  --scan-states "$WORK_DIR/visual_odom_in_lidar_ts.txt" \
  --visual-odom "$WORK_DIR/visual_odom.txt" \
  --output "$WORK_DIR/vo_interpolated_odom.txt" \
  --tum-output "$WORK_DIR/visual_odom_in_lidar_ts_TUM.txt"
```

Then generate selected keyframe images and `mvs_frame_result.txt`:

```bash
python openmvs_slam/scripts/generate_mvs_workspace.py \
  --work-dir "$WORK_DIR" \
  --config "$WORK_DIR/index.json" \
  --raw-images "$WORK_DIR/raw_images" \
  --point-cloud "$WORK_DIR/scans-clean-mls-clean.pcd" \
  --distance-threshold 0.5 \
  --camera-model pinhole
```

The calibration JSON is expected to contain:

```json
{
  "camera_para": {
    "w": 4096,
    "h": 3000,
    "fx": 4818.2,
    "fy": 4819.1,
    "cx": 2032.4,
    "cy": 1535.2,
    "k1": 0.003,
    "k2": 0.066,
    "k3": -0.0002,
    "k4": -0.0006
  },
  "Tlc": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
}
```

Finally run OpenMVS:

```bash
MVS_WORKSPACE=/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/MVS_Workspace
OPENMVS_BIN=/opt/openMVS/build/bin

bash openmvs_slam/scripts/run_mvs.sh \
  "$MVS_WORKSPACE" \
  "$OPENMVS_BIN" \
  "$WORK_DIR/mvs_frame_result.txt"
```

## Dependencies

Core system dependencies:

- ROS Noetic
- Eigen
- PCL
- OpenCV
- Ceres
- GTSAM
- Python 3 with `numpy`, `scipy`, `opencv-python`, `open3d`, and `tqdm`

Module-specific dependencies:

- `direct_visual_lidar_calibration` requires its upstream dependencies, including Iridescence and optional SuperGlue.
- `openmvs_slam` requires OpenMVS dependencies such as CGAL, Boost, VCGLib, OpenCV, Eigen, and Ceres.
- Sensor SDKs such as Hikrobot MVS and Hesai SDK are not vendored in this release because they contain binary/vendor files and should be installed from the vendor packages.

## Source Audit Notes

- `fastlio-sam-ws`, `fastlio-sam-qn-ws`, and `SmartScanner` from the uploaded source folder were not copied because their files are zero-length in the provided upload and are not usable as source.
- `ros-dependences` was not copied because it contains third-party dependency source/build artifacts; install dependencies through the system package manager or upstream projects.
- `kalibr` was not copied because it is an external calibration project and not part of the paper-specific implementation. Use upstream Kalibr if camera-IMU calibration must be reproduced.
- A duplicate point-cloud processor from the original source folder was not copied because the cleaned `04_pointcloud_colorization/` release already exists at the repository root.

## Output Policy

Do not commit generated data to Git:

- `.bag`, `.pcd`, `.ply`, `.mvs`
- extracted images and videos
- OpenMVS workspaces
- ROS `build/`, `devel/`, `install/`, and `logs/`
- vendor SDK binary libraries
