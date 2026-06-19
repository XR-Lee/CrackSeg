# FAST-LIO Color Mapping

This package is the LiDAR-inertial reconstruction and RGB point-cloud colorization component used in the CCIE 2026 crack reconstruction pipeline.

It expects a ROS bag containing synchronized LiDAR, IMU, and RGB image topics. The paper-specific configuration is provided in:

```text
config/zhongnan.yaml
launch/mapping_zhongnan.launch
```

## Build

Use Ubuntu 20.04 and ROS Noetic.

Create a Catkin workspace and link this package together with the included Livox driver package:

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

Required ROS/system dependencies include `roscpp`, `rospy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `tf`, `pcl_ros`, `cv_bridge`, `eigen_conversions`, Eigen, PCL, OpenCV, Boost filesystem/system, OpenMP, and Python development headers.

## Configure Topics and Calibration

Edit `config/zhongnan.yaml` if your ROS bag uses different topics or calibration parameters.

The default paper configuration uses:

```yaml
common:
  lid_topic: "/Pandar/XT32_M2X/pandar"
  imu_topic: "/wit/imu"
  camera_topic: "/hikrobot_camera/rgb"
```

The LiDAR-camera intrinsics/extrinsics are also stored in `config/zhongnan.yaml` under `color_mapping`.

## Run

From the Catkin workspace root:

```bash
RELEASE_ROOT=/mnt/e/crackmeasurement/CCIE2026_Crack3D_pipeline_release
RAW_BAG=/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag

bash "$RELEASE_ROOT/03_reconstruction_fusion/fast_lio_color_mapping/run_slam_and_colorization.sh" \
  "$RAW_BAG" \
  devel/setup.bash \
  mapping_zhongnan.launch
```

The script creates an output folder next to the bag:

```text
CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct/
|-- raw_images/
|-- operation_logs/
|-- visual_odom.txt
|-- visual_odom_in_lidar_ts.txt
|-- scans.pcd
`-- color_scans.pcd
```

`visual_odom*.txt` and the generated images/point clouds are used by the later OpenMVS and crack-width measurement steps.

## Notes

- This release contains source code only. ROS `build/`, `devel/`, bags, point clouds, extracted images, and vendor SDK binaries are intentionally excluded.
- The included `livox_ros_driver` package is required at build time because FAST-LIO preprocessing supports Livox custom messages, even when the active launch file uses a standard `sensor_msgs/PointCloud2` LiDAR topic.
- Sensor vendor drivers such as Hikrobot and Hesai are not vendored. For offline bag replay, they are not required if the bag already contains standard ROS messages.

## Acknowledgments

This package is based on FAST-LIO2 and FAST-LIO-COLOR-MAPPING, with project-specific changes for crack reconstruction and visual-LiDAR fusion.
