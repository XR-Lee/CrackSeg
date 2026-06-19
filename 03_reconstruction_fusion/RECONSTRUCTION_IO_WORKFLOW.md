# 重建程序输入、输出与流程说明

本文档用于向协作者说明论文中三维重建程序的运行入口、输入数据、输出结果、内部流程以及后续软件开发时需要注意的数据接口。目标读者是准备协助封装、维护或继续开发重建功能的软件开发人员。

当前重建与彩色点云融合的主要运行入口是：

```text
fast_lio_color_mapping/run_slam_and_colorization.sh
```

这个脚本本身不是完整算法主体，而是一个启动器。它负责检查输入 ROS bag、检查 ROS/Catkin 环境、生成输出目录，然后调用 `roslaunch` 启动 `fast_lio_color_mapping` 中的重建节点。

## 1. 程序入口

在已经编译好的 Catkin 工作空间根目录下运行：

```bash
bash /mnt/e/crackmeasurement/CCIE2026_Crack3D_pipeline_release/03_reconstruction_fusion/fast_lio_color_mapping/run_slam_and_colorization.sh \
  /mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag \
  devel/setup.bash \
  mapping_zhongnan.launch
```

脚本参数格式：

```bash
bash run_slam_and_colorization.sh /mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag [devel/setup.bash] [launch_file]
```

参数说明：

| 参数 | 是否必需 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag` | 是 | 无 | 输入 ROS bag，包含 LiDAR、IMU 和 RGB 图像 topic。 |
| `devel/setup.bash` | 否 | `devel/setup.bash` | Catkin 工作空间编译后生成的环境文件。 |
| `launch_file` | 否 | `mapping_zhongnan.launch` | `fast_lio_color_mapping/launch/` 下的 ROS launch 文件。 |

脚本启动前会检查：

- 输入 ROS bag 是否存在；
- `/opt/ros/noetic/setup.bash` 是否存在；
- Catkin 工作空间的 `devel/setup.bash` 是否存在。

如果其中任意一项不存在，程序会直接退出，不会启动重建。

## 2. 运行环境

推荐环境：

| 组件 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS Noetic |
| 编译系统 | Catkin |
| 基础库 | Eigen, PCL, OpenCV, Boost, Python development headers, OpenMP |
| ROS 依赖 | `roscpp`, `rospy`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `tf`, `pcl_ros`, `cv_bridge`, `eigen_conversions`, `rosbag` |

Catkin 工作空间至少需要包含：

```text
ccie_crack3d_ws/
|-- src/
|   |-- fast_lio_color_mapping/
|   `-- livox_ros_driver/
|-- build/
`-- devel/
```

示例配置：

```bash
mkdir -p ~/ccie_crack3d_ws/src
cd ~/ccie_crack3d_ws/src

ln -s /mnt/e/crackmeasurement/CCIE2026_Crack3D_pipeline_release/03_reconstruction_fusion/fast_lio_color_mapping .
ln -s /mnt/e/crackmeasurement/CCIE2026_Crack3D_pipeline_release/03_reconstruction_fusion/livox_ros_driver/livox_ros_driver .

cd ~/ccie_crack3d_ws
catkin_make
source devel/setup.bash
```

说明：

- `fast_lio_color_mapping` 是重建和 RGB 点云融合的核心 ROS package。
- `livox_ros_driver` 主要提供 FAST-LIO 代码依赖的 Livox 自定义消息定义。
- 本论文默认 `zhongnan.yaml` 配置使用的是标准 `sensor_msgs/PointCloud2` LiDAR topic，因此离线 replay 论文 bag 时不需要连接真实 Livox 设备。

## 3. 输入数据

### 3.1 ROS Bag

主输入是一个 ROS bag 文件：

```text
/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag
```

默认配置要求 bag 中包含以下 topic：

| 数据 | `config/zhongnan.yaml` 中的默认 topic | 期望消息类型 |
| --- | --- | --- |
| LiDAR 点云 | `/Pandar/XT32_M2X/pandar` | `sensor_msgs/PointCloud2` |
| IMU | `/wit/imu` | `sensor_msgs/Imu` |
| RGB 图像 | `/hikrobot_camera/rgb` | `sensor_msgs/Image` |

如果使用其他数据集，需要修改：

```text
fast_lio_color_mapping/config/zhongnan.yaml
```

重建结果对以下因素非常敏感：

- LiDAR 和 IMU 时间同步；
- LiDAR 和相机时间同步；
- LiDAR-IMU 外参；
- LiDAR-相机外参；
- 相机内参和畸变参数；
- bag 中 topic 名称是否和 YAML 配置一致。

### 3.2 配置文件

论文默认配置文件是：

```text
fast_lio_color_mapping/config/zhongnan.yaml
```

关键字段：

| YAML 字段 | 含义 |
| --- | --- |
| `common/lid_topic` | ROS bag 中的 LiDAR topic。 |
| `common/imu_topic` | ROS bag 中的 IMU topic。 |
| `common/camera_topic` | ROS bag 中的 RGB 图像 topic。 |
| `preprocess/lidar_type` | LiDAR 预处理类型。论文配置使用标准点云输入。 |
| `mapping/extrinsic_T`, `mapping/extrinsic_R` | LiDAR-IMU 外参。 |
| `color_mapping/K_camera` | 相机内参矩阵。 |
| `color_mapping/D_camera` | 相机畸变参数。 |
| `color_mapping/extrinsic_T`, `color_mapping/extrinsic_R` | LiDAR-相机外参。 |
| `color_mapping/max_time_diff` | LiDAR 帧和图像帧匹配时允许的最大时间差。 |
| `pcd_save/pcd_save_en` | 是否保存重建点云。 |

### 3.3 Launch 文件

默认启动文件是：

```text
fast_lio_color_mapping/launch/mapping_zhongnan.launch
```

这个 launch 文件会完成三件事：

1. 加载 `config/zhongnan.yaml`；
2. 启动 `fastlio_mapping` 重建节点；
3. 用 `rosbag play` 播放输入 bag。

实际等价于：

```bash
roslaunch fast_lio_color_mapping mapping_zhongnan.launch \
  rosbag_path:=/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag \
  reconstruction_path:=/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct/
```

## 4. 输出结果

对于输入：

```text
/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag
```

脚本会自动生成输出目录：

```text
/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct/
```

输出目录命名规则：

```text
<rosbag 所在目录>/<rosbag 文件名去掉 .bag>_reconstruct/
```

典型输出结构：

```text
data_reconstruct/
|-- raw_images/
|   `-- <image_timestamp>.jpg
|-- operation_logs/
|   |-- pos_log.txt
|   |-- mat_pre.txt
|   |-- mat_out.txt
|   `-- dbg.txt
|-- visual_odom.txt
|-- visual_odom_in_lidar_ts.txt
|-- scans.pcd
`-- color_scans.pcd
```

### 4.1 `raw_images/`

从 ROS 图像 topic 中抽取保存的 RGB 图像。

文件名格式：

```text
<matched_image_timestamp>.jpg
```

示例：

```text
1710833369.347823.jpg
```

当前代码默认每 20 个匹配到的相机帧保存 1 张图像。这个频率目前写在 `laserMapping.cpp` 中，不是 shell 脚本参数。

### 4.2 `visual_odom.txt`

图像时间戳对应的相机位姿。

每一行格式：

```text
timestamp tx ty tz qw qx qy qz
```

字段含义：

| 字段 | 含义 |
| --- | --- |
| `timestamp` | 匹配到的 RGB 图像时间戳。 |
| `tx ty tz` | 相机在重建/world 坐标系下的位置。 |
| `qw qx qy qz` | 相机姿态四元数。 |

这个文件后续可用于：

- 图像帧和三维重建结果关联；
- OpenMVS 工作空间生成；
- 裂缝语义 mask 从图像空间投影到三维空间。

### 4.3 `visual_odom_in_lidar_ts.txt`

LiDAR 时间戳对应的相机位姿。

每一行格式同样是：

```text
timestamp tx ty tz qw qx qy qz
```

区别是这个文件的 `timestamp` 使用 LiDAR 帧时间序列，而不是图像帧时间序列。它适合用于需要和 LiDAR 扫描帧对齐的后处理程序。

### 4.4 `scans.pcd`

FAST-LIO 重建得到的几何点云。

保存条件由 YAML 控制：

```yaml
pcd_save:
  pcd_save_en: true
```

如果 `pcd_save_en` 关闭，`scans.pcd` 可能不会生成。

### 4.5 `color_scans.pcd`

RGB 彩色点云。

程序会根据以下信息把 LiDAR 点投影到图像中：

- LiDAR-相机外参；
- 相机内参；
- 相机畸变参数；
- FAST-LIO 估计得到的 LiDAR/IMU 轨迹；
- 与当前 LiDAR 帧匹配的 RGB 图像。

如果某个三维点投影后落在图像范围内，就从对应像素读取 RGB 值并赋给该三维点，最终累计保存为 `color_scans.pcd`。

### 4.6 `operation_logs/`

运行过程中的调试日志和状态日志。

这些文件主要用于排查问题，不建议作为稳定的软件接口依赖。

## 5. 重建流程

整体数据流如下：

```text
ROS bag
  |
  |-- LiDAR PointCloud2
  |-- IMU
  `-- RGB Image
        |
        v
run_slam_and_colorization.sh
        |
        v
roslaunch mapping_zhongnan.launch
        |
        v
fastlio_mapping node
        |
        |-- 同步 LiDAR、IMU、RGB 图像
        |-- 用 IMU 辅助 LiDAR 点云去畸变
        |-- FAST-LIO 估计 LiDAR-inertial 轨迹
        |-- 增量构建三维点云地图
        |-- 根据 LiDAR-相机外参计算相机位姿
        |-- 将 LiDAR 点投影到 RGB 图像
        |-- 给三维点赋 RGB 颜色
        |
        v
data_reconstruct/
```

详细步骤：

1. **播放 ROS bag**

   `mapping_zhongnan.launch` 内部调用 `rosbag play`，把输入 bag 中的 topic 发布出来。

2. **订阅传感器数据**

   `fastlio_mapping` 订阅 LiDAR、IMU 和 RGB 图像 topic。

3. **时间戳匹配**

   程序把 LiDAR 帧和附近的图像帧进行匹配。`color_mapping/max_time_diff` 控制允许的最大时间差。

4. **LiDAR-inertial SLAM**

   FAST-LIO 使用 LiDAR 和 IMU 数据估计传感器运动轨迹，并逐帧构建三维点云地图。

5. **计算相机位姿**

   根据估计得到的 IMU/LiDAR 位姿，以及 LiDAR-相机外参，得到每个匹配图像时刻的相机位姿。

6. **RGB 点云融合**

   程序把三维 LiDAR 点投影到 RGB 图像平面。投影有效的点会读取图像像素颜色，形成彩色点云。

7. **导出结果**

   程序把图像帧、位姿文本、几何点云、彩色点云和运行日志保存到重建输出目录。

## 6. 软件开发接口建议

如果要把该程序封装成软件，可以先把它当成一个命令行后端。

### 6.1 建议的软件输入

可以把前端或服务端传给后端的参数组织成：

```json
{
  "rosbag_path": "/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag",
  "catkin_setup": "/home/user/ccie_crack3d_ws/devel/setup.bash",
  "launch_file": "mapping_zhongnan.launch",
  "config_file": "/mnt/e/crackmeasurement/CCIE2026_Crack3D_pipeline_release/03_reconstruction_fusion/fast_lio_color_mapping/config/zhongnan.yaml"
}
```

### 6.2 建议的软件输出

后端运行结束后，可以返回：

```json
{
  "reconstruction_dir": "/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct",
  "raw_images_dir": "/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct/raw_images",
  "visual_odom": "/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct/visual_odom.txt",
  "visual_odom_lidar_ts": "/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct/visual_odom_in_lidar_ts.txt",
  "point_cloud": "/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct/scans.pcd",
  "color_point_cloud": "/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct/color_scans.pcd",
  "logs_dir": "/mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54_reconstruct/operation_logs"
}
```

### 6.3 运行前检查

软件封装层建议在启动前检查：

- ROS Noetic 是否安装；
- Catkin 工作空间是否已经编译；
- `fast_lio_color_mapping` 是否能被 ROS 找到；
- ROS bag 文件是否存在；
- ROS bag 是否包含 YAML 中配置的 LiDAR、IMU、RGB 图像 topic；
- 输出目录是否已有旧结果；
- YAML 中是否存在必要的相机内参、相机畸变、LiDAR-IMU 外参和 LiDAR-相机外参。

常用检查命令：

```bash
rospack find fast_lio_color_mapping
rosbag info /mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag
```

### 6.4 重要的覆盖行为

当前 C++ 节点启动后，如果重建输出目录已经存在，会先删除该目录，再重新创建。

因此，如果后续开发 GUI 或服务端程序，需要特别处理：

- 每次运行生成唯一输出目录；
- 或在覆盖前提醒用户；
- 或自动把旧输出目录归档备份。

否则可能误删已有重建结果。

### 6.5 进度与完成判断

当前 shell 脚本没有输出结构化进度。软件封装时可以通过以下方式判断运行状态：

- 监听进程退出码；
- 解析 ROS launch 输出；
- 检查 `raw_images/` 是否开始生成图像；
- 检查 `visual_odom.txt` 是否持续增长；
- 检查最终是否生成 `color_scans.pcd`。

建议完成判据：

```text
进程正常结束
AND visual_odom.txt 存在
AND visual_odom_in_lidar_ts.txt 存在
AND color_scans.pcd 存在
```

如果 YAML 中关闭了 `pcd_save/pcd_save_en`，则 `scans.pcd` 缺失不一定代表失败。

### 6.6 常见问题

| 现象 | 可能原因 | 建议检查 |
| --- | --- | --- |
| `ROS bag not found` | bag 路径错误 | 检查绝对路径。 |
| `ROS Noetic setup file not found` | 当前机器没有 ROS Noetic | 安装 ROS Noetic 或换到正确环境。 |
| `Catkin setup file not found` | 工作空间未编译或路径错误 | 运行 `catkin_make`，并传入正确的 `devel/setup.bash`。 |
| 没有图像输出 | 图像 topic 错误或时间戳无法匹配 | 检查 `common/camera_topic` 和 `max_time_diff`。 |
| `color_scans.pcd` 很稀疏或颜色错位 | LiDAR-相机外参、相机内参或时间同步有问题 | 检查标定和时间同步。 |
| `scans.pcd` 未生成 | 点云保存关闭或 SLAM 没有成功建图 | 检查 `pcd_save/pcd_save_en` 和 ROS 日志。 |
| ROS 找不到 package | 没有 source Catkin 环境 | 执行 `source devel/setup.bash`。 |

## 7. 后续模块如何使用这些输出

重建输出会被论文后续流程继续使用：

| 输出 | 后续用途 |
| --- | --- |
| `raw_images/` | 和裂缝分割 mask 关联；作为 OpenMVS 或语义投影图像输入。 |
| `visual_odom.txt` | 图像时间戳下的相机位姿，用于视觉重建和 mask 投影。 |
| `visual_odom_in_lidar_ts.txt` | LiDAR 时间戳下的位姿，用于点云帧对齐和位姿插值。 |
| `scans.pcd` | 基础几何点云，用于滤波、网格化、裂缝投影。 |
| `color_scans.pcd` | 彩色点云，用于可视化和 RGB 辅助检查。 |

完整论文流程中，重建之后通常还会继续执行：

1. 点云滤波和清理；
2. OpenMVS 工作空间生成；
3. 网格重建、优化和纹理化；
4. 裂缝 mask 从图像空间投影到三维模型；
5. 在三维模型或点云上进行裂缝宽度测量。

## 8. 建议后续开发任务

为了把这个重建后端做成更稳定的软件功能，建议后续优先补充：

1. 给脚本增加显式 `--output-dir` 参数，不只依赖 bag 文件名自动生成目录。
2. 增加 dry-run 检查模式，提前检查 bag topic、YAML 标定字段和 ROS package。
3. 每次运行生成结构化结果文件，例如 `reconstruction_summary.json`。
4. 根据 bag 总时长和当前播放时间输出进度。
5. 把当前硬编码的图像保存频率改成 YAML 或 launch 参数。
6. 准备一个小型 smoke-test ROS bag 或模拟 topic 测试，用于验证软件封装是否能跑通。

