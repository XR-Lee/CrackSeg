# Pipeline I/O Contract

## Stage 1: Image Segmentation

Inputs:

- RGB crack images named by timestamp or any image filename.
- Optional DeepLab checkpoint.
- Optional SAM checkpoint for refinement.

Outputs:

- `mask35/*.png`: coarse DeepLab binary masks.
- `refine35/*.png`: SAM-refined binary masks.

Files used downstream:

- `refine35/*.png` copied or matched into `masks/` for 3D semantic projection.
- Selected refined masks copied into `mask_select/` for width measurement.

## Stage 2: ROS Bag Extraction

Inputs:

- ROS bag containing camera, LiDAR, IMU, location, and magnetic topics.

Outputs:

- `raw_images/*.jpg`
- LiDAR frame PCDs
- `imu.csv`, `location.csv`, `mag.csv`
- optional message JSONL files

Files used downstream:

- `raw_images/`
- LiDAR and navigation streams used by reconstruction.

## Stage 3: Reconstruction and Fusion

Inputs:

- ROS bag
- ROS Noetic workspace
- FAST-LIO/R3LIVE/OpenMVS configuration
- Camera and LiDAR calibration

Outputs:

- `{bag_name}_reconstruct/scans.pcd`
- `{bag_name}_reconstruct/color_scans.pcd`
- `{bag_name}_reconstruct/visual_odom.txt`
- `{bag_name}_reconstruct/visual_odom_in_lidar_ts.txt`
- `{bag_name}_reconstruct/raw_images/`

Files used downstream:

- fused point cloud, usually `scans.pcd` or a cropped/cleaned variant.
- camera odometry text files.
- raw images.

## Stage 4: Point-Cloud Colorization and Semantic Projection

Inputs:

- `scans-crop.pcd`, `ronghe.pcd`, or `scans.pcd`
- `vo_interpolated_odom.txt`
- `raw_images/*.jpg`
- optional `masks/*.png`

Outputs:

- `scans-crop.pcd`
- `filtered_pcd/*_beforeNID.pcd`
- `cloudInWorldWithRGB.pcd`
- optional `cloudInWorldWithRGBandMask.pcd`

Files used downstream:

- `filtered_pcd/` visible per-frame point clouds for width measurement.

## Stage 5: 3D Crack-Width Measurement

Inputs:

- `filtered_pcd/{timestamp}.pcd`
- `mask_select/{timestamp}.png`
- `raw_images/{timestamp}.jpg`
- `selected_points.json` for non-interactive runs, or `--manual-select`

Outputs:

- `norm_masks/*_norm.png`
- `distance_mask/*_distance.png`
- `distance_mask/*_3d_pts_on_img.png`
- `edt_skeleton/*_edt.png`
- `edt_skeleton/*_skeleton.png`
- `edt_skeleton/*_skeleton_edge_pts.png`
- `crack_width_3d_results.json`

Main result:

- `crack_width_3d_results.json`
