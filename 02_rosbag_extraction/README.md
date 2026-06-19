# Rosbag Extractor

`rosbag_extractor` provides command-line tools for exporting camera images, LiDAR point clouds, IMU/location/magnetic-field CSV files, and arbitrary ROS topics from `.bag` files.

The tools are designed for reproducible offline preprocessing. They can run either with ROS `rosbag` installed or with the pure-Python `rosbags` backend.

## Install

```bash
cd 02_rosbag_extraction
conda env create -f environment.yml
conda activate rosbag-extractor
pip install -e .
```

If you prefer `pip` only:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -e .
```

On Windows PowerShell, activate the virtual environment with `.venv\Scripts\Activate.ps1`.

## Commands

After installation, the following commands are available:

| Command | Purpose |
| --- | --- |
| `bag_info` | Print a `rosbag info`-style summary. |
| `image_extractor` | Export image topics and create `overview.mp4`. |
| `point_cloud_extractor` | Export `sensor_msgs/PointCloud2` messages as PCD files. |
| `message_extractor` | Export arbitrary topics as JSONL. |
| `imu_csv_extractor` | Export IMU messages to CSV. |
| `location_csv_extractor` | Export GPS/location messages to CSV. |
| `mag_csv_extractor` | Export magnetic-field messages to CSV. |

## Examples

Inspect a bag:

```bash
FULL_DATA=../CCIE2026_Crack3D_demo_data/full_dataset
RAW_BAG="$FULL_DATA/_2024-07-15-17-12-54.bag"
EXTRACTED="$FULL_DATA/extracted"

bag_info "$RAW_BAG"
message_extractor "$RAW_BAG" --list-topics
```

Export compressed camera frames:

```bash
image_extractor "$RAW_BAG" \
  --output_folder "$EXTRACTED/images" \
  --topic /hikrobot_camera/rgb/compressed \
  --format jpg
```

Export raw camera frames:

```bash
image_extractor "$RAW_BAG" \
  --output_folder "$EXTRACTED/images_raw" \
  --topic /hikrobot_camera/rgb \
  --format jpg
```

Export LiDAR frames:

```bash
point_cloud_extractor "$RAW_BAG" \
  "$EXTRACTED/pcds" \
  --topic /Pandar/XT32_M2X/pandar
```

Export IMU, location, and magnetic-field topics:

```bash
imu_csv_extractor "$RAW_BAG" \
  --output_csv "$EXTRACTED/csv/imu.csv" \
  --topic /wit/imu

location_csv_extractor "$RAW_BAG" \
  --output_csv "$EXTRACTED/csv/location.csv" \
  --topic /wit/location

mag_csv_extractor "$RAW_BAG" \
  --output_csv "$EXTRACTED/csv/mag.csv" \
  --topic /wit/mag
```

Export selected topics as JSONL:

```bash
message_extractor "$RAW_BAG" \
  --topic /hikrobot_camera/camera_info \
  --topic /wit/imu \
  --output_folder "$EXTRACTED/messages"
```

Export all topics as JSONL:

```bash
message_extractor "$RAW_BAG" \
  --output_folder "$EXTRACTED/messages_all"
```

Binary `msg.data` fields are skipped by default to keep JSONL files small. Use `--include-data-field` only when the raw binary payload is required:

```bash
message_extractor "$RAW_BAG" \
  --topic /hikrobot_camera/rgb/compressed \
  --output_folder "$EXTRACTED/messages" \
  --include-data-field
```

## Outputs

`image_extractor` writes image frames named by header timestamp and also creates `overview.mp4`.

`point_cloud_extractor` writes one binary PCD file per `PointCloud2` message. It preserves supported point fields and filters invalid `x/y/z` values.

The CSV extractors include bag time, header time, sequence/frame information, sensor values, and covariance fields where available.

## Important Notes

- `image_extractor` deletes the target output folder if it already exists, then recreates it.
- Topic names in the examples match the dataset used in the paper. Replace them if your bag uses different topic names.
- Keep `.bag`, generated `.pcd`, generated videos, generated CSV files, and extracted image folders out of Git.
