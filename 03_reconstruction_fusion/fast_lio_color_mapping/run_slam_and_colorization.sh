#!/bin/bash
set -euo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 3 ]; then
    echo "Usage: $0 /mnt/e/crackmeasurement/CCIE2026_Crack3D_demo_data/full_dataset/_2024-07-15-17-12-54.bag [devel/setup.bash] [launch_file]"
    exit 1
fi

rosbag_path="$1"
catkin_setup="${2:-devel/setup.bash}"
launch_file="${3:-mapping_zhongnan.launch}"

if [ ! -f "$rosbag_path" ]; then
    echo "ROS bag not found: $rosbag_path" >&2
    exit 1
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
    echo "ROS Noetic setup file not found: /opt/ros/noetic/setup.bash" >&2
    exit 1
fi

if [ ! -f "$catkin_setup" ]; then
    echo "Catkin setup file not found: $catkin_setup" >&2
    echo "Build the workspace first, then pass the path to devel/setup.bash." >&2
    exit 1
fi

rosbag_dir=$(dirname "$rosbag_path")
rosbag_basename=$(basename "$rosbag_path" .bag)
reconstruction_dir="${rosbag_dir}/${rosbag_basename}_reconstruct/"
mkdir -p "$reconstruction_dir"

echo "Reconstruction directory: $reconstruction_dir"

source /opt/ros/noetic/setup.bash
source "$catkin_setup"

roslaunch fast_lio_color_mapping "$launch_file" \
    rosbag_path:="$rosbag_path" \
    reconstruction_path:="$reconstruction_dir"
