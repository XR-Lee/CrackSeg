#!/usr/bin/env python

import argparse
import os
import sys

from tqdm import tqdm

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from rosbag_extractor.bag_utils import BagReader
from rosbag_extractor.csv_utils import base_header_columns, default_csv_path, write_rows_to_csv


IMU_FIELDNAMES = [
    "bag_time",
    "bag_time_nanoseconds",
    "stamp",
    "stamp_nanoseconds",
    "seq",
    "frame_id",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "orientation_covariance_00",
    "orientation_covariance_01",
    "orientation_covariance_02",
    "orientation_covariance_03",
    "orientation_covariance_04",
    "orientation_covariance_05",
    "orientation_covariance_06",
    "orientation_covariance_07",
    "orientation_covariance_08",
    "angular_velocity_x",
    "angular_velocity_y",
    "angular_velocity_z",
    "angular_velocity_covariance_00",
    "angular_velocity_covariance_01",
    "angular_velocity_covariance_02",
    "angular_velocity_covariance_03",
    "angular_velocity_covariance_04",
    "angular_velocity_covariance_05",
    "angular_velocity_covariance_06",
    "angular_velocity_covariance_07",
    "angular_velocity_covariance_08",
    "linear_acceleration_x",
    "linear_acceleration_y",
    "linear_acceleration_z",
    "linear_acceleration_covariance_00",
    "linear_acceleration_covariance_01",
    "linear_acceleration_covariance_02",
    "linear_acceleration_covariance_03",
    "linear_acceleration_covariance_04",
    "linear_acceleration_covariance_05",
    "linear_acceleration_covariance_06",
    "linear_acceleration_covariance_07",
    "linear_acceleration_covariance_08",
]


def covariance_columns(prefix, values):
    data = {}
    for index, value in enumerate(values):
        data[f"{prefix}_{index:02d}"] = value
    return data


def imu_row(message, bag_time_ns):
    row = base_header_columns(message, bag_time_ns)
    row.update({
        "orientation_x": message.orientation.x,
        "orientation_y": message.orientation.y,
        "orientation_z": message.orientation.z,
        "orientation_w": message.orientation.w,
        "angular_velocity_x": message.angular_velocity.x,
        "angular_velocity_y": message.angular_velocity.y,
        "angular_velocity_z": message.angular_velocity.z,
        "linear_acceleration_x": message.linear_acceleration.x,
        "linear_acceleration_y": message.linear_acceleration.y,
        "linear_acceleration_z": message.linear_acceleration.z,
    })
    row.update(covariance_columns("orientation_covariance", message.orientation_covariance))
    row.update(covariance_columns("angular_velocity_covariance", message.angular_velocity_covariance))
    row.update(covariance_columns("linear_acceleration_covariance", message.linear_acceleration_covariance))
    return row


def extract_imu_csv(bag_file, output_csv=None, topic="/wit/imu"):
    output_csv = output_csv or default_csv_path(bag_file, "imu")

    rows = []
    with BagReader(bag_file) as bag:
        total = bag.get_message_count(topic_filters=[topic])
        with tqdm(total=total, desc=f"Extracting IMU from {topic}", unit="msg", ascii=True) as pbar:
            for _, message, bag_time_ns, _ in bag.iter_messages(topics=[topic]):
                rows.append(imu_row(message, bag_time_ns))
                pbar.update(1)

    write_rows_to_csv(output_csv, IMU_FIELDNAMES, rows)
    return output_csv


def main():
    parser = argparse.ArgumentParser(description="Extract IMU topic data to CSV.")
    parser.add_argument("bag_file", help="Input ROS bag file")
    parser.add_argument("--output_csv", help="Output CSV path")
    parser.add_argument("--topic", default="/wit/imu", help="IMU topic to extract")
    args = parser.parse_args()
    extract_imu_csv(args.bag_file, output_csv=args.output_csv, topic=args.topic)


if __name__ == "__main__":
    main()
