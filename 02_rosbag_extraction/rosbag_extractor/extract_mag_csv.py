#!/usr/bin/env python

import argparse
import os
import sys

from tqdm import tqdm

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from rosbag_extractor.bag_utils import BagReader
from rosbag_extractor.csv_utils import base_header_columns, default_csv_path, write_rows_to_csv


MAG_FIELDNAMES = [
    "bag_time",
    "bag_time_nanoseconds",
    "stamp",
    "stamp_nanoseconds",
    "seq",
    "frame_id",
    "magnetic_field_x",
    "magnetic_field_y",
    "magnetic_field_z",
    "magnetic_field_covariance_00",
    "magnetic_field_covariance_01",
    "magnetic_field_covariance_02",
    "magnetic_field_covariance_03",
    "magnetic_field_covariance_04",
    "magnetic_field_covariance_05",
    "magnetic_field_covariance_06",
    "magnetic_field_covariance_07",
    "magnetic_field_covariance_08",
]


def magnetic_covariance_columns(values):
    return {
        f"magnetic_field_covariance_{index:02d}": value
        for index, value in enumerate(values)
    }


def mag_row(message, bag_time_ns):
    row = base_header_columns(message, bag_time_ns)
    row.update({
        "magnetic_field_x": message.magnetic_field.x,
        "magnetic_field_y": message.magnetic_field.y,
        "magnetic_field_z": message.magnetic_field.z,
    })
    row.update(magnetic_covariance_columns(message.magnetic_field_covariance))
    return row


def extract_mag_csv(bag_file, output_csv=None, topic="/wit/mag"):
    output_csv = output_csv or default_csv_path(bag_file, "mag")

    rows = []
    with BagReader(bag_file) as bag:
        total = bag.get_message_count(topic_filters=[topic])
        with tqdm(total=total, desc=f"Extracting magnetic field from {topic}", unit="msg", ascii=True) as pbar:
            for _, message, bag_time_ns, _ in bag.iter_messages(topics=[topic]):
                rows.append(mag_row(message, bag_time_ns))
                pbar.update(1)

    write_rows_to_csv(output_csv, MAG_FIELDNAMES, rows)
    return output_csv


def main():
    parser = argparse.ArgumentParser(description="Extract MagneticField topic data to CSV.")
    parser.add_argument("bag_file", help="Input ROS bag file")
    parser.add_argument("--output_csv", help="Output CSV path")
    parser.add_argument("--topic", default="/wit/mag", help="Magnetic field topic to extract")
    args = parser.parse_args()
    extract_mag_csv(args.bag_file, output_csv=args.output_csv, topic=args.topic)


if __name__ == "__main__":
    main()
