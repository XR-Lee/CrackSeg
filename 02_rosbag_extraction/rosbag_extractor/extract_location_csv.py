#!/usr/bin/env python

import argparse
import os
import sys

from tqdm import tqdm

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from rosbag_extractor.bag_utils import BagReader
from rosbag_extractor.csv_utils import base_header_columns, default_csv_path, write_rows_to_csv


LOCATION_FIELDNAMES = [
    "bag_time",
    "bag_time_nanoseconds",
    "stamp",
    "stamp_nanoseconds",
    "seq",
    "frame_id",
    "status",
    "service",
    "latitude",
    "longitude",
    "altitude",
    "position_covariance_00",
    "position_covariance_01",
    "position_covariance_02",
    "position_covariance_03",
    "position_covariance_04",
    "position_covariance_05",
    "position_covariance_06",
    "position_covariance_07",
    "position_covariance_08",
    "position_covariance_type",
]


def position_covariance_columns(values):
    return {
        f"position_covariance_{index:02d}": value
        for index, value in enumerate(values)
    }


def location_row(message, bag_time_ns):
    row = base_header_columns(message, bag_time_ns)
    row.update({
        "status": message.status.status,
        "service": message.status.service,
        "latitude": message.latitude,
        "longitude": message.longitude,
        "altitude": message.altitude,
        "position_covariance_type": message.position_covariance_type,
    })
    row.update(position_covariance_columns(message.position_covariance))
    return row


def extract_location_csv(bag_file, output_csv=None, topic="/wit/location"):
    output_csv = output_csv or default_csv_path(bag_file, "location")

    rows = []
    with BagReader(bag_file) as bag:
        total = bag.get_message_count(topic_filters=[topic])
        with tqdm(total=total, desc=f"Extracting location from {topic}", unit="msg", ascii=True) as pbar:
            for _, message, bag_time_ns, _ in bag.iter_messages(topics=[topic]):
                rows.append(location_row(message, bag_time_ns))
                pbar.update(1)

    write_rows_to_csv(output_csv, LOCATION_FIELDNAMES, rows)
    return output_csv


def main():
    parser = argparse.ArgumentParser(description="Extract NavSatFix topic data to CSV.")
    parser.add_argument("bag_file", help="Input ROS bag file")
    parser.add_argument("--output_csv", help="Output CSV path")
    parser.add_argument("--topic", default="/wit/location", help="Location topic to extract")
    args = parser.parse_args()
    extract_location_csv(args.bag_file, output_csv=args.output_csv, topic=args.topic)


if __name__ == "__main__":
    main()
