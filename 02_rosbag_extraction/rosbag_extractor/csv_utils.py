#!/usr/bin/env python

from __future__ import annotations

import csv
import os
from typing import Iterable

from rosbag_extractor.bag_utils import get_header_stamp_nsec


def default_csv_path(bag_file, suffix):
    bag_name = os.path.splitext(os.path.basename(bag_file))[0]
    return os.path.join(os.path.dirname(bag_file), f"{bag_name}-{suffix}.csv")


def ensure_parent_dir(output_path):
    parent = os.path.dirname(output_path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def base_header_columns(message, bag_time_ns):
    stamp_ns = get_header_stamp_nsec(message, bag_time_ns)
    header = getattr(message, "header", None)
    return {
        "bag_time": bag_time_ns / 1_000_000_000.0,
        "bag_time_nanoseconds": int(bag_time_ns),
        "stamp": stamp_ns / 1_000_000_000.0,
        "stamp_nanoseconds": int(stamp_ns),
        "seq": getattr(header, "seq", ""),
        "frame_id": getattr(header, "frame_id", ""),
    }


def write_rows_to_csv(output_path, fieldnames, rows: Iterable[dict]):
    ensure_parent_dir(output_path)
    with open(output_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
