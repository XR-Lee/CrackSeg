#!/usr/bin/env python

from __future__ import annotations

import argparse
import os
import sys
from collections import Counter
from datetime import datetime

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from rosbag_extractor.bag_utils import BagReader

try:
    from rosbags.rosbag1.reader import decompressors as rosbag1_decompressors
except ImportError:
    rosbag1_decompressors = {}


def raw_reader(bag):
    if bag.backend == "rosbag":
        return bag.reader
    return bag.reader.readers[0]


def detect_storage_format(bag):
    module_name = type(raw_reader(bag)).__module__
    if "rosbag1" in module_name:
        return "rosbag1"
    if "rosbag2" in module_name:
        return "rosbag2"
    if bag.backend == "rosbag":
        return "rosbag1"
    return "unknown"


def read_bag_version(path):
    if not os.path.isfile(path):
        return None

    try:
        with open(path, "rb") as handle:
            first_line = handle.readline().decode("ascii", errors="ignore").strip()
    except OSError:
        return None

    if first_line.startswith("#ROSBAG V"):
        return first_line.split("V", 1)[1]
    return None


def display_msg_type(msg_type, storage_format):
    if storage_format == "rosbag1":
        return msg_type.replace("/msg/", "/", 1)
    return msg_type


def format_timestamp(timestamp_ns):
    timestamp_sec = timestamp_ns / 1_000_000_000.0
    return f"{datetime.fromtimestamp(timestamp_sec).astimezone().isoformat()} ({timestamp_sec:.6f})"


def format_duration(duration_ns):
    total_seconds = duration_ns / 1_000_000_000.0
    hours = duration_ns // 3_600_000_000_000
    minutes = (duration_ns // 60_000_000_000) % 60
    seconds = (duration_ns // 1_000_000_000) % 60
    milliseconds = (duration_ns % 1_000_000_000) // 1_000_000
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}.{milliseconds:03d} ({total_seconds:.6f}s)"


def format_size(size_bytes):
    units = ["B", "KB", "MB", "GB", "TB"]
    size = float(size_bytes)
    unit = units[0]
    for unit in units:
        if size < 1024.0 or unit == units[-1]:
            break
        size /= 1024.0
    return f"{size:.2f} {unit} ({size_bytes} bytes)"


def compression_summary(bag, storage_format):
    reader = raw_reader(bag)

    if storage_format == "rosbag1" and rosbag1_decompressors:
        decompressor_to_name = {func: name for name, func in rosbag1_decompressors.items()}
        counts = Counter(
            decompressor_to_name.get(chunk.decompressor, "unknown")
            for chunk in reader.chunks.values()
        )
        total = sum(counts.values())
        if len(counts) == 1:
            name, count = next(iter(counts.items()))
            return f"{name} [{count}/{total} chunks]"
        parts = [f"{name} [{count} chunks]" for name, count in sorted(counts.items())]
        return ", ".join(parts)

    return "unknown"


def type_summaries(bag, storage_format):
    reader = raw_reader(bag)
    summaries = {}

    if bag.backend == "rosbag":
        topic_info = bag.get_topic_info()
        for topic, meta in topic_info.items():
            summaries.setdefault(meta.msg_type, {"digest": None, "count": 0})
            summaries[meta.msg_type]["count"] += meta.message_count
        return summaries

    for metadata in reader.topics.values():
        digest = None
        if metadata.connections:
            digest = metadata.connections[0].digest
        summaries.setdefault(metadata.msgtype, {"digest": digest, "count": 0})
        summaries[metadata.msgtype]["count"] += metadata.msgcount
    return summaries


def topic_summaries(bag, storage_format):
    reader = raw_reader(bag)
    rows = []

    if bag.backend == "rosbag":
        for topic, meta in sorted(bag.get_topic_info().items()):
            rows.append((topic, display_msg_type(meta.msg_type, storage_format), meta.message_count))
        return rows

    for topic, metadata in sorted(reader.topics.items()):
        rows.append((topic, display_msg_type(metadata.msgtype, storage_format), metadata.msgcount))
    return rows


def build_summary(bag_path):
    with BagReader(bag_path) as bag:
        reader = raw_reader(bag)
        storage_format = detect_storage_format(bag)
        version = read_bag_version(bag_path)
        summary_lines = [
            f"path:        {os.path.abspath(bag_path)}",
            f"storage:     {storage_format}",
            f"version:     {version or 'unknown'}",
            f"duration:    {format_duration(reader.duration)}",
            f"start:       {format_timestamp(reader.start_time)}",
            f"end:         {format_timestamp(reader.end_time)}",
            f"size:        {format_size(os.path.getsize(bag_path))}",
            f"messages:    {reader.message_count}",
            f"compression: {compression_summary(bag, storage_format)}",
            "types:",
        ]

        type_info = type_summaries(bag, storage_format)
        for msg_type in sorted(type_info):
            digest = type_info[msg_type]["digest"]
            digest_suffix = f" [{digest}]" if digest else ""
            summary_lines.append(f"  {display_msg_type(msg_type, storage_format)}{digest_suffix}")

        summary_lines.append("topics:")
        topic_info = topic_summaries(bag, storage_format)
        topic_width = max(len(topic) for topic, _, _ in topic_info) if topic_info else 0
        count_width = max(len(str(count)) for _, _, count in topic_info) if topic_info else 0
        for topic, msg_type, count in topic_info:
            summary_lines.append(
                f"  {topic.ljust(topic_width)}   {str(count).rjust(count_width)} msgs    : {msg_type}",
            )

        return "\n".join(summary_lines)


def main():
    parser = argparse.ArgumentParser(description="Show a rosbag info-style summary without requiring ROS.")
    parser.add_argument("bag_file", help="Input ROS bag file")
    args = parser.parse_args()
    print(build_summary(args.bag_file))


if __name__ == "__main__":
    main()
