#!/usr/bin/env python

import argparse
import base64
import json
import logging
import math
import os
import sys

import numpy as np
from tqdm import tqdm

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from rosbag_extractor.bag_utils import (
    BagReader,
    get_message_fields,
    get_message_type,
    time_like_to_nsec,
    time_like_to_sec,
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("rosbag_message_extractor")


def sanitize_topic_name(topic):
    name = topic.strip("/").replace("/", "__")
    if not name:
        name = "root"
    return "".join(ch if ch.isalnum() or ch in ("_", "-", ".") else "_" for ch in name)


def default_output_folder(bag_file):
    bag_name = os.path.splitext(os.path.basename(bag_file))[0]
    return os.path.join(os.path.dirname(bag_file), f"{bag_name}-messages")


def is_ros_time_like(value):
    if hasattr(value, "to_sec") or (hasattr(value, "secs") and hasattr(value, "nsecs")):
        return True
    return hasattr(value, "sec") and hasattr(value, "nanosec")


def should_omit_binary_array(field_name, value, include_data_field):
    if include_data_field or field_name != "data" or len(value) <= 256:
        return False
    return all(isinstance(item, int) and 0 <= item <= 255 for item in value[: min(len(value), 32)])


def ros_value_to_python(value, include_data_field=False, field_name=None):
    if value is None or isinstance(value, (bool, int, str)):
        return value

    if isinstance(value, float):
        return value if math.isfinite(value) else str(value)

    if isinstance(value, np.generic):
        return ros_value_to_python(value.item(), include_data_field=include_data_field, field_name=field_name)

    if isinstance(value, (bytes, bytearray)):
        if include_data_field:
            return {
                "__encoding__": "base64",
                "data": base64.b64encode(bytes(value)).decode("ascii"),
            }
        return {
            "__omitted__": "binary_data",
            "length": len(value),
        }

    if isinstance(value, np.ndarray):
        if should_omit_binary_array(field_name, value.reshape(-1), include_data_field):
            return {
                "__omitted__": "uint8_array",
                "length": int(value.size),
            }
        return ros_value_to_python(
            value.tolist(),
            include_data_field=include_data_field,
            field_name=field_name,
        )

    if is_ros_time_like(value):
        return {
            "nanoseconds": time_like_to_nsec(value),
            "seconds": time_like_to_sec(value),
        }

    message_fields = get_message_fields(value)
    if message_fields is not None:
        data = {}
        for slot, slot_value in message_fields.items():
            data[slot] = ros_value_to_python(
                slot_value,
                include_data_field=include_data_field,
                field_name=slot,
            )
        data["_type"] = get_message_type(value)
        return data

    if isinstance(value, (list, tuple)):
        if should_omit_binary_array(field_name, value, include_data_field):
            return {
                "__omitted__": "uint8_array",
                "length": len(value),
            }
        return [
            ros_value_to_python(item, include_data_field=include_data_field)
            for item in value
        ]

    if hasattr(value, "tolist"):
        return ros_value_to_python(
            value.tolist(),
            include_data_field=include_data_field,
            field_name=field_name,
        )

    return str(value)


def list_topics(bag_file):
    with BagReader(bag_file) as bag:
        info = bag.get_topic_info()
        for topic, meta in sorted(info.items()):
            print(f"{topic}\t{meta.msg_type}\t{meta.message_count}")


def extract_messages(bag_file, output_folder=None, topics=None, include_data_field=False):
    if not output_folder:
        output_folder = default_output_folder(bag_file)
    os.makedirs(output_folder, exist_ok=True)

    writers = {}

    try:
        with BagReader(bag_file) as bag:
            topic_filters = topics if topics else None
            total_messages = bag.get_message_count(topic_filters=topic_filters)
            with tqdm(total=total_messages, desc="Extracting messages", unit="msg", ascii=True) as pbar:
                for topic, msg, bag_time_ns, msg_type in bag.iter_messages(topics=topic_filters):
                    topic_path = os.path.join(output_folder, f"{sanitize_topic_name(topic)}.jsonl")
                    if topic not in writers:
                        writers[topic] = open(topic_path, "w", encoding="utf-8")

                    record = {
                        "topic": topic,
                        "bag_time": bag_time_ns / 1_000_000_000.0,
                        "bag_time_nanoseconds": bag_time_ns,
                        "message_type": msg_type,
                        "message": ros_value_to_python(
                            msg,
                            include_data_field=include_data_field,
                        ),
                    }
                    json.dump(record, writers[topic], ensure_ascii=False)
                    writers[topic].write("\n")
                    pbar.update(1)
    finally:
        for writer in writers.values():
            writer.close()
    logger.info("ROS bag closed.")


def main():
    parser = argparse.ArgumentParser(
        description="List topics in a ROS bag or export arbitrary ROS messages to JSONL.",
    )
    parser.add_argument("bag_file", help="Input ROS bag file")
    parser.add_argument(
        "--output_folder",
        help="Output folder for JSONL files. Defaults to <bag-name>-messages next to the bag.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        dest="topics",
        help="Topic to extract. Provide multiple times to export multiple topics. Defaults to all topics.",
    )
    parser.add_argument(
        "--list-topics",
        action="store_true",
        help="Only print topic, message type, and message count.",
    )
    parser.add_argument(
        "--include-data-field",
        action="store_true",
        help="Include raw msg.data content as base64. Disabled by default to avoid huge JSON files.",
    )
    args = parser.parse_args()

    if args.list_topics:
        list_topics(args.bag_file)
        return

    extract_messages(
        args.bag_file,
        output_folder=args.output_folder,
        topics=args.topics,
        include_data_field=args.include_data_field,
    )


if __name__ == "__main__":
    main()
