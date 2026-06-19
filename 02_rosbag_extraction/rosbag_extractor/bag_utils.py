#!/usr/bin/env python

from __future__ import annotations

from dataclasses import dataclass
from numbers import Integral
from pathlib import Path


try:
    import rosbag as rosbag_module
except ImportError:
    rosbag_module = None

try:
    from rosbags.highlevel import AnyReader
except ImportError:
    AnyReader = None


@dataclass(frozen=True)
class TopicMetadata:
    msg_type: str
    message_count: int


class BagReader:
    def __init__(self, bag_file):
        self.bag_file = bag_file
        self.backend = None
        self.reader = None

    def open(self):
        if self.reader is not None:
            return self

        if rosbag_module is not None:
            self.backend = "rosbag"
            self.reader = rosbag_module.Bag(self.bag_file, "r")
            return self

        if AnyReader is not None:
            self.backend = "rosbags"
            self.reader = AnyReader([Path(self.bag_file)])
            self.reader.open()
            return self

        raise ImportError(
            "No ROS bag backend available. Install ROS or add the 'rosbags' package.",
        )

    def close(self):
        if self.reader is not None:
            self.reader.close()
            self.reader = None

    def __enter__(self):
        return self.open()

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def get_topic_info(self):
        if self.reader is None:
            self.open()

        if self.backend == "rosbag":
            info = self.reader.get_type_and_topic_info()
            return {
                topic: TopicMetadata(
                    msg_type=metadata.msg_type,
                    message_count=metadata.message_count,
                )
                for topic, metadata in info.topics.items()
            }

        return {
            topic: TopicMetadata(
                msg_type=metadata.msgtype,
                message_count=metadata.msgcount,
            )
            for topic, metadata in self.reader.topics.items()
        }

    def get_message_count(self, topic_filters=None):
        if self.reader is None:
            self.open()

        if self.backend == "rosbag":
            return self.reader.get_message_count(topic_filters=topic_filters)

        topic_info = self.get_topic_info()
        if not topic_filters:
            return sum(metadata.message_count for metadata in topic_info.values())

        total = 0
        for topic in dict.fromkeys(topic_filters):
            metadata = topic_info.get(topic)
            if metadata is not None:
                total += metadata.message_count
        return total

    def iter_messages(self, topics=None):
        if self.reader is None:
            self.open()

        if self.backend == "rosbag":
            for topic, message, stamp in self.reader.read_messages(topics=topics):
                yield topic, message, time_like_to_nsec(stamp), get_message_type(message)
            return

        connections = None
        if topics:
            connections = []
            for topic in dict.fromkeys(topics):
                metadata = self.reader.topics.get(topic)
                if metadata is not None:
                    connections.extend(metadata.connections)

        for connection, timestamp, rawdata in self.reader.messages(connections=connections):
            message = self.reader.deserialize(rawdata, connection.msgtype)
            yield connection.topic, message, int(timestamp), connection.msgtype


def get_message_type(message):
    return getattr(message, "_type", None) or getattr(message, "__msgtype__", None) or type(message).__name__


def get_message_fields(message):
    if hasattr(message, "__slots__") and hasattr(message, "_slot_types"):
        return {
            slot: getattr(message, slot)
            for slot in message.__slots__
        }

    annotations = getattr(type(message), "__annotations__", None) or getattr(message, "__annotations__", None)
    if annotations:
        fields = {}
        for name, annotation in annotations.items():
            if name.startswith("__"):
                continue
            if "ClassVar" in str(annotation):
                continue
            if hasattr(message, name):
                fields[name] = getattr(message, name)
        if fields:
            return fields

    if hasattr(message, "__dict__"):
        return {
            key: value
            for key, value in vars(message).items()
            if not key.startswith("__")
        }

    return None


def time_like_to_nsec(value):
    if isinstance(value, Integral):
        return int(value)

    if hasattr(value, "to_nsec"):
        return int(value.to_nsec())

    if hasattr(value, "secs") and hasattr(value, "nsecs"):
        return int(value.secs) * 1_000_000_000 + int(value.nsecs)

    if hasattr(value, "sec") and hasattr(value, "nanosec"):
        return int(value.sec) * 1_000_000_000 + int(value.nanosec)

    raise TypeError(f"Unsupported time value: {type(value)!r}")


def time_like_to_sec(value):
    return time_like_to_nsec(value) / 1_000_000_000.0


def get_header_stamp_nsec(message, fallback_nsec):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return int(fallback_nsec)
    return time_like_to_nsec(stamp)


def get_header_stamp_sec(message, fallback_nsec):
    return get_header_stamp_nsec(message, fallback_nsec) / 1_000_000_000.0
