#!/usr/bin/env python

import argparse
import logging
import os
import sys

import numpy as np
from tqdm import tqdm

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from rosbag_extractor.bag_utils import BagReader, get_header_stamp_nsec

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('point_cloud_extractor')

def extract_and_save_point_clouds(bag_file, output_folder, topic):
    """
    Extracts point cloud data and additional fields from a ROS bag file and saves them as PCD files along with JSON files for additional fields.
    
    :param bag_file: Path to the ROS bag file.
    :param output_folder: Directory where PCD and JSON files will be saved.
    :param topic: The point cloud topic to extract from.
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    try:
        with BagReader(bag_file) as bag:
            logger.info("Successfully opened ROS bag with %s backend.", bag.backend)
            total_points = bag.get_message_count(topic_filters=[topic])
            with tqdm(total=total_points, desc=f"Extracting point clouds from {topic}", unit="msg", ascii=True) as pbar:
                for _, msg, bag_time_ns, _ in bag.iter_messages(topics=[topic]):
                    timestamp = get_header_stamp_nsec(msg, bag_time_ns)
                    process_and_save_point_cloud(msg, output_folder, timestamp)
                    pbar.update(1)
    except Exception as e:
        logger.error(f"Error during point cloud extraction: {e}")
    finally:
        logger.info("ROS bag closed.")


POINT_FIELD_TO_DTYPE = {
    1: np.dtype("i1"),
    2: np.dtype("u1"),
    3: np.dtype("i2"),
    4: np.dtype("u2"),
    5: np.dtype("i4"),
    6: np.dtype("u4"),
    7: np.dtype("f4"),
    8: np.dtype("f8"),
}


def process_and_save_point_cloud(msg, output_folder, timestamp):
    """
    Converts a PointCloud2 message to a structured numpy array and saves it as binary PCD.
    """
    try:
        structured = point_cloud2_to_structured_array(msg)
        pcd_filename = os.path.join(output_folder, f"{timestamp}.pcd")
        write_pcd(pcd_filename, structured)
        logger.info(f"Saved {pcd_filename}")
    except Exception as e:
        logger.error(f"Failed to save point cloud for timestamp {timestamp}: {e}")


def point_cloud2_to_structured_array(msg):
    dtype = build_pointcloud_dtype(msg.fields, msg.point_step, msg.is_bigendian)
    raw = np.asarray(msg.data, dtype=np.uint8).reshape(-1)
    byte_data = raw.tobytes()
    point_count = int(msg.width) * int(msg.height)

    if int(msg.row_step) == int(msg.point_step) * int(msg.width):
        structured = np.frombuffer(byte_data, dtype=dtype, count=point_count)
    else:
        rows = []
        for row_index in range(int(msg.height)):
            offset = row_index * int(msg.row_step)
            rows.append(np.frombuffer(byte_data, dtype=dtype, count=int(msg.width), offset=offset))
        structured = np.concatenate(rows, axis=0) if rows else np.empty((0,), dtype=dtype)

    if {"x", "y", "z"}.issubset(structured.dtype.names):
        mask = np.isfinite(structured["x"]) & np.isfinite(structured["y"]) & np.isfinite(structured["z"])
        structured = structured[mask]

    return structured


def build_pointcloud_dtype(fields, point_step, is_bigendian):
    endian = ">" if bool(is_bigendian) else "<"
    dtype_fields = []
    offset = 0

    for field in sorted(fields, key=lambda item: item.offset):
        field_offset = int(field.offset)
        if field_offset > offset:
            dtype_fields.append((f"__pad_{offset}", f"V{field_offset - offset}"))

        base_dtype = POINT_FIELD_TO_DTYPE.get(int(field.datatype))
        if base_dtype is None:
            raise ValueError(f"Unsupported PointField datatype: {field.datatype}")

        if base_dtype.itemsize > 1:
            base_dtype = base_dtype.newbyteorder(endian)

        count = int(field.count)
        if count == 1:
            dtype_fields.append((field.name, base_dtype))
        else:
            dtype_fields.append((field.name, base_dtype, (count,)))

        offset = field_offset + base_dtype.itemsize * count

    if offset < int(point_step):
        dtype_fields.append((f"__pad_{offset}", f"V{int(point_step) - offset}"))

    return np.dtype(dtype_fields)


def write_pcd(output_path, structured_array):
    real_fields = []
    packed_dtype = []

    for name in structured_array.dtype.names or []:
        if name.startswith("__pad_"):
            continue

        field_dtype = structured_array.dtype[name]
        count = 1
        base_dtype = field_dtype
        if field_dtype.subdtype is not None:
            base_dtype, shape = field_dtype.subdtype
            count = int(np.prod(shape))

        base_dtype = np.dtype(base_dtype).newbyteorder("=")
        field_type = numpy_dtype_to_pcd_type(base_dtype)
        if field_type is None:
            continue

        real_fields.append((name, base_dtype.itemsize, field_type, count))
        if count == 1:
            packed_dtype.append((name, base_dtype))
        else:
            packed_dtype.append((name, base_dtype, (count,)))

    packed = np.empty(structured_array.shape[0], dtype=np.dtype(packed_dtype))
    for name, _, _, _ in real_fields:
        packed[name] = structured_array[name]

    header = "\n".join([
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS " + " ".join(name for name, _, _, _ in real_fields),
        "SIZE " + " ".join(str(size) for _, size, _, _ in real_fields),
        "TYPE " + " ".join(field_type for _, _, field_type, _ in real_fields),
        "COUNT " + " ".join(str(count) for _, _, _, count in real_fields),
        f"WIDTH {packed.shape[0]}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {packed.shape[0]}",
        "DATA binary",
        "",
    ])

    with open(output_path, "wb") as handle:
        handle.write(header.encode("ascii"))
        packed.tofile(handle)


def numpy_dtype_to_pcd_type(dtype):
    if dtype.kind == "f":
        return "F"
    if dtype.kind == "i":
        return "I"
    if dtype.kind == "u":
        return "U"
    return None

def main():
    parser = argparse.ArgumentParser(description="Extract point clouds and additional fields from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag file")
    parser.add_argument("output_folder", help="Output folder for the point clouds and additional fields")
    parser.add_argument("--topic", default="/Pandar/XT32_M2X/pandar", help="Point cloud topic to extract (default: /your_point_cloud_topic)")
    args = parser.parse_args()
    
    extract_and_save_point_clouds(args.bag_file, args.output_folder, args.topic)


if __name__ == "__main__":
    main()
