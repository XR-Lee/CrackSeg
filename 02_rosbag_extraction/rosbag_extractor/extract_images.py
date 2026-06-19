#!/usr/bin/env python

import argparse
import logging
import os
import shutil
import subprocess
import sys
import tempfile

import cv2
import imageio_ffmpeg
import numpy as np
from tqdm import tqdm

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from rosbag_extractor.bag_utils import BagReader, get_header_stamp_sec

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("rosbag_extractor")

def extract_and_save_images(bag_file, output_folder, topic, img_format):
    """
    Extracts images from a ROS bag file and saves them to the specified folder.
    
    :param bag_file: Path to the ROS bag file.
    :param output_folder: Directory where images will be saved.
    :param topic: The ROS topic to extract images from.
    :param img_format: The format to save images in.
    """
    # If no output folder given, create one based on the bag file's name
    if not output_folder:
        bag_name = os.path.splitext(os.path.basename(bag_file))[0]
        folder_name = bag_name + '-images'
        output_folder = os.path.join(os.path.dirname(bag_file), folder_name)

    if os.path.exists(output_folder):
        shutil.rmtree(output_folder)
        print("Directory '% s' has been removed successfully" % output_folder)
    os.makedirs(output_folder, exist_ok=True)
    print("Directory '% s' has been created successfully" % output_folder)

    try:
        with BagReader(bag_file) as bag:
            logger.info("Successfully opened ROS bag with %s backend.", bag.backend)
            total_images = bag.get_message_count(topic_filters=[topic])
            with tqdm(total=total_images, desc=f"Extracting images from {topic}", unit="image", leave=True, position=0, ascii=True) as pbar:
                for _, msg, bag_time_ns, _ in bag.iter_messages(topics=[topic]):
                    try:
                        cv_image = message_to_cv_image(msg)
                        timestamp = get_header_stamp_sec(msg, bag_time_ns)
                        image_filename = os.path.join(output_folder, f"{timestamp:.6f}.{img_format}")
                        cv2.imwrite(image_filename, cv_image)
                    except Exception as e:
                        logger.error(f"Failed to extract image: {e}")
                    finally:
                        pbar.update(1)
    except Exception as e:
        logger.error(f"Error during image extraction: {e}")

    video_path =  f'{output_folder}/overview.mp4'

    images_to_video(output_folder,video_path, img_format)


def message_to_cv_image(msg):
    if hasattr(msg, "format") and hasattr(msg, "data") and not hasattr(msg, "encoding"):
        encoded = as_uint8_array(msg.data)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("Failed to decode compressed image data.")
        return image

    encoding = str(getattr(msg, "encoding", "")).lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    raw = as_uint8_array(msg.data)

    if raw.size < height * step:
        raise ValueError("Image buffer is smaller than expected from height and step.")

    row_data = raw[: height * step].reshape(height, step)

    if encoding in {"mono8", "8uc1"}:
        return row_data[:, :width].copy()

    if encoding in {"bgr8", "rgb8"}:
        image = row_data[:, : width * 3].reshape(height, width, 3).copy()
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    if encoding in {"bgra8", "rgba8"}:
        image = row_data[:, : width * 4].reshape(height, width, 4).copy()
        if encoding == "bgra8":
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

    if encoding in {"mono16", "16uc1", "16sc1"}:
        dtype = np.uint16 if encoding != "16sc1" else np.int16
        raw16 = np.frombuffer(raw[: height * step].tobytes(), dtype=dtype)
        image16 = raw16.reshape(height, step // np.dtype(dtype).itemsize)[:, :width].copy()
        return cv2.convertScaleAbs(image16, alpha=255.0 / max(float(image16.max()), 1.0))

    raise ValueError(f"Unsupported image encoding: {encoding}")


def as_uint8_array(data):
    if isinstance(data, np.ndarray):
        return np.asarray(data, dtype=np.uint8).reshape(-1)
    if isinstance(data, (bytes, bytearray, memoryview)):
        return np.frombuffer(data, dtype=np.uint8)
    return np.asarray(list(data), dtype=np.uint8).reshape(-1)

def images_to_video(img_folder, output_vid_file,img_format):
    """Creates a video from images in a specified folder.
    Args:
        img_folder (str): Path to the folder containing the images
        output_vid_file (str): Path for the output video file.
    """
    # Ensure the image folder exists
    if not os.path.exists(img_folder):
        logger.debug(f"The specified image folder '{img_folder}' does not exist.")
        return

    image_paths = sorted(
        os.path.join(img_folder, name)
        for name in os.listdir(img_folder)
        if name.lower().endswith(f".{img_format.lower()}")
    )
    if not image_paths:
        logger.warning("No images found for overview video generation.")
        return
    
    # Ensure output directory exists
    output_dir = os.path.dirname(output_vid_file)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=False)
    
    # Construct the ffmpeg command
    # command = [
    #     'ffmpeg', 
    #     '-y', 
    #     '-threads', '16', 
    #     '-framerate', '30'
    #     '-pattern_type', 'glob',
    #     '-i', f'{img_folder}/*.png', 
    #     # '-profile:v', 'baseline',
    #     '-level', '3.0', 
    #     '-c:v', 'libx264', 
    #     '-pix_fmt', 'yuv420p', 
    #         '-r', '30'
    #     # '-an', 
    #     '-v', 'error', output_vid_file
    # ]
        
    # ## high quality mp4
    # command = [
    #     'ffmpeg', '-y', 
    #     '-threads', '16', 
    #     '-framerate', '5', 
    #     '-pattern_type', 'glob', 
    #     '-i', f'{img_folder}/*.{img_format}', 
    #     '-c:v', 'libx264', 
    #     '-pix_fmt', 'yuv420p', 
    #     '-r', '5', 
    #     '-v', 'error', output_vid_file
    # ]

    ## low quality mp4 for overview
    ffmpeg_executable = shutil.which("ffmpeg")
    if ffmpeg_executable is None:
        try:
            ffmpeg_executable = imageio_ffmpeg.get_ffmpeg_exe()
        except Exception:
            logger.warning("ffmpeg is not available; skipping overview video generation.")
            return

    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False, encoding="utf-8") as handle:
        list_file = handle.name
        for image_path in image_paths:
            normalized = image_path.replace("\\", "/").replace("'", "'\\''")
            handle.write(f"file '{normalized}'\n")

    command = [
        ffmpeg_executable, '-y',
        '-threads', '16',
        '-f', 'concat',
        '-safe', '0',
        '-i', list_file,
        '-vf', 'fps=5',
        '-c:v', 'libx264',
        '-preset', 'ultrafast',  # Use a faster preset to reduce encoding time and file size
        '-b:v', '500k',  # Lower bitrate to reduce quality and file size
        '-pix_fmt', 'yuv420p',
        '-r', '5',
        '-v', 'error', output_vid_file
    ]

    try:
        logger.info(f'Running \"{" ".join(command)}\"')
        result = subprocess.call(command)
    finally:
        try:
            os.remove(list_file)
        except OSError:
            logger.warning("Failed to remove temporary ffmpeg input list: %s", list_file)

    if result != 0:
        logger.error("Generate video failed with exit code %s", result)
        return

    logger.info("Generate video success!")


def main():
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag file")
    parser.add_argument("--output_folder", help="Output folder for the images")
    parser.add_argument("--topic", default="/hikrobot_camera/rgb", help="Image topic to extract (default: /camera/image_raw)")
    parser.add_argument("--format", default="jpg", help="Format to save images in (default: jpg)")
    # parser.add_argument("video_file", default="")
    args = parser.parse_args()
    
    extract_and_save_images(args.bag_file, args.output_folder, args.topic, args.format)


if __name__ == "__main__":
    main()
