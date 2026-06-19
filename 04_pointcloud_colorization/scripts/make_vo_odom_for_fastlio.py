import numpy as np
import logging
from scipy.spatial.transform import Rotation as R
from pose_interp import interpolate_poses
import os
import argparse

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


# Tlc = [
#         -0.9999487,
#         -0.0060765,
#         -0.0081089,
#         0.07317228523672217,
#         0.0080976,
#         0.0018884,
#         -0.9999654,
#         -0.048289129209371784,
#          0.0060916,
#         -0.9999797,
#         -0.0018391 ,
#         -0.06748855890332943,
#         0.0,
#         0.0,
#         0.0,
#         1.0
#     ]

# Tci = [
#         0.999916812873781,
#         0.0005159530022363122,
#         0.012888022534107549,
#         0.14019804643418302,
#         0.012897647385573308,
#         -0.02959566554827303,
#         -0.9994787377791847,
#         0.03680700838981143,
# 	-0.00013425445093047644,
# 	0.9995618191854141, 
# 	-0.029599858149797875, 
# 	-0.01400772611768411,
#         0.0,
#         0.0,
#         0.0,
#         1.0
#     ]


def read_scan_states(file_path):
    """
    读取scan_states文件并提取时间戳、位置和旋转数据.

    Args:
        file_path (str): 文件路径，包含scan_states文件的路径。

    Returns:
        list: 包含时间戳、位置和旋转数据的列表。每个元素是一个元组，包含三个值：
            - timestamp (float): 时间戳
            - position (numpy.ndarray): 位置数据，形状为(3,)
            - rotation (numpy.ndarray): 旋转数据，形状为(4,)，表示四元数 [x, y, z, w]

    Raises:
        FileNotFoundError: 如果文件路径无效或文件不存在，则引发此异常。
    """
    scan_states = []
    try:
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.split()
                if len(parts) < 8:
                    logging.error("Invalid line with insufficient data: %s", line)
                    continue
                timestamp = float(parts[0])
                position = np.array(parts[1:4], dtype=float)
                rotation = np.array(parts[4:8], dtype=float)  # quaternion [w, x, y, z]
                rotation = np.array([rotation[1],rotation[2],rotation[3],rotation[0]]) #[x, y, z, w]
                scan_states.append((timestamp, position, rotation))
    except FileNotFoundError:
        logging.error("File not found: %s", file_path)
        raise

    # save_camera_odometry(scan_states, path_to_TUM_odometry)
    return scan_states

def interpolate_scan_states(scan_states, timestamps):
    """使用SE3插值scan_states以匹配visual_odom的时间戳.

    Args:
        scan_states (list): A list of scan states.
        timestamps (list): A list of timestamps.

    Returns:
        list: A list of interpolated scan states.

    """
    pose_timestamps = [int(state[0] * 1e6) for state in scan_states] 

    abs_poses = [np.concatenate((state[1], state[2])) for state in scan_states]  # 合并位置和旋转为一个数组

    origin_timestamp = int(pose_timestamps[0])
    
    interpolated_poses = interpolate_poses(pose_timestamps, abs_poses, timestamps, origin_timestamp)
    
    interpolated_states = []
    for pose in interpolated_poses:
        timestamp, position, quaternion = pose[0], pose[1:4], pose[4:8]
        interpolated_states.append((timestamp, position, quaternion))
    return interpolated_states

def save_camera_odometry(camera_states, file_path):
    """
    将相机odometry数据保存到新的txt文件中.

    Args:
        camera_states (list): A list of camera states, where each state is a tuple of timestamp, position, and rotation.
        file_path (str): The file path to save the camera odometry data.

    Raises:
        IOError: If there is an error writing to the file.

    """
    try:
        with open(file_path, 'w') as file:
            for ts, position, rotation in camera_states: 
                file.write(f"{ts:.6f} {position[0]:.8f} {position[1]:.8f} {position[2]:.8f} {rotation[3]:.8f} {rotation[0]:.8f} {rotation[1]:.8f} {rotation[2]:.8f}\n") 

    except IOError as e:
        logging.error("Failed to write to file %s: %s", file_path, e)

def read_visual_odom_timestamps(file_path):
    """
    Reads the visual odometry timestamps file.

    Args:
        file_path (str): The path to the visual odometry timestamps file.

    Returns:
        list: A list of timestamps as integers.

    Raises:
        FileNotFoundError: If the specified file_path does not exist.
    """
    timestamps = []
    try:
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.split()
                if len(parts) < 8:
                    logging.error("Invalid line with insufficient data: %s", line)
                    continue
                timestamp = float(parts[0])
                timestamps.append((timestamp))
    except FileNotFoundError:
        logging.error("File not found: %s", file_path)
        raise
    timestamps_np = np.array(timestamps)
    timestamps_np = np.squeeze(timestamps_np[:]) * 1e6
    timestamps_list = timestamps_np.astype(np.int64).tolist()
    return timestamps_list

def main(args):
    """
    Main function to process visual odometry data.

    Args:
        args (object): Command-line arguments.

    Returns:
        None
    """

    path_to_scan_states = os.path.join(args.root_dir, "visual_odom_in_lidar_ts.txt")
    path_to_visual_odom_timestamp = os.path.join(args.root_dir, "visual_odom.txt")
    path_to_output = os.path.join(args.root_dir, "vo_interpolated_odom.txt")
    # path_to_TUM_odometry = os.path.join(args.root_dir, "visual_odom_in_lidar_ts_TUM.txt")

    scan_states = read_scan_states(path_to_scan_states)
    visual_timestamps = read_visual_odom_timestamps(path_to_visual_odom_timestamp)
    interpolated_states = interpolate_scan_states(scan_states, visual_timestamps)
    save_camera_odometry(interpolated_states, path_to_output)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_dir", type=str, help="Root directory path")
    args = parser.parse_args()

    main(args)
