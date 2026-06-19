import argparse
import json
import logging
import math
import shutil
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm


logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def load_calibration(path: Path):
    with path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    camera = data["camera_para"]
    width = int(camera["w"])
    height = int(camera["h"])
    fx = float(camera["fx"])
    fy = float(camera["fy"])
    cx = float(camera["cx"])
    cy = float(camera["cy"])
    distortion = [
        float(camera.get("k1", 0.0)),
        float(camera.get("k2", 0.0)),
        float(camera.get("k3", 0.0)),
        float(camera.get("k4", 0.0)),
    ]
    return width, height, fx, fy, cx, cy, distortion, data.get("Tlc")


def select_keyframes(vo_file: Path, output_file: Path, distance_threshold: float):
    output_file.parent.mkdir(parents=True, exist_ok=True)
    keyframes = []
    last_position = None

    with vo_file.open("r", encoding="utf-8") as file:
        for line_no, line in enumerate(file, start=1):
            parts = line.split()
            if len(parts) < 8:
                logging.warning("Skip line %d with insufficient columns: %s", line_no, line.strip())
                continue
            position = tuple(float(value) for value in parts[1:4])
            if last_position is None:
                keyframes.append(line)
                last_position = position
                continue

            distance = math.sqrt(sum((position[i] - last_position[i]) ** 2 for i in range(3)))
            if distance >= distance_threshold:
                keyframes.append(line)
                last_position = position

    if not keyframes:
        raise ValueError(f"No keyframes selected from {vo_file}")

    with output_file.open("w", encoding="utf-8") as file:
        file.writelines(keyframes)
    logging.info("Selected %d keyframes into %s", len(keyframes), output_file)


def read_trajectory(path: Path, quat_order: str):
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    states = []
    for row in data:
        timestamp = float(row[0])
        position = row[1:4].astype(float)
        quat = row[4:8].astype(float)
        if quat_order == "wxyz":
            quat_xyzw = np.array([quat[1], quat[2], quat[3], quat[0]], dtype=float)
        else:
            quat_xyzw = quat
        states.append((timestamp, position, quat_xyzw))
    return states


def image_timestamp(path: Path):
    try:
        return round(float(path.stem), 6)
    except ValueError:
        return None


def undistort_image(image, camera_matrix, distortion, model):
    height, width = image.shape[:2]
    size = (width, height)
    distortion = np.asarray(distortion, dtype=float)

    if model == "fisheye":
        new_camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            camera_matrix, distortion[:4], size, np.eye(3), balance=0.0, new_size=size, fov_scale=0.2
        )
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            camera_matrix, distortion[:4], np.eye(3), new_camera_matrix, size, cv2.CV_16SC2
        )
        return cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT), new_camera_matrix

    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, distortion[:4], size, alpha=1, newImgSize=size
    )
    return cv2.undistort(image, camera_matrix, distortion[:4], None, new_camera_matrix), new_camera_matrix


def write_selected_images(raw_dir: Path, output_dir: Path, states, camera_matrix, distortion, model):
    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)

    images = {}
    for image_path in raw_dir.iterdir():
        if image_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        timestamp = image_timestamp(image_path)
        if timestamp is not None:
            images[timestamp] = image_path

    new_camera_matrix = camera_matrix
    missing = 0
    for timestamp, _, _ in tqdm(states, desc="Selecting keyframe images"):
        image_path = images.get(round(timestamp, 6))
        if image_path is None:
            missing += 1
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            missing += 1
            continue
        undistorted, new_camera_matrix = undistort_image(image, camera_matrix, distortion, model)
        cv2.imwrite(str(output_dir / f"{timestamp:.6f}.jpg"), undistorted)

    if missing:
        logging.warning("Missing or unreadable images for %d keyframes", missing)
    return new_camera_matrix


def write_mvs_frames(output_file: Path, width: int, height: int, camera_matrix, states):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    lines = [f"MVS {width} {height}", str(len(states))]
    for idx, (timestamp, position, quat_xyzw) in enumerate(states):
        camera_pose = np.eye(4)
        camera_pose[:3, :3] = R.from_quat(quat_xyzw).as_matrix()
        camera_pose[:3, 3] = position
        camera_pose_inv = np.linalg.inv(camera_pose)
        qx, qy, qz, qw = R.from_matrix(camera_pose_inv[:3, :3]).as_quat()
        lines.append(
            f"{idx} {fx} {fy} {cx} {cy} {qw} {qx} {qy} {qz} "
            f"{position[0]} {position[1]} {position[2]} {timestamp:.6f}"
        )

    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text("\n".join(lines), encoding="utf-8")


def load_points(point_cloud_path: Path):
    cloud = o3d.io.read_point_cloud(str(point_cloud_path))
    points = np.asarray(cloud.points)
    if len(points):
        return cloud, points

    mesh = o3d.io.read_triangle_mesh(str(point_cloud_path))
    points = np.asarray(mesh.vertices)
    if not len(points):
        raise ValueError(f"No points or mesh vertices found in {point_cloud_path}")
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud, points


def append_visible_points(output_file: Path, point_cloud_path: Path, states, vertical_fov: float, horizontal_fov: float):
    cloud, vertices = load_points(point_cloud_path)
    logging.info("Loaded %d reconstruction points from %s", len(vertices), point_cloud_path)

    diameter = np.linalg.norm(np.asarray(cloud.get_max_bound()) - np.asarray(cloud.get_min_bound()))
    radius = diameter * 100
    point_to_keyframes = {i: [] for i in range(len(vertices))}
    index_map = np.arange(len(vertices))

    for idx, (_, position, quat_xyzw) in tqdm(enumerate(states), total=len(states), desc="Computing visible points"):
        camera_pose = np.eye(4)
        camera_pose[:3, :3] = R.from_quat(quat_xyzw).as_matrix()
        camera_pose[:3, 3] = position
        camera_pose_inv = np.linalg.inv(camera_pose)

        _, pt_map = cloud.hidden_point_removal(position, radius)
        visible_points = vertices[pt_map]
        visible_indices = index_map[pt_map]

        points_in_camera = camera_pose_inv[:3, :3] @ visible_points.T + camera_pose_inv[:3, 3].reshape((3, 1))
        points_in_camera = points_in_camera.T
        ranges = np.linalg.norm(points_in_camera, axis=1)
        valid = ranges > 0
        points_in_camera = points_in_camera[valid]
        ranges = ranges[valid]
        visible_indices = visible_indices[valid]

        theta_z = np.arccos(points_in_camera[:, 2] / ranges)
        zenith = np.arccos(points_in_camera[:, 1] / ranges)
        mask_zenith = np.logical_and(np.deg2rad(90 - vertical_fov / 2) < zenith, zenith < np.deg2rad(90 + vertical_fov / 2))
        mask_theta = np.logical_and(np.deg2rad(-horizontal_fov / 2) < theta_z, theta_z < np.deg2rad(horizontal_fov / 2))
        visible_indices_in_fov = visible_indices[np.logical_and(mask_zenith, mask_theta)]

        for point_index in visible_indices_in_fov:
            point_to_keyframes[int(point_index)].append(idx)

    result = []
    for point_index, keyframes in point_to_keyframes.items():
        if not keyframes:
            continue
        point = vertices[point_index]
        result.append(f"{point[0]} {point[1]} {point[2]} {len(keyframes)} {' '.join(map(str, keyframes))}")

    with output_file.open("a", encoding="utf-8") as file:
        file.write("\n")
        file.write(f"{len(result)}\n")
        file.write("\n".join(result))
    logging.info("Appended visibility for %d points to %s", len(result), output_file)


def parse_args():
    parser = argparse.ArgumentParser(description="Generate OpenMVS frame metadata from LiDAR-camera odometry.")
    parser.add_argument("--work-dir", required=True, type=Path, help="Reconstruction working directory.")
    parser.add_argument("--config", type=Path, help="Calibration JSON. Defaults to <work-dir>/index.json.")
    parser.add_argument("--raw-images", type=Path, help="Raw image folder. Defaults to <work-dir>/raw_images.")
    parser.add_argument("--selected-images", type=Path, help="Output keyframe image folder. Defaults to <work-dir>/selected_images.")
    parser.add_argument("--vo", type=Path, help="Interpolated odometry file. Defaults to <work-dir>/vo_interpolated_odom.txt.")
    parser.add_argument("--keyframes", type=Path, help="Output keyframe odometry file. Defaults to <work-dir>/vo_odom_keyframes.txt.")
    parser.add_argument("--point-cloud", type=Path, help="Point cloud or mesh for visibility. Defaults to <work-dir>/scans-clean-mls-clean.pcd.")
    parser.add_argument("--output", type=Path, help="OpenMVS frame metadata output. Defaults to <work-dir>/mvs_frame_result.txt.")
    parser.add_argument("--distance-threshold", type=float, default=0.5, help="Minimum keyframe translation distance in meters.")
    parser.add_argument("--vertical-fov", type=float, default=54.8)
    parser.add_argument("--horizontal-fov", type=float, default=58.4)
    parser.add_argument("--camera-model", choices=["pinhole", "fisheye"], default="pinhole")
    parser.add_argument("--quat-order", choices=["wxyz", "xyzw"], default="wxyz")
    return parser.parse_args()


def main():
    args = parse_args()
    work_dir = args.work_dir
    config = args.config or work_dir / "index.json"
    raw_images = args.raw_images or work_dir / "raw_images"
    selected_images = args.selected_images or work_dir / "selected_images"
    vo_file = args.vo or work_dir / "vo_interpolated_odom.txt"
    keyframe_file = args.keyframes or work_dir / "vo_odom_keyframes.txt"
    point_cloud = args.point_cloud or work_dir / "scans-clean-mls-clean.pcd"
    output_file = args.output or work_dir / "mvs_frame_result.txt"

    width, height, fx, fy, cx, cy, distortion, _ = load_calibration(config)
    camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=float)

    select_keyframes(vo_file, keyframe_file, args.distance_threshold)
    states = read_trajectory(keyframe_file, args.quat_order)
    new_camera_matrix = write_selected_images(raw_images, selected_images, states, camera_matrix, distortion, args.camera_model)
    write_mvs_frames(output_file, width, height, new_camera_matrix, states)
    append_visible_points(output_file, point_cloud, states, args.vertical_fov, args.horizontal_fov)


if __name__ == "__main__":
    main()
