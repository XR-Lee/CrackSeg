import argparse
import logging
from pathlib import Path

import numpy as np

from pose_interp import interpolate_poses


logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


def read_scan_states(file_path: Path):
    """Read timestamp, position, and quaternion from FAST-LIO scan-state output."""
    scan_states = []
    with file_path.open("r", encoding="utf-8") as file:
        for line_no, line in enumerate(file, start=1):
            parts = line.split()
            if len(parts) < 8:
                logging.warning("Skip line %d with insufficient columns: %s", line_no, line.strip())
                continue
            timestamp = float(parts[0])
            position = np.array(parts[1:4], dtype=float)
            quat_wxyz = np.array(parts[4:8], dtype=float)
            quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
            scan_states.append((timestamp, position, quat_xyzw))

    if not scan_states:
        raise ValueError(f"No valid scan states found in {file_path}")
    return scan_states


def read_visual_odom_timestamps(file_path: Path):
    """Read visual odometry timestamps and convert seconds to integer microseconds."""
    timestamps = []
    with file_path.open("r", encoding="utf-8") as file:
        for line_no, line in enumerate(file, start=1):
            parts = line.split()
            if len(parts) < 1:
                logging.warning("Skip empty line %d", line_no)
                continue
            timestamps.append(float(parts[0]))

    if not timestamps:
        raise ValueError(f"No timestamps found in {file_path}")
    return (np.asarray(timestamps) * 1e6).astype(np.int64).tolist()


def interpolate_scan_states(scan_states, timestamps_us):
    pose_timestamps_us = [int(state[0] * 1e6) for state in scan_states]
    abs_poses = [np.concatenate((state[1], state[2])) for state in scan_states]
    origin_timestamp = pose_timestamps_us[0]
    interpolated = interpolate_poses(pose_timestamps_us, abs_poses, timestamps_us, origin_timestamp)

    interpolated_states = []
    for pose in interpolated:
        timestamp, position, quaternion = pose[0], pose[1:4], pose[4:8]
        interpolated_states.append((timestamp, position, quaternion))
    return interpolated_states


def save_tum_odometry(states, file_path: Path):
    """Save states as timestamp tx ty tz qw qx qy qz."""
    file_path.parent.mkdir(parents=True, exist_ok=True)
    with file_path.open("w", encoding="utf-8") as file:
        for timestamp, position, quat_xyzw in states:
            file.write(
                f"{timestamp:.6f} "
                f"{position[0]:.8f} {position[1]:.8f} {position[2]:.8f} "
                f"{quat_xyzw[3]:.8f} {quat_xyzw[0]:.8f} {quat_xyzw[1]:.8f} {quat_xyzw[2]:.8f}\n"
            )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interpolate FAST-LIO scan-state odometry to visual-image timestamps."
    )
    parser.add_argument("--scan-states", required=True, type=Path, help="FAST-LIO odometry at LiDAR timestamps.")
    parser.add_argument("--visual-odom", required=True, type=Path, help="File whose first column is image timestamp.")
    parser.add_argument("--output", required=True, type=Path, help="Interpolated odometry output path.")
    parser.add_argument("--tum-output", type=Path, help="Optional original scan-state odometry in TUM format.")
    return parser.parse_args()


def main():
    args = parse_args()
    scan_states = read_scan_states(args.scan_states)

    if args.tum_output:
        save_tum_odometry(scan_states, args.tum_output)

    visual_timestamps_us = read_visual_odom_timestamps(args.visual_odom)
    interpolated_states = interpolate_scan_states(scan_states, visual_timestamps_us)
    save_tum_odometry(interpolated_states, args.output)
    logging.info("Wrote interpolated odometry to %s", args.output)


if __name__ == "__main__":
    main()
