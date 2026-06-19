import argparse
import os
import shutil
from datetime import timedelta

import pandas as pd


def find_closest_timestamp(image_timestamp, dataframe, max_delta_seconds):
    time_diff = (dataframe["timestamp"] - image_timestamp).abs()
    min_diff = time_diff.min()
    if min_diff <= timedelta(seconds=max_delta_seconds):
        return dataframe.loc[time_diff.idxmin(), "timestamp"]
    return None


def process_masks(source_folder, destination_folder, odometry_file, max_delta_seconds=0.5, move=False):
    os.makedirs(destination_folder, exist_ok=True)

    data = pd.read_csv(odometry_file, delimiter=" ", usecols=[0], names=["timestamp"])
    data["timestamp"] = pd.to_datetime(data["timestamp"], unit="s")

    operation = shutil.move if move else shutil.copy2
    for filename in os.listdir(source_folder):
        if not filename.endswith(".png"):
            continue

        timestamp_str = filename[:-4]
        image_timestamp = pd.to_datetime(float(timestamp_str), unit="s")
        closest_timestamp = find_closest_timestamp(image_timestamp, data, max_delta_seconds)
        if closest_timestamp is None:
            continue

        new_filename = f"{float(closest_timestamp.timestamp())}.png"
        operation(
            os.path.join(source_folder, filename),
            os.path.join(destination_folder, new_filename),
        )


def main():
    parser = argparse.ArgumentParser(description="Match crack masks to odometry timestamps.")
    parser.add_argument("--source_folder", required=True, help="Folder containing original mask PNG files.")
    parser.add_argument("--destination_folder", required=True, help="Folder to write matched mask PNG files.")
    parser.add_argument("--odometry_file", required=True, help="Whitespace-delimited odometry file.")
    parser.add_argument("--max_delta_seconds", type=float, default=0.5, help="Maximum timestamp difference.")
    parser.add_argument("--move", action="store_true", help="Move files instead of copying them.")
    args = parser.parse_args()

    process_masks(
        args.source_folder,
        args.destination_folder,
        args.odometry_file,
        max_delta_seconds=args.max_delta_seconds,
        move=args.move,
    )
    print("Processing complete.")


if __name__ == "__main__":
    main()
