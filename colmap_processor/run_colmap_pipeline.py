#!/usr/bin/env python3

import os
import subprocess
import argparse

def run_command(command):
    print(f"Running command: {' '.join(command)}")
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for line in process.stdout:
        print(line, end='')
    process.wait()
    if process.returncode != 0:
        raise RuntimeError(f"Command failed with exit code {process.returncode}: {' '.join(command)}")

def main():
    parser = argparse.ArgumentParser(description="Run the COLMAP pipeline.")
    parser.add_argument("--bag_path", required=True, help="Path to the rosbag file.")
    parser.add_argument("--image_topic", required=True, help="Image topic name.")
    parser.add_argument("--camera_info_topic", required=True, help="Camera info topic name.")
    parser.add_argument("--output_dir", required=True, help="Directory to store the COLMAP data.")
    args = parser.parse_args()

    # Step 1: Run the data extractor
    extractor_executable = "bazel"
    run_command([extractor_executable, "run", "//colmap_processor:colmap_processor", "--", args.bag_path, args.image_topic, args.camera_info_topic, args.output_dir])

    # Step 2: Run COLMAP
    database_path = os.path.join(args.output_dir, "database.db")
    image_path = os.path.join(args.output_dir, "images")
    sparse_path = os.path.join(args.output_dir, "sparse")

    if not os.path.exists(sparse_path):
        os.makedirs(sparse_path)

    # Feature extraction
    run_command(["colmap", "feature_extractor", "--database_path", database_path, "--image_path", image_path, "--ImageReader.single_camera=true"])

    # Feature matching
    run_command(["colmap", "exhaustive_matcher", "--database_path", database_path])

    # Mapping
    run_command(["colmap", "mapper", "--database_path", database_path, "--image_path", image_path, "--output_path", sparse_path])

if __name__ == "__main__":
    main()
