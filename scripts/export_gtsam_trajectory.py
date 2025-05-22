#!/usr/bin/env python3

import csv
import math

def read_gtsam_data(gtsam_csv_path):
    """Read GTSAM CSV data with correct column mapping"""
    data_rows = []

    with open(gtsam_csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # Skip header
        print(f"Original header: {header}")

        for row in reader:
            if len(row) >= 7:
                data_rows.append({
                    'x': float(row[0]),           # X position
                    'y': float(row[1]),           # Y position
                    'z': float(row[2]),           # Z position
                    'roll': float(row[3]),        # Roll (degrees)
                    'pitch': float(row[4]),       # Pitch (degrees)
                    'yaw': float(row[5]),         # Yaw (degrees)
                    'kf_id': int(row[6])          # Keyframe ID
                })

    # Remove duplicates by kf_id (keep last)
    unique_data = {}
    for row in data_rows:
        unique_data[row['kf_id']] = row

    # Sort by kf_id
    sorted_data = sorted(unique_data.values(), key=lambda x: x['kf_id'])
    return sorted_data

def export_trajectory_csv(data, output_path):
    """Export trajectory data to standard CSV format"""
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)

        # Write header
        writer.writerow(['type', 'id', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'timestamp', 'method'])

        # Write data
        for point in data:
            # Convert angles from degrees to radians
            roll_rad = math.radians(point['roll'])
            pitch_rad = math.radians(point['pitch'])
            yaw_rad = math.radians(point['yaw'])

            writer.writerow([
                'gtsam',                    # type
                point['kf_id'],             # id
                f"{point['x']:.6f}",        # x
                f"{point['y']:.6f}",        # y
                f"{point['z']:.6f}",        # z
                f"{roll_rad:.6f}",          # roll (radians)
                f"{pitch_rad:.6f}",         # pitch (radians)
                f"{yaw_rad:.6f}",           # yaw (radians)
                0,                          # timestamp (not available)
                'gtsam_optimized'           # method
            ])

def export_simple_trajectory(data, output_path):
    """Export simplified trajectory for easy plotting"""
    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)

        # Write header
        writer.writerow(['kf_id', 'x', 'y', 'z', 'roll_deg', 'pitch_deg', 'yaw_deg'])

        # Write data
        for point in data:
            writer.writerow([
                point['kf_id'],
                f"{point['x']:.6f}",
                f"{point['y']:.6f}",
                f"{point['z']:.6f}",
                f"{point['roll']:.2f}",
                f"{point['pitch']:.2f}",
                f"{point['yaw']:.2f}"
            ])

def main():
    gtsam_input = "/data/robot/bags/house11/gtsam.csv"
    standard_output = "/data/robot/bags/house11/gtsam_trajectory.csv"
    simple_output = "/data/robot/bags/house11/gtsam_simple.csv"

    print("Loading GTSAM trajectory data...")
    data = read_gtsam_data(gtsam_input)

    if not data:
        print("Failed to load GTSAM data")
        return

    print(f"Loaded {len(data)} unique trajectory points")
    print(f"Keyframe ID range: {data[0]['kf_id']} to {data[-1]['kf_id']}")

    # Export in standard format compatible with plot_trajectory.py
    print(f"\nExporting standard format to: {standard_output}")
    export_trajectory_csv(data, standard_output)

    # Export simplified format for easy analysis
    print(f"Exporting simple format to: {simple_output}")
    export_simple_trajectory(data, simple_output)

    print(f"\nExport completed successfully!")
    print(f"Use the exported files with plotting tools:")
    print(f"  python3 scripts/plot_trajectory.py {standard_output} --gtsam-only --show")

if __name__ == "__main__":
    main()