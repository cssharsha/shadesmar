#!/usr/bin/env python3

import sys
sys.path.append('/workspace')

def simple_read_gtsam_csv(gtsam_csv_path):
    """Simple test of GTSAM CSV reading"""
    try:
        import csv
        data_rows = []

        with open(gtsam_csv_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # Read header
            print(f"GTSAM CSV header: {header}")

            for row in reader:
                if len(row) >= 7:  # Must have at least 7 columns
                    data_rows.append({
                        'x': float(row[0]),           # X position
                        'y_pos': float(row[1]),       # Y position
                        'z': float(row[2]),           # Z position
                        'r': float(row[3]),           # Roll
                        'p': float(row[4]),           # Pitch
                        'y_angle': float(row[5]),     # Yaw (the second 'y')
                        'kf_id': int(row[6])          # Keyframe ID
                    })

        print(f"Reading GTSAM CSV with {len(data_rows)} rows")

        if not data_rows:
            print("No data found in GTSAM CSV")
            return False

        # Show first few rows
        print("\nFirst 5 rows:")
        for i, row in enumerate(data_rows[:5]):
            print(f"  Row {i+1}: x={row['x']:.3f}, y={row['y_pos']:.3f}, z={row['z']:.3f}, "
                  f"r={row['r']:.1f}, p={row['p']:.1f}, yaw={row['y_angle']:.1f}, kf_id={row['kf_id']}")

        # Show data ranges
        x_vals = [row['x'] for row in data_rows]
        y_vals = [row['y_pos'] for row in data_rows]
        z_vals = [row['z'] for row in data_rows]

        print(f"\nPosition ranges:")
        print(f"  X: [{min(x_vals):.3f}, {max(x_vals):.3f}]")
        print(f"  Y: [{min(y_vals):.3f}, {max(y_vals):.3f}]")
        print(f"  Z: [{min(z_vals):.3f}, {max(z_vals):.3f}]")

        # Check for unique keyframes
        kf_ids = [row['kf_id'] for row in data_rows]
        unique_kf_ids = list(set(kf_ids))
        print(f"\nKeyframe IDs: {len(kf_ids)} total, {len(unique_kf_ids)} unique")
        print(f"  Range: {min(unique_kf_ids)} to {max(unique_kf_ids)}")

        return True

    except Exception as e:
        print(f"Error reading GTSAM CSV: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    gtsam_path = "/data/robot/bags/house11/gtsam.csv"
    success = simple_read_gtsam_csv(gtsam_path)
    print(f"\nTest completed: {'Success' if success else 'Failed'}")