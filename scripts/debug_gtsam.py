#!/usr/bin/env python3

import pandas as pd
import numpy as np

def debug_gtsam_csv(gtsam_csv_path):
    """Debug GTSAM CSV file loading"""
    print(f"Loading GTSAM CSV from: {gtsam_csv_path}")

    try:
        # Read raw CSV
        gtsam_df = pd.read_csv(gtsam_csv_path)
        print(f"Raw CSV loaded: {len(gtsam_df)} rows")
        print("Columns:", list(gtsam_df.columns))
        print("\nFirst 5 rows:")
        print(gtsam_df.head())

        # Check for required columns
        required_columns = ['x', 'y', 'z', 'r', 'p', 'y', 'kf_id']
        missing_cols = [col for col in required_columns if col not in gtsam_df.columns]
        if missing_cols:
            print(f"Missing columns: {missing_cols}")
            return None

        print("\nData ranges:")
        for col in ['x', 'y', 'z', 'r', 'p', 'y', 'kf_id']:
            if col in gtsam_df.columns:
                print(f"{col}: min={gtsam_df[col].min():.3f}, max={gtsam_df[col].max():.3f}")

        # Convert to standard format
        converted_df = pd.DataFrame()
        converted_df['type'] = 'gtsam'
        converted_df['id'] = gtsam_df['kf_id']
        converted_df['x'] = gtsam_df['x']
        converted_df['y'] = gtsam_df['y']
        converted_df['z'] = gtsam_df['z']

        # Check if angles are in degrees
        max_angle = max(gtsam_df['r'].abs().max(), gtsam_df['p'].abs().max(), gtsam_df['y'].abs().max())
        print(f"\nMax absolute angle value: {max_angle}")

        if max_angle > 10:  # Likely in degrees
            converted_df['roll'] = np.radians(gtsam_df['r'])
            converted_df['pitch'] = np.radians(gtsam_df['p'])
            converted_df['yaw'] = np.radians(gtsam_df['y'])
            print("Converting angles from degrees to radians")
        else:
            converted_df['roll'] = gtsam_df['r']
            converted_df['pitch'] = gtsam_df['p']
            converted_df['yaw'] = gtsam_df['y']
            print("Angles appear to be in radians already")

        converted_df['timestamp'] = 0
        converted_df['method'] = 'gtsam_optimized'

        print(f"\nConverted DataFrame: {len(converted_df)} rows")
        print("Converted columns:", list(converted_df.columns))

        # Remove duplicates
        initial_count = len(converted_df)
        converted_df = converted_df.drop_duplicates(subset=['id'], keep='last')
        final_count = len(converted_df)
        print(f"After removing duplicates: {final_count} unique keyframes (removed {initial_count - final_count})")

        print("\nFirst 5 converted rows:")
        print(converted_df.head())

        print("\nFinal data ranges:")
        for col in ['x', 'y', 'z']:
            print(f"{col}: min={converted_df[col].min():.3f}, max={converted_df[col].max():.3f}")

        return converted_df

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    gtsam_path = "/data/robot/bags/house11/gtsam.csv"
    result = debug_gtsam_csv(gtsam_path)
    if result is not None:
        print(f"\nSuccess! Loaded {len(result)} data points")
    else:
        print("\nFailed to load data")