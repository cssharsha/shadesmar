#!/usr/bin/env python3

import csv

def debug_gtsam_csv_simple(gtsam_csv_path):
    """Simple debug of GTSAM CSV file without pandas"""
    print(f"Loading GTSAM CSV from: {gtsam_csv_path}")

    try:
        with open(gtsam_csv_path, 'r') as f:
            # Read first few lines
            lines = f.readlines()
            print(f"File has {len(lines)} lines")

            if lines:
                header = lines[0].strip()
                print(f"Header: {header}")
                print(f"Header columns: {header.split(',')}")

                print("\nFirst 5 data lines:")
                for i, line in enumerate(lines[1:6]):
                    print(f"Line {i+2}: {line.strip()}")

        # Parse with csv module
        data_rows = []
        with open(gtsam_csv_path, 'r') as f:
            reader = csv.DictReader(f)
            print(f"\nCSV columns detected: {reader.fieldnames}")

            for i, row in enumerate(reader):
                if i < 5:  # First 5 rows
                    print(f"Row {i+1}: {row}")
                data_rows.append(row)
                if i >= 10:  # Just read first 10 rows for debugging
                    break

        print(f"\nRead {len(data_rows)} rows successfully")

        # Check if we have expected columns
        expected_cols = ['x', 'y', 'z', 'r', 'p', 'y', 'kf_id']
        if data_rows:
            actual_cols = list(data_rows[0].keys())
            print(f"Expected columns: {expected_cols}")
            print(f"Actual columns: {actual_cols}")

            missing = [col for col in expected_cols if col not in actual_cols]
            if missing:
                print(f"Missing columns: {missing}")
            else:
                print("All expected columns found!")

                # Show data ranges
                for col in ['x', 'y', 'z', 'kf_id']:
                    values = [float(row[col]) for row in data_rows if row[col]]
                    print(f"{col}: min={min(values):.3f}, max={max(values):.3f}")

        return True

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    gtsam_path = "/data/robot/bags/house11/gtsam.csv"
    success = debug_gtsam_csv_simple(gtsam_path)
    print(f"\nDebug completed: {'Success' if success else 'Failed'}")