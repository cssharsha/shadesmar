#!/usr/bin/env python3

import csv
import math

def read_gtsam_data(gtsam_csv_path):
    """Read GTSAM CSV data"""
    data_rows = []

    with open(gtsam_csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # Skip header

        for row in reader:
            if len(row) >= 7:
                data_rows.append({
                    'x': float(row[0]),
                    'y': float(row[1]),
                    'z': float(row[2]),
                    'r': float(row[3]),
                    'p': float(row[4]),
                    'yaw': float(row[5]),
                    'kf_id': int(row[6])
                })

    # Remove duplicates by kf_id (keep last)
    unique_data = {}
    for row in data_rows:
        unique_data[row['kf_id']] = row

    # Sort by kf_id
    sorted_data = sorted(unique_data.values(), key=lambda x: x['kf_id'])
    return sorted_data

def create_text_plot(data, width=80, height=20):
    """Create a simple text-based 2D plot of the trajectory"""
    if not data:
        return "No data to plot"

    # Get data ranges
    x_vals = [p['x'] for p in data]
    y_vals = [p['y'] for p in data]

    x_min, x_max = min(x_vals), max(x_vals)
    y_min, y_max = min(y_vals), max(y_vals)

    # Create grid
    grid = [[' ' for _ in range(width)] for _ in range(height)]

    # Plot points
    for i, point in enumerate(data):
        # Normalize coordinates to grid
        if x_max != x_min:
            x_norm = (point['x'] - x_min) / (x_max - x_min)
        else:
            x_norm = 0.5

        if y_max != y_min:
            y_norm = (point['y'] - y_min) / (y_max - y_min)
        else:
            y_norm = 0.5

        # Convert to grid coordinates (flip Y for display)
        grid_x = int(x_norm * (width - 1))
        grid_y = int((1 - y_norm) * (height - 1))  # Flip Y axis

        # Mark the point
        if i == 0:
            grid[grid_y][grid_x] = 'S'  # Start
        elif i == len(data) - 1:
            grid[grid_y][grid_x] = 'E'  # End
        else:
            grid[grid_y][grid_x] = '*'  # Trajectory point

    # Convert grid to string
    result = []
    result.append(f"GTSAM Trajectory (X-Y plane)")
    result.append(f"X range: [{x_min:.3f}, {x_max:.3f}]")
    result.append(f"Y range: [{y_min:.3f}, {y_max:.3f}]")
    result.append(f"Points: {len(data)} keyframes")
    result.append("Legend: S=Start, E=End, *=Trajectory")
    result.append("-" * width)

    for row in grid:
        result.append(''.join(row))

    result.append("-" * width)

    return '\n'.join(result)

def print_trajectory_summary(data):
    """Print detailed trajectory information"""
    if not data:
        print("No trajectory data available")
        return

    print(f"\n=== GTSAM TRAJECTORY SUMMARY ===")
    print(f"Total keyframes: {len(data)}")
    print(f"Keyframe ID range: {data[0]['kf_id']} to {data[-1]['kf_id']}")

    # Position statistics
    x_vals = [p['x'] for p in data]
    y_vals = [p['y'] for p in data]
    z_vals = [p['z'] for p in data]

    print(f"\nPosition ranges:")
    print(f"  X: [{min(x_vals):.3f}, {max(x_vals):.3f}] meters")
    print(f"  Y: [{min(y_vals):.3f}, {max(y_vals):.3f}] meters")
    print(f"  Z: [{min(z_vals):.3f}, {max(z_vals):.3f}] meters")

    # Calculate trajectory length
    total_distance = 0.0
    for i in range(1, len(data)):
        prev = data[i-1]
        curr = data[i]
        dx = curr['x'] - prev['x']
        dy = curr['y'] - prev['y']
        dz = curr['z'] - prev['z']
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        total_distance += distance

    print(f"\nTrajectory statistics:")
    print(f"  Total path length: {total_distance:.3f} meters")
    if len(data) > 1:
        start = data[0]
        end = data[-1]
        straight_distance = math.sqrt((end['x'] - start['x'])**2 +
                                    (end['y'] - start['y'])**2 +
                                    (end['z'] - start['z'])**2)
        print(f"  Straight-line distance: {straight_distance:.3f} meters")
        print(f"  Path efficiency: {straight_distance/total_distance:.1%}")

    # Orientation statistics
    r_vals = [p['r'] for p in data]
    p_vals = [p['p'] for p in data]
    yaw_vals = [p['yaw'] for p in data]

    print(f"\nOrientation ranges (degrees):")
    print(f"  Roll: [{min(r_vals):.1f}, {max(r_vals):.1f}]")
    print(f"  Pitch: [{min(p_vals):.1f}, {max(p_vals):.1f}]")
    print(f"  Yaw: [{min(yaw_vals):.1f}, {max(yaw_vals):.1f}]")

def main():
    gtsam_path = "/data/robot/bags/house11/gtsam.csv"

    print("Loading GTSAM trajectory data...")
    data = read_gtsam_data(gtsam_path)

    if not data:
        print("Failed to load GTSAM data")
        return

    print(f"Loaded {len(data)} unique trajectory points")

    # Print summary statistics
    print_trajectory_summary(data)

    # Create text-based visualization
    print("\n" + "="*80)
    plot = create_text_plot(data)
    print(plot)
    print("="*80)

    # Show first and last few points
    print(f"\nFirst 3 trajectory points:")
    for i, point in enumerate(data[:3]):
        print(f"  {i+1}: kf_id={point['kf_id']}, pos=({point['x']:.3f}, {point['y']:.3f}, {point['z']:.3f})")

    print(f"\nLast 3 trajectory points:")
    for i, point in enumerate(data[-3:]):
        idx = len(data) - 3 + i + 1
        print(f"  {idx}: kf_id={point['kf_id']}, pos=({point['x']:.3f}, {point['y']:.3f}, {point['z']:.3f})")

if __name__ == "__main__":
    main()