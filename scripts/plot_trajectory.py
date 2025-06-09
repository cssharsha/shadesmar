#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import sys
from mpl_toolkits.mplot3d import Axes3D

def filter_data(df, args):
    """Filter data based on command line arguments"""
    filtered_df = df.copy()

    # Apply trajectory type filters
    trajectory_filters = []

    if args.keyframes_only:
        trajectory_filters.append(df['type'] == 'keyframe')
    elif args.transforms_only:
        trajectory_filters.append(df['type'] == 'transform')
    elif args.tf_only:
        trajectory_filters.append((df['type'] == 'transform') & (df['method'] == 'TF'))
    elif args.essential_only:
        trajectory_filters.append((df['type'] == 'transform') & (df['method'] == 'Essential'))
    elif args.odometry_only:
        trajectory_filters.append(df['type'] == 'odometry')
    elif args.gtsam_only:
        trajectory_filters.append(df['type'] == 'gtsam')
    else:
        # Default: show what's enabled
        include_conditions = []
        if not args.hide_keyframes:
            include_conditions.append(df['type'] == 'keyframe')
        if not args.hide_transforms:
            if args.hide_tf and not args.hide_essential:
                include_conditions.append((df['type'] == 'transform') & (df['method'] == 'Essential'))
            elif args.hide_essential and not args.hide_tf:
                include_conditions.append((df['type'] == 'transform') & (df['method'] == 'TF'))
            elif not args.hide_tf and not args.hide_essential:
                include_conditions.append(df['type'] == 'transform')
        if not args.hide_odometry:
            include_conditions.append(df['type'] == 'odometry')
        if not args.hide_gtsam:
            include_conditions.append(df['type'] == 'gtsam')

        if include_conditions:
            trajectory_filters = include_conditions
        else:
            # If everything is hidden, show everything
            trajectory_filters = [True] * len(df)

    # Combine filters
    if trajectory_filters:
        if len(trajectory_filters) == 1:
            final_filter = trajectory_filters[0]
        else:
            final_filter = trajectory_filters[0]
            for f in trajectory_filters[1:]:
                final_filter = final_filter | f
        filtered_df = df[final_filter]

    return filtered_df

def plot_2d_trajectory(df, args):
    """Plot 2D trajectory (X-Y plane)"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Filter data
    df = filter_data(df, args)

    # Separate data types
    keyframes = df[df['type'] == 'keyframe']
    transforms = df[df['type'] == 'transform']
    tf_transforms = transforms[transforms['method'] == 'TF']
    essential_transforms = transforms[transforms['method'] == 'Essential']
    odometry = df[df['type'] == 'odometry']
    gtsam = df[df['type'] == 'gtsam']

    # Plot X-Y trajectory
    if not keyframes.empty and not args.hide_keyframes:
        ax1.plot(keyframes['x'], keyframes['y'], 'b-o', linewidth=2, markersize=4,
                label=f'Keyframes ({len(keyframes)} points)', alpha=0.8)

    if not tf_transforms.empty and not args.hide_tf:
        ax1.plot(tf_transforms['x'], tf_transforms['y'], 'r-s', linewidth=1.5, markersize=3,
                label=f'TF Transforms ({len(tf_transforms)} points)', alpha=0.7)

    if not essential_transforms.empty and not args.hide_essential:
        ax1.plot(essential_transforms['x'], essential_transforms['y'], 'g-^', linewidth=1.5, markersize=3,
                label=f'Essential Transforms ({len(essential_transforms)} points)', alpha=0.7)

    if not odometry.empty and not args.hide_odometry:
        ax1.plot(odometry['x'], odometry['y'], 'm-d', linewidth=1.5, markersize=3,
                label=f'Odometry Factors ({len(odometry)} points)', alpha=0.7)

    if not gtsam.empty and not args.hide_gtsam:
        ax1.plot(gtsam['x'], gtsam['y'], 'c-*', linewidth=2, markersize=5,
                label=f'GTSAM ({len(gtsam)} points)', alpha=0.8)

    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    ax1.set_title('2D Trajectory Comparison (X-Y Plane)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # Plot X-Z trajectory
    if not keyframes.empty and not args.hide_keyframes:
        ax2.plot(keyframes['x'], keyframes['z'], 'b-o', linewidth=2, markersize=4,
                label=f'Keyframes ({len(keyframes)} points)', alpha=0.8)

    if not tf_transforms.empty and not args.hide_tf:
        ax2.plot(tf_transforms['x'], tf_transforms['z'], 'r-s', linewidth=1.5, markersize=3,
                label=f'TF Transforms ({len(tf_transforms)} points)', alpha=0.7)

    if not essential_transforms.empty and not args.hide_essential:
        ax2.plot(essential_transforms['x'], essential_transforms['z'], 'g-^', linewidth=1.5, markersize=3,
                label=f'Essential Transforms ({len(essential_transforms)} points)', alpha=0.7)

    if not odometry.empty and not args.hide_odometry:
        ax2.plot(odometry['x'], odometry['z'], 'm-d', linewidth=1.5, markersize=3,
                label=f'Odometry Factors ({len(odometry)} points)', alpha=0.7)

    if not gtsam.empty and not args.hide_gtsam:
        ax2.plot(gtsam['x'], gtsam['z'], 'c-*', linewidth=2, markersize=5,
                label=f'GTSAM ({len(gtsam)} points)', alpha=0.8)

    ax2.set_xlabel('X (meters)')
    ax2.set_ylabel('Z (meters)')
    ax2.set_title('2D Trajectory Comparison (X-Z Plane)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')

    plt.tight_layout()
    return fig

def plot_3d_trajectory(df, args):
    """Plot 3D trajectory"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Filter data
    df = filter_data(df, args)

    # Separate data types
    keyframes = df[df['type'] == 'keyframe']
    transforms = df[df['type'] == 'transform']
    tf_transforms = transforms[transforms['method'] == 'TF']
    essential_transforms = transforms[transforms['method'] == 'Essential']
    odometry = df[df['type'] == 'odometry']
    gtsam = df[df['type'] == 'gtsam']

    # Plot trajectories
    if not keyframes.empty and not args.hide_keyframes:
        ax.plot(keyframes['x'], keyframes['y'], keyframes['z'], 'b-o', linewidth=2, markersize=4,
               label=f'Keyframes ({len(keyframes)} points)', alpha=0.8)

    if not tf_transforms.empty and not args.hide_tf:
        ax.plot(tf_transforms['x'], tf_transforms['y'], tf_transforms['z'], 'r-s', linewidth=1.5, markersize=3,
               label=f'TF Transforms ({len(tf_transforms)} points)', alpha=0.7)

    if not essential_transforms.empty and not args.hide_essential:
        ax.plot(essential_transforms['x'], essential_transforms['y'], essential_transforms['z'], 'g-^', linewidth=1.5, markersize=3,
               label=f'Essential Transforms ({len(essential_transforms)} points)', alpha=0.7)

    if not odometry.empty and not args.hide_odometry:
        ax.plot(odometry['x'], odometry['y'], odometry['z'], 'm-d', linewidth=1.5, markersize=3,
               label=f'Odometry Factors ({len(odometry)} points)', alpha=0.7)

    if not gtsam.empty and not args.hide_gtsam:
        ax.plot(gtsam['x'], gtsam['y'], gtsam['z'], 'c-*', linewidth=2, markersize=5,
               label=f'GTSAM ({len(gtsam)} points)', alpha=0.8)

    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title('3D Trajectory Comparison')
    ax.legend()

    return fig

def plot_orientation_comparison(df, args):
    """Plot orientation (RPY) over time/sequence"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # Filter data
    df = filter_data(df, args)

    keyframes = df[df['type'] == 'keyframe'].sort_values('id')
    transforms = df[df['type'] == 'transform'].sort_values('id')
    odometry = df[df['type'] == 'odometry'].sort_values('id')
    gtsam = df[df['type'] == 'gtsam'].sort_values('id')

    orientations = ['roll', 'pitch', 'yaw']
    colors = ['red', 'green', 'blue']

    for i, (orient, color) in enumerate(zip(orientations, colors)):
        if not keyframes.empty and not args.hide_keyframes:
            axes[i].plot(keyframes.index, np.degrees(keyframes[orient]),
                        'o-', color=color, linewidth=2, markersize=4,
                        label=f'Keyframes {orient.title()}', alpha=0.8)

        if not transforms.empty and not args.hide_transforms:
            axes[i].plot(transforms.index, np.degrees(transforms[orient]),
                        's-', color=color, linewidth=1.5, markersize=3, alpha=0.7,
                        label=f'Transforms {orient.title()}')

        if not odometry.empty and not args.hide_odometry:
            axes[i].plot(odometry.index, np.degrees(odometry[orient]),
                        'd-', color=color, linewidth=1.5, markersize=3, alpha=0.7,
                        label=f'Odometry {orient.title()}')

        if not gtsam.empty and not args.hide_gtsam:
            axes[i].plot(gtsam.index, np.degrees(gtsam[orient]),
                        '*-', color=color, linewidth=2, markersize=5, alpha=0.8,
                        label=f'GTSAM {orient.title()}')

        axes[i].set_ylabel(f'{orient.title()} (degrees)')
        axes[i].set_title(f'{orient.title()} Comparison')
        axes[i].legend()
        axes[i].grid(True, alpha=0.3)

    axes[-1].set_xlabel('Sequence Index')
    plt.tight_layout()
    return fig

def plot_transform_statistics(df, args):
    """Plot transform step statistics"""
    # Only show transform statistics if transforms are enabled
    df = filter_data(df, args)
    transforms = df[df['type'] == 'transform']

    if transforms.empty:
        print("No transform data available for statistics")
        return None

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))

    # Calculate step sizes
    transforms = transforms.sort_values('id')
    if len(transforms) > 1:
        dx = np.diff(transforms['x'])
        dy = np.diff(transforms['y'])
        dz = np.diff(transforms['z'])
        step_sizes = np.sqrt(dx**2 + dy**2 + dz**2)

        # Step size histogram
        axes[0, 0].hist(step_sizes, bins=20, alpha=0.7, edgecolor='black')
        axes[0, 0].set_xlabel('Step Size (meters)')
        axes[0, 0].set_ylabel('Frequency')
        axes[0, 0].set_title('Transform Step Size Distribution')
        axes[0, 0].grid(True, alpha=0.3)

        # Step size over sequence
        axes[0, 1].plot(step_sizes, 'b-o', markersize=3)
        axes[0, 1].set_xlabel('Transform Index')
        axes[0, 1].set_ylabel('Step Size (meters)')
        axes[0, 1].set_title('Step Size Over Sequence')
        axes[0, 1].grid(True, alpha=0.3)

    # Method distribution
    method_counts = transforms['method'].value_counts()
    if not method_counts.empty:
        axes[1, 0].pie(method_counts.values, labels=method_counts.index, autopct='%1.1f%%')
        axes[1, 0].set_title('Transform Method Distribution')

    # Cumulative distance
    if len(transforms) > 1:
        cumulative_distance = np.cumsum(np.concatenate([[0], step_sizes]))
        axes[1, 1].plot(cumulative_distance, 'g-', linewidth=2)
        axes[1, 1].set_xlabel('Transform Index')
        axes[1, 1].set_ylabel('Cumulative Distance (meters)')
        axes[1, 1].set_title('Cumulative Distance Traveled')
        axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    return fig

def print_summary_statistics(df, args):
    """Print summary statistics"""
    print("\n" + "="*60)
    print("TRAJECTORY COMPARISON SUMMARY")
    print("="*60)

    # Show filter information
    if args.keyframes_only:
        print("FILTER: Showing keyframes only")
    elif args.transforms_only:
        print("FILTER: Showing transforms only")
    elif args.tf_only:
        print("FILTER: Showing TF transforms only")
    elif args.essential_only:
        print("FILTER: Showing Essential transforms only")
    elif args.odometry_only:
        print("FILTER: Showing odometry factors only")
    elif args.gtsam_only:
        print("FILTER: Showing GTSAM trajectory only")
    else:
        filters = []
        if args.hide_keyframes:
            filters.append("hiding keyframes")
        if args.hide_transforms:
            filters.append("hiding transforms")
        if args.hide_tf:
            filters.append("hiding TF")
        if args.hide_essential:
            filters.append("hiding Essential")
        if args.hide_odometry:
            filters.append("hiding odometry")
        if args.hide_gtsam:
            filters.append("hiding GTSAM")
        if filters:
            print(f"FILTER: {', '.join(filters)}")

    # Filter data for statistics
    df = filter_data(df, args)

    keyframes = df[df['type'] == 'keyframe']
    transforms = df[df['type'] == 'transform']
    odometry = df[df['type'] == 'odometry']
    gtsam = df[df['type'] == 'gtsam']

    if not keyframes.empty:
        kf_start = keyframes.iloc[0][['x', 'y', 'z']].values
        kf_end = keyframes.iloc[-1][['x', 'y', 'z']].values
        kf_distance = np.linalg.norm(kf_end - kf_start)

        print(f"\nKEYFRAME TRAJECTORY:")
        print(f"  Number of keyframes: {len(keyframes)}")
        print(f"  Start position: [{kf_start[0]:.3f}, {kf_start[1]:.3f}, {kf_start[2]:.3f}]")
        print(f"  End position: [{kf_end[0]:.3f}, {kf_end[1]:.3f}, {kf_end[2]:.3f}]")
        print(f"  Net displacement: {kf_distance:.3f} meters")

    if not transforms.empty:
        tf_end = transforms.iloc[-1][['x', 'y', 'z']].values
        tf_distance = np.linalg.norm(tf_end)

        method_counts = transforms['method'].value_counts()

        print(f"\nTRANSFORM TRAJECTORY:")
        print(f"  Number of transforms: {len(transforms)}")
        for method, count in method_counts.items():
            print(f"  {method} transforms: {count}")
        print(f"  Final position: [{tf_end[0]:.3f}, {tf_end[1]:.3f}, {tf_end[2]:.3f}]")
        print(f"  Net displacement: {tf_distance:.3f} meters")

        if not keyframes.empty:
            position_error = np.linalg.norm(kf_end - tf_end)
            print(f"\nTRAJECTORY COMPARISON:")
            print(f"  Position error: {position_error:.3f} meters")
            print(f"  Relative error: {100 * position_error / max(kf_distance, 0.001):.2f}%")

    if not odometry.empty:
        odom_start = odometry.iloc[0][['x', 'y', 'z']].values
        odom_end = odometry.iloc[-1][['x', 'y', 'z']].values
        odom_distance = np.linalg.norm(odom_end - odom_start)

        print(f"\nODOMETRY TRAJECTORY:")
        print(f"  Number of odometry points: {len(odometry)}")
        print(f"  Start position: [{odom_start[0]:.3f}, {odom_start[1]:.3f}, {odom_start[2]:.3f}]")
        print(f"  End position: [{odom_end[0]:.3f}, {odom_end[1]:.3f}, {odom_end[2]:.3f}]")
        print(f"  Net displacement: {odom_distance:.3f} meters")

        if not keyframes.empty:
            position_error = np.linalg.norm(kf_end - odom_end)
            print(f"\nODOMETRY vs KEYFRAMES COMPARISON:")
            print(f"  Position error: {position_error:.3f} meters")
            print(f"  Relative error: {100 * position_error / max(kf_distance, 0.001):.2f}%")

    if not gtsam.empty:
        gtsam_start = gtsam.iloc[0][['x', 'y', 'z']].values
        gtsam_end = gtsam.iloc[-1][['x', 'y', 'z']].values
        gtsam_distance = np.linalg.norm(gtsam_end - gtsam_start)

        print(f"\nGTSAM TRAJECTORY:")
        print(f"  Number of GTSAM points: {len(gtsam)}")
        print(f"  Start position: [{gtsam_start[0]:.3f}, {gtsam_start[1]:.3f}, {gtsam_start[2]:.3f}]")
        print(f"  End position: [{gtsam_end[0]:.3f}, {gtsam_end[1]:.3f}, {gtsam_end[2]:.3f}]")
        print(f"  Net displacement: {gtsam_distance:.3f} meters")

        if not keyframes.empty:
            position_error = np.linalg.norm(kf_end - gtsam_end)
            print(f"\nGTSAM vs KEYFRAMES COMPARISON:")
            print(f"  Position error: {position_error:.3f} meters")
            print(f"  Relative error: {100 * position_error / max(kf_distance, 0.001):.2f}%")

def read_gtsam_csv(gtsam_csv_path):
    """Read GTSAM CSV file and convert to standard format"""
    try:
        # Read CSV manually to handle the 'y' column ambiguity
        import csv
        data_rows = []

        with open(gtsam_csv_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # Read header
            print(f"GTSAM CSV header: {header}")

            # Expected header: x,y,z,r,p,y,kf_id,frame_id
            # But CSV parsing treats the second 'y' as the column name, confusing position y with yaw y

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
            return None

        # Convert to DataFrame format
        converted_df = pd.DataFrame()
        converted_df['type'] = 'gtsam'
        converted_df['id'] = [row['kf_id'] for row in data_rows]
        converted_df['x'] = [row['x'] for row in data_rows]
        converted_df['y'] = [row['y_pos'] for row in data_rows]  # Use position Y, not yaw Y
        converted_df['z'] = [row['z'] for row in data_rows]

        # Convert angles from degrees to radians if they appear to be in degrees
        r_values = [row['r'] for row in data_rows]
        p_values = [row['p'] for row in data_rows]
        y_values = [row['y_angle'] for row in data_rows]

        max_angle = max(max(abs(v) for v in r_values),
                       max(abs(v) for v in p_values),
                       max(abs(v) for v in y_values))
        print(f"Max absolute angle value: {max_angle}")

        if max_angle > 10:  # Likely in degrees
            converted_df['roll'] = np.radians(r_values)
            converted_df['pitch'] = np.radians(p_values)
            converted_df['yaw'] = np.radians(y_values)
            print("Converting angles from degrees to radians")
        else:
            converted_df['roll'] = r_values
            converted_df['pitch'] = p_values
            converted_df['yaw'] = y_values
            print("Angles appear to be in radians already")

        converted_df['timestamp'] = 0  # GTSAM CSV doesn't have timestamps
        converted_df['method'] = 'gtsam_optimized'

        # Remove duplicate entries (same kf_id)
        initial_count = len(converted_df)
        converted_df = converted_df.drop_duplicates(subset=['id'], keep='last')
        final_count = len(converted_df)
        print(f"After removing duplicates: {final_count} unique keyframes (removed {initial_count - final_count})")

        # Show data ranges for debugging
        print(f"Position ranges: X=[{converted_df['x'].min():.3f}, {converted_df['x'].max():.3f}], "
              f"Y=[{converted_df['y'].min():.3f}, {converted_df['y'].max():.3f}], "
              f"Z=[{converted_df['z'].min():.3f}, {converted_df['z'].max():.3f}]")

        return converted_df

    except Exception as e:
        print(f"Error reading GTSAM CSV: {e}")
        import traceback
        traceback.print_exc()
        return None

def combine_data_sources(trajectory_csv=None, gtsam_csv=None):
    """Combine trajectory comparison CSV and GTSAM CSV data"""
    combined_df = pd.DataFrame()

    # Read trajectory comparison CSV if provided
    if trajectory_csv:
        try:
            traj_df = pd.read_csv(trajectory_csv)
            combined_df = pd.concat([combined_df, traj_df], ignore_index=True)
            print(f"Loaded {len(traj_df)} entries from trajectory CSV")
        except Exception as e:
            print(f"Error reading trajectory CSV: {e}")

    # Read GTSAM CSV if provided
    if gtsam_csv:
        gtsam_df = None

        # First try to read as standard CSV format (exported format)
        try:
            test_df = pd.read_csv(gtsam_csv)
            expected_standard_columns = ['type', 'id', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'timestamp', 'method']

            if all(col in test_df.columns for col in expected_standard_columns):
                # This is already in standard format
                gtsam_df = test_df
                print(f"Loaded {len(gtsam_df)} entries from standard format GTSAM CSV")
            else:
                raise ValueError("Not in standard format, will try original GTSAM format")

        except Exception as e:
            print(f"Not in standard format ({e}), trying original GTSAM CSV format...")

            # Fall back to original GTSAM CSV parsing
            gtsam_df = read_gtsam_csv(gtsam_csv)
            if gtsam_df is not None:
                print(f"Added {len(gtsam_df)} GTSAM entries using original format parser")

        # Add GTSAM data to combined dataframe
        if gtsam_df is not None:
            combined_df = pd.concat([combined_df, gtsam_df], ignore_index=True)

    return combined_df

def main():
    parser = argparse.ArgumentParser(description='Plot trajectory comparison data')
    parser.add_argument('csv_file', nargs='?', help='Path to trajectory comparison CSV file')
    parser.add_argument('--gtsam-csv', help='Path to GTSAM CSV file (original format or exported standard format)')
    parser.add_argument('--output', '-o', help='Output directory for plots')
    parser.add_argument('--show', action='store_true', help='Show plots interactively')
    parser.add_argument('--format', default='png', choices=['png', 'pdf', 'svg'],
                       help='Output format for saved plots')

    # Trajectory filtering options
    filter_group = parser.add_argument_group('Trajectory Filters')

    # Exclusive filters (show only one type)
    exclusive_group = filter_group.add_mutually_exclusive_group()
    exclusive_group.add_argument('--keyframes-only', action='store_true',
                               help='Show only keyframe trajectory')
    exclusive_group.add_argument('--transforms-only', action='store_true',
                               help='Show only transform trajectory')
    exclusive_group.add_argument('--tf-only', action='store_true',
                               help='Show only TF transform trajectory')
    exclusive_group.add_argument('--essential-only', action='store_true',
                               help='Show only Essential matrix transform trajectory')
    exclusive_group.add_argument('--odometry-only', action='store_true',
                               help='Show only odometry factors trajectory')
    exclusive_group.add_argument('--gtsam-only', action='store_true',
                               help='Show only GTSAM trajectory')

    # Inclusive filters (hide specific types)
    filter_group.add_argument('--hide-keyframes', action='store_true',
                            help='Hide keyframe trajectory')
    filter_group.add_argument('--hide-transforms', action='store_true',
                            help='Hide all transform trajectories')
    filter_group.add_argument('--hide-tf', action='store_true',
                            help='Hide TF transform trajectory')
    filter_group.add_argument('--hide-essential', action='store_true',
                            help='Hide Essential matrix transform trajectory')
    filter_group.add_argument('--hide-odometry', action='store_true',
                            help='Hide odometry factors trajectory')
    filter_group.add_argument('--hide-gtsam', action='store_true',
                            help='Hide GTSAM trajectory')

    args = parser.parse_args()

    # Check if at least one data source is provided
    if not args.csv_file and not args.gtsam_csv:
        print("Error: Must provide either a trajectory CSV file or --gtsam-csv option")
        parser.print_help()
        sys.exit(1)

    try:
        # Combine data from multiple sources
        df = combine_data_sources(args.csv_file, args.gtsam_csv)

        if df.empty:
            print("Error: No valid data loaded from any source")
            sys.exit(1)

        print(f"Total loaded data points: {len(df)}")

        # Print data type breakdown
        if 'type' in df.columns:
            type_counts = df['type'].value_counts()
            print("Data type breakdown:")
            for data_type, count in type_counts.items():
                print(f"  {data_type}: {count} points")

        # Print summary
        print_summary_statistics(df, args)

        # Generate plots
        plots = []

        # 2D trajectory plot
        fig_2d = plot_2d_trajectory(df, args)
        plots.append(('2d_trajectory', fig_2d))

        # 3D trajectory plot
        fig_3d = plot_3d_trajectory(df, args)
        plots.append(('3d_trajectory', fig_3d))

        # Orientation comparison
        fig_orient = plot_orientation_comparison(df, args)
        plots.append(('orientation_comparison', fig_orient))

        # Transform statistics (only if transforms are shown)
        if not args.keyframes_only and not args.odometry_only and not args.gtsam_only:
            fig_stats = plot_transform_statistics(df, args)
            if fig_stats:
                plots.append(('transform_statistics', fig_stats))

        # Save or show plots
        if args.output:
            import os
            os.makedirs(args.output, exist_ok=True)
            for name, fig in plots:
                filepath = os.path.join(args.output, f'{name}.{args.format}')
                fig.savefig(filepath, dpi=300, bbox_inches='tight')
                print(f"Saved plot: {filepath}")

        if args.show:
            plt.show()
        elif not args.output:
            print("Use --show to display plots or --output to save them")

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()