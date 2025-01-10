import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse

def plot_odometry_path(filename):
    # Read the CSV file
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"Error: Could not find file: {filename}")
        return

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # Plot top-down view (X-Y plane)
    ax1.plot(df['x'], df['y'], 'b-', label='Path')
    ax1.scatter(df['x'].iloc[0], df['y'].iloc[0], c='g', marker='o', s=100, label='Start')
    ax1.scatter(df['x'].iloc[-1], df['y'].iloc[-1], c='r', marker='o', s=100, label='End')
    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    ax1.set_title('Top-down View (X-Y)')
    ax1.grid(True)
    ax1.axis('equal')  # Equal aspect ratio
    ax1.legend()

    # Plot side view (X-Z plane)
    ax2.plot(df['x'], df['z'], 'b-', label='Path')
    ax2.scatter(df['x'].iloc[0], df['z'].iloc[0], c='g', marker='o', s=100, label='Start')
    ax2.scatter(df['x'].iloc[-1], df['z'].iloc[-1], c='r', marker='o', s=100, label='End')
    ax2.set_xlabel('X (meters)')
    ax2.set_ylabel('Z (meters)')
    ax2.set_title('Side View (X-Z)')
    ax2.grid(True)
    ax2.axis('equal')  # Equal aspect ratio
    ax2.legend()

    # Add overall title
    plt.suptitle(f'Odometry Path Visualization: {filename}')

    # Adjust layout to prevent overlap
    plt.tight_layout()

    # Show plot
    plt.show()

    # Print some basic statistics
    total_distance = np.sum(np.sqrt(
        np.diff(df['x'])**2 +
        np.diff(df['y'])**2 +
        np.diff(df['z'])**2
    ))

    print(f"\nPath Statistics:")
    print(f"Total distance traveled: {total_distance:.2f} meters")
    print(f"Duration: {df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]:.2f} seconds")
    print(f"Number of poses: {len(df)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot odometry path from CSV file')
    parser.add_argument('filename', type=str, help='Path to the CSV file containing odometry data')
    args = parser.parse_args()

    plot_odometry_path(args.filename)
