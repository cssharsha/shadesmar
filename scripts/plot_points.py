import os
import sys
import glob
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button # For standalone buttons

# --- Configuration ---
# You can set this directly, or modify the script to take it as a command-line argument
DATA_DIR = "./csv_data/"  # <--- CHANGE THIS TO YOUR DIRECTORY or pass as arg

# Column names expected in your CSV files
X_COL = 'x'
Y_COL = 'y'
Z_COL = 'z'

# --- Global State ---
all_keyframe_ids = []
current_keyframe_idx = 0
fig = None  # Matplotlib figure
ax = None   # Matplotlib 3D axes
status_text_obj = None # For displaying status messages
prev_button_widget = None
next_button_widget = None

# --- Helper Functions (can be in this file or imported if you have a utils.py) ---

def discover_keyframe_ids(data_dir):
    ids = set()
    id_pattern = re.compile(r"3d_points_world_(\d+)\.csv") # Assuming discovery based on world files
    try:
        if not os.path.isdir(data_dir):
            print(f"Error: Data directory '{data_dir}' not found or is not a directory.")
            return []
        for filename in os.listdir(data_dir):
            match = id_pattern.match(filename)
            if match:
                ids.add(int(match.group(1)))
    except Exception as e:
        print(f"Error discovering keyframes in '{data_dir}': {e}")
        return []
    return sorted(list(ids))

def load_points_from_csv(filepath, x_col=X_COL, y_col=Y_COL, z_col=Z_COL):
    if not os.path.exists(filepath):
        # print(f"Debug: File not found at {filepath}")
        return None
    try:
        df = pd.read_csv(filepath, sep=r'\s+')
        if not {x_col, y_col, z_col}.issubset(df.columns):
            print(f"Warning: CSV '{os.path.basename(filepath)}' missing coordinate columns.")
            return None
        points = pd.DataFrame({
            'x': pd.to_numeric(df[x_col], errors='coerce'),
            'y': pd.to_numeric(df[y_col], errors='coerce'),
            'z': pd.to_numeric(df[z_col], errors='coerce')
        }).dropna()
        return points
    except Exception as e:
        print(f"Error loading CSV '{os.path.basename(filepath)}': {e}")
        return None

def plot_keyframe_data_on_ax(current_ax, kf_id, prev_pts_df, cur_pts_df, world_pts_df):
    global status_text_obj
    current_ax.clear() # Clear previous plot elements from the axes
    
    all_x, all_y, all_z = [], [], []
    current_status_messages = []

    if prev_pts_df is not None and not prev_pts_df.empty:
        current_ax.scatter(prev_pts_df['x'], prev_pts_df['y'], prev_pts_df['z'], 
                       label=f'Previous KF (rel.)', color='blue', marker='o', s=10, alpha=0.6)
        all_x.extend(prev_pts_df['x']); all_y.extend(prev_pts_df['y']); all_z.extend(prev_pts_df['z'])
    else:
        current_status_messages.append(f"No/empty prev_{kf_id}.csv")
        
    if cur_pts_df is not None and not cur_pts_df.empty:
        current_ax.scatter(cur_pts_df['x'], cur_pts_df['y'], cur_pts_df['z'], 
                       label=f'Current KF (rel.)', color='red', marker='^', s=10, alpha=0.6)
        all_x.extend(cur_pts_df['x']); all_y.extend(cur_pts_df['y']); all_z.extend(cur_pts_df['z'])
    else:
        current_status_messages.append(f"No/empty cur_{kf_id}.csv")

    if world_pts_df is not None and not world_pts_df.empty:
        current_ax.scatter(world_pts_df['x'], world_pts_df['y'], world_pts_df['z'], 
                       label='World Frame', color='green', marker='s', s=10, alpha=0.6)
        all_x.extend(world_pts_df['x']); all_y.extend(world_pts_df['y']); all_z.extend(world_pts_df['z'])
    else:
        current_status_messages.append(f"No/empty world_{kf_id}.csv")

    current_ax.set_xlabel('X')
    current_ax.set_ylabel('Y')
    current_ax.set_zlabel('Z')
    current_ax.legend(loc='upper left', fontsize='small')
    
    if all_x:
        all_x_np, all_y_np, all_z_np = np.array(all_x), np.array(all_y), np.array(all_z)
        max_range = np.array([all_x_np.max()-all_x_np.min(), all_y_np.max()-all_y_np.min(), all_z_np.max()-all_z_np.min()]).max() / 2.0
        if max_range == 0 or np.isnan(max_range): max_range = 1.0
        mid_x = (all_x_np.max()+all_x_np.min())*0.5; mid_y = (all_y_np.max()+all_y_np.min())*0.5; mid_z = (all_z_np.max()+all_z_np.min())*0.5
        current_ax.set_xlim(mid_x - max_range, mid_x + max_range)
        current_ax.set_ylim(mid_y - max_range, mid_y + max_range)
        current_ax.set_zlim(mid_z - max_range, mid_z + max_range)
    else:
        current_ax.set_xlim([-1,1]); current_ax.set_ylim([-1,1]); current_ax.set_zlim([-1,1])
        current_status_messages.append("No points plotted.")
    
    try:
        current_ax.view_init(elev=20., azim=-35)
    except Exception:
        pass # view_init might not work on all backends initially
    
    if status_text_obj:
        status_text_obj.set_text("\n".join(current_status_messages))


def update_visualization(new_idx=None):
    global current_keyframe_idx, all_keyframe_ids, ax, fig, status_text_obj
    global prev_button_widget, next_button_widget

    if new_idx is not None:
        current_keyframe_idx = new_idx
        
    if not all_keyframe_ids:
        fig.suptitle("No keyframes found. Check DATA_DIR.", color='red')
        if status_text_obj: status_text_obj.set_text("Ensure CSV files like world_ID.csv exist.")
        if prev_button_widget: prev_button_widget.ax.set_visible(False) # Hide buttons
        if next_button_widget: next_button_widget.ax.set_visible(False)
        fig.canvas.draw_idle()
        return

    if current_keyframe_idx < 0: current_keyframe_idx = 0
    if current_keyframe_idx >= len(all_keyframe_ids): current_keyframe_idx = len(all_keyframe_ids) - 1
        
    kf_id = all_keyframe_ids[current_keyframe_idx]
    
    title = f'Keyframe ID: {kf_id} ({current_keyframe_idx + 1} of {len(all_keyframe_ids)})'
    fig.suptitle(title) # Set title on the figure

    prev_path = os.path.join(DATA_DIR, f"3d_points_prev_{kf_id}.csv")
    cur_path = os.path.join(DATA_DIR, f"3d_points_cur_{kf_id}.csv")
    world_path = os.path.join(DATA_DIR, f"3d_points_world_{kf_id}.csv")

    prev_pts = load_points_from_csv(prev_path)
    cur_pts = load_points_from_csv(cur_path)
    world_pts = load_points_from_csv(world_path)
    
    plot_keyframe_data_on_ax(ax, kf_id, prev_pts, cur_pts, world_pts)

    # Update button states (enable/disable visually isn't direct with matplotlib buttons,
    # but we can control behavior in callbacks)
    # For actual disabling, one might hide/show or change color,
    # here we just control via the index bounds in callbacks.

    fig.canvas.draw_idle() # Redraw the figure

# --- Button Click Handlers ---
def on_prev_button_clicked(event):
    global current_keyframe_idx
    if current_keyframe_idx > 0:
        update_visualization(current_keyframe_idx - 1)

def on_next_button_clicked(event):
    global current_keyframe_idx, all_keyframe_ids
    if current_keyframe_idx < len(all_keyframe_ids) - 1:
        update_visualization(current_keyframe_idx + 1)

# --- Main Initialization and UI Display ---
def main():
    global all_keyframe_ids, current_keyframe_idx, fig, ax, status_text_obj
    global prev_button_widget, next_button_widget

    # --- Argument Parsing (Optional) ---
    # import argparse
    # parser = argparse.ArgumentParser(description="Visualize 3D keyframe points.")
    # parser.add_argument('--data_dir', type=str, default=DATA_DIR,
    #                     help='Directory containing the CSV point cloud files.')
    # args = parser.parse_args()
    # data_directory = args.data_dir
    data_directory = DATA_DIR # Using global for simplicity here

    print(f"Scanning for keyframe data in: {os.path.abspath(data_directory)}")
    all_keyframe_ids = discover_keyframe_ids(data_directory)
    
    if not all_keyframe_ids:
        print(f"No keyframe IDs found in '{data_directory}'. Exiting.")
        # plt.figure() # Create a dummy figure to show message
        # plt.text(0.5, 0.5, f"No keyframe IDs found in '{data_directory}'.\nCheck console for errors.", 
        #          ha='center', va='center', fontsize=12, color='red')
        # plt.axis('off')
        # plt.show()
        return

    print(f"Found {len(all_keyframe_ids)} keyframe IDs: {all_keyframe_ids}")
    current_keyframe_idx = 0
    
    # Create the main figure and 3D axes
    fig = plt.figure(figsize=(13, 10))
    ax = fig.add_subplot(111, projection='3d')
    plt.subplots_adjust(bottom=0.15, top=0.9) # Adjust subplot to make space for buttons and title

    # Add status text area
    # Position it below the main plot, above buttons
    status_text_obj = fig.text(0.05, 0.06, "", va="bottom", ha="left", fontsize=9, wrap=True)


    # Define button positions [left, bottom, width, height] in figure coordinates (0-1)
    button_height = 0.05
    button_width = 0.1
    button_ypos = 0.01 # At the very bottom
    
    ax_prev = fig.add_axes([0.35 - button_width / 2, button_ypos, button_width, button_height])
    prev_button_widget = Button(ax_prev, 'Back')
    prev_button_widget.on_clicked(on_prev_button_clicked)

    ax_next = fig.add_axes([0.65 - button_width / 2, button_ypos, button_width, button_height])
    next_button_widget = Button(ax_next, 'Next')
    next_button_widget.on_clicked(on_next_button_clicked)
    
    # Initial plot
    update_visualization()
    
    plt.show() # Display the plot and start Matplotlib event loop

if __name__ == "__main__":
    # Example: Override DATA_DIR if you want to run from command line with a different path
    # import sys
    if len(sys.argv) > 1:
        DATA_DIR = sys.argv[1]
        if not os.path.isdir(DATA_DIR):
            print(f"Error: Provided data directory '{DATA_DIR}' does not exist.")
            sys.exit(1)

    main()
