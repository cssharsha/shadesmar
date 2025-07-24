#!/usr/bin/env python3
import argparse
import json
import os
from typing import List, Tuple

import open3d as o3d


def load_trajectory(traj_path: str) -> Tuple[List[Tuple[float, float, float]], List[int]]:
    with open(traj_path, "r") as f:
        data = json.load(f)
    keyframes = data.get("keyframes", [])
    # Sort by id to ensure proper order
    keyframes = sorted(keyframes, key=lambda k: k.get("id", 0))
    positions = [tuple(k["position"]) for k in keyframes]
    ids = [int(k.get("id", i)) for i, k in enumerate(keyframes)]
    return positions, ids


def make_lineset_from_points(points: List[Tuple[float, float, float]], color=(0.2, 1.0, 0.2)):
    if len(points) < 2:
        return None
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(points)
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector([color for _ in lines])
    return ls


def main():
    parser = argparse.ArgumentParser(description="Simple Open3D viewer for COLMAP export")
    parser.add_argument(
        "--dir",
        type=str,
        required=True,
        help="Directory containing points.ply and trajectory.json",
    )
    args = parser.parse_args()

    base = args.dir
    points_path = os.path.join(base, "points.ply")
    kf_points_path = os.path.join(base, "keyframes.ply")
    traj_path = os.path.join(base, "trajectory.json")

    geometries = []

    if os.path.exists(points_path):
        print(f"Loading points from {points_path}")
        pcd = o3d.io.read_point_cloud(points_path)
        geometries.append(pcd)
    else:
        print(f"Warning: {points_path} not found")

    if os.path.exists(kf_points_path):
        print(f"Loading keyframe points from {kf_points_path}")
        kf_pcd = o3d.io.read_point_cloud(kf_points_path)
        geometries.append(kf_pcd)

    if os.path.exists(traj_path):
        print(f"Loading trajectory from {traj_path}")
        positions, _ = load_trajectory(traj_path)
        ls = make_lineset_from_points(positions, color=(0.0, 0.6, 1.0))
        if ls is not None:
            geometries.append(ls)

    # Always show an origin coordinate frame axis
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    geometries.append(axis)

    if not geometries:
        print("No geometries to display. Ensure export produced files in the directory.")
        return

    o3d.visualization.draw_geometries(
        geometries,
        window_name="COLMAP Export Viewer",
        width=1280,
        height=800,
    )


if __name__ == "__main__":
    main()
