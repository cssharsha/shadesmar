#!/usr/bin/env python3
"""
Cleanup COLMAP text format data to remove references to missing images and points.
This fixes the "Image with ID X does not exist" and "Point3D with ID Y does not exist" errors.
"""

import sys
import os
from pathlib import Path
from typing import Set

def get_existing_image_names(images_dir: Path) -> Set[str]:
    """Get set of image filenames that actually exist."""
    if not images_dir.exists():
        print(f"ERROR: Images directory does not exist: {images_dir}")
        sys.exit(1)

    image_files = set()
    for ext in ['*.jpg', '*.png', '*.jpeg', '*.JPG', '*.PNG']:
        image_files.update(f.name for f in images_dir.glob(ext))

    print(f"Found {len(image_files)} image files in {images_dir}")
    return image_files

def get_valid_image_ids(images_txt: Path, existing_images: Set[str]) -> Set[int]:
    """Parse images.txt and return set of image IDs that have corresponding files."""
    valid_ids = set()

    with open(images_txt, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            parts = line.split()
            if len(parts) < 10:
                continue

            try:
                image_id = int(parts[0])
                image_name = parts[9]  # Last field is image name

                if image_name in existing_images:
                    valid_ids.add(image_id)
            except (ValueError, IndexError):
                continue

    print(f"Found {len(valid_ids)} valid image IDs with corresponding files")
    return valid_ids

def cleanup_points3d_txt(input_path: Path, output_path: Path, valid_image_ids: Set[int]) -> Set[int]:
    """
    Remove point observations that reference missing images.
    Returns set of valid point3D IDs.
    """
    removed_points = 0
    kept_points = 0
    total_observations_removed = 0
    valid_point_ids = set()

    with open(input_path, 'r') as fin, open(output_path, 'w') as fout:
        for line in fin:
            # Always write comments
            if line.startswith('#'):
                fout.write(line)
                continue

            line = line.strip()
            if not line:
                fout.write('\n')
                continue

            parts = line.split()
            if len(parts) < 8:
                fout.write(line + '\n')
                continue

            # Format: POINT3D_ID X Y Z R G B ERROR TRACK[]
            # TRACK format: IMAGE_ID POINT2D_IDX pairs
            point_id = int(parts[0])
            xyz_rgb_error = parts[1:8]  # X Y Z R G B ERROR
            track = parts[8:]  # Remaining are IMAGE_ID POINT2D_IDX pairs

            # Filter track to only include valid image IDs
            new_track = []
            observations_removed = 0
            for i in range(0, len(track), 2):
                if i + 1 >= len(track):
                    break

                try:
                    image_id = int(track[i])
                    point2d_idx = track[i + 1]

                    if image_id in valid_image_ids:
                        new_track.extend([track[i], point2d_idx])
                    else:
                        observations_removed += 1
                except ValueError:
                    continue

            # Only write point if it has at least 2 observations
            if len(new_track) >= 4:  # At least 2 pairs (2 observations)
                fout.write(f"{point_id} {' '.join(xyz_rgb_error)} {' '.join(new_track)}\n")
                kept_points += 1
                valid_point_ids.add(point_id)
                if observations_removed > 0:
                    total_observations_removed += observations_removed
            else:
                removed_points += 1
                total_observations_removed += len(track) // 2

    print(f"points3D.txt: kept {kept_points} points, removed {removed_points} points")
    print(f"              removed {total_observations_removed} observations total")
    return valid_point_ids

def cleanup_images_txt(input_path: Path, output_path: Path, valid_image_ids: Set[int], valid_point_ids: Set[int]) -> int:
    """Remove images that don't have corresponding files and fix point3D references."""
    removed_images = 0
    kept_images = 0
    replaced_point_refs = 0

    with open(input_path, 'r') as fin, open(output_path, 'w') as fout:
        skip_next = False
        current_image_id = None

        for line in fin:
            # Always write comments
            if line.startswith('#'):
                fout.write(line)
                continue

            # Skip the keypoint line if we skipped the image line
            if skip_next:
                skip_next = False
                continue

            parts = line.strip().split()

            # Check if this is an image pose line (has 10+ fields)
            if len(parts) >= 10:
                try:
                    image_id = int(parts[0])
                    current_image_id = image_id

                    if image_id in valid_image_ids:
                        fout.write(line)
                        kept_images += 1
                    else:
                        skip_next = True  # Skip the next line (keypoints)
                        removed_images += 1
                        current_image_id = None
                except ValueError:
                    fout.write(line)
                    current_image_id = None

            # This is a keypoints line (POINTS2D: X Y POINT3D_ID triplets)
            elif current_image_id is not None and len(parts) > 0:
                # Parse keypoints: triplets of X Y POINT3D_ID
                new_parts = []
                for i in range(0, len(parts), 3):
                    if i + 2 >= len(parts):
                        # Incomplete triplet, just copy remaining
                        new_parts.extend(parts[i:])
                        break

                    try:
                        x = parts[i]
                        y = parts[i + 1]
                        point3d_id_str = parts[i + 2]
                        point3d_id = int(point3d_id_str)

                        # Replace invalid point3D IDs with -1
                        if point3d_id != -1 and point3d_id not in valid_point_ids:
                            new_parts.extend([x, y, '-1'])
                            replaced_point_refs += 1
                        else:
                            new_parts.extend([x, y, point3d_id_str])
                    except (ValueError, IndexError):
                        # If parsing fails, keep original
                        if i + 2 < len(parts):
                            new_parts.extend(parts[i:i+3])
                        else:
                            new_parts.extend(parts[i:])

                fout.write(' '.join(new_parts) + '\n')
                current_image_id = None
            else:
                # Unknown line format, keep as is
                fout.write(line)

    print(f"images.txt: kept {kept_images}, removed {removed_images}")
    print(f"            replaced {replaced_point_refs} invalid point3D references with -1")
    return removed_images

def main():
    if len(sys.argv) != 2:
        print("Usage: cleanup_colmap_data.py <colmap_output_path>")
        print("Example: cleanup_colmap_data.py /data/robot/colmaphouse")
        print("")
        print("This script will:")
        print("  1. Find which images actually exist in images/")
        print("  2. Remove point observations from text/points3D.txt that reference missing images")
        print("  3. Remove entries from text/images.txt for missing images")
        print("  4. Replace invalid point3D references in images.txt with -1")
        print("  5. Create cleaned files and backup originals")
        sys.exit(1)

    output_path = Path(sys.argv[1])
    text_path = output_path / "text"
    images_path = output_path / "images"

    print("=" * 60)
    print("COLMAP Data Cleanup")
    print("=" * 60)
    print(f"Output path: {output_path}")
    print(f"Text path: {text_path}")
    print(f"Images path: {images_path}")
    print()

    # Verify paths exist
    if not text_path.exists():
        print(f"ERROR: Text directory does not exist: {text_path}")
        sys.exit(1)

    images_txt = text_path / "images.txt"
    points3d_txt = text_path / "points3D.txt"

    if not images_txt.exists():
        print(f"ERROR: images.txt not found: {images_txt}")
        sys.exit(1)

    if not points3d_txt.exists():
        print(f"ERROR: points3D.txt not found: {points3d_txt}")
        sys.exit(1)

    # Step 1: Get existing images
    existing_images = get_existing_image_names(images_path)
    print()

    # Step 2: Get valid image IDs
    valid_image_ids = get_valid_image_ids(images_txt, existing_images)
    print()

    # Step 3: Cleanup points3D.txt first (this gives us valid point IDs)
    print("Cleaning points3D.txt...")
    points3d_cleaned = text_path / "points3D_cleaned.txt"
    valid_point_ids = cleanup_points3d_txt(points3d_txt, points3d_cleaned, valid_image_ids)
    print()

    # Step 4: Cleanup images.txt (remove invalid images and fix point references)
    print("Cleaning images.txt...")
    images_cleaned = text_path / "images_cleaned.txt"
    cleanup_images_txt(images_txt, images_cleaned, valid_image_ids, valid_point_ids)
    print()

    # Step 5: Backup originals and replace with cleaned versions
    print("Backing up original files and replacing with cleaned versions...")

    # Backup originals (if not already backed up)
    if not (text_path / "images_original.txt").exists():
        os.rename(images_txt, text_path / "images_original.txt")
    else:
        os.remove(images_txt)

    if not (text_path / "points3D_original.txt").exists():
        os.rename(points3d_txt, text_path / "points3D_original.txt")
    else:
        os.remove(points3d_txt)

    # Replace with cleaned versions
    os.rename(images_cleaned, images_txt)
    os.rename(points3d_cleaned, points3d_txt)

    print("=" * 60)
    print("✓ Cleanup completed!")
    print("=" * 60)
    print("Original files backed up as:")
    print(f"  - {text_path / 'images_original.txt'}")
    print(f"  - {text_path / 'points3D_original.txt'}")
    print()
    print("You can now run COLMAP refinement:")
    print(f"  ./colmap_processor/colmap_refine_existing.sh {output_path}")

if __name__ == "__main__":
    main()
