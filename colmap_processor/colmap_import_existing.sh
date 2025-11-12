#!/bin/bash
# COLMAP Option 1: Import Existing Reconstruction into Database
# This script imports your existing reconstruction (with poses and 3D points) into COLMAP

set -e  # Exit on error

# Check arguments
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <colmap_output_path>"
    echo "Example: $0 /tmp/colmap_output"
    exit 1
fi

OUTPUT_PATH="$1"
TEXT_PATH="${OUTPUT_PATH}/text"
IMAGE_PATH="${OUTPUT_PATH}/images"
DATABASE_PATH="${OUTPUT_PATH}/database.db"
SPARSE_PATH="${OUTPUT_PATH}/sparse"

echo "=================================================="
echo "COLMAP Option 1: Import Existing Reconstruction"
echo "=================================================="
echo "Output path: ${OUTPUT_PATH}"
echo "Text files: ${TEXT_PATH}"
echo "Images: ${IMAGE_PATH}"
echo ""

# Verify input files exist
if [ ! -f "${TEXT_PATH}/cameras.txt" ]; then
    echo "ERROR: cameras.txt not found at ${TEXT_PATH}"
    exit 1
fi

if [ ! -d "${IMAGE_PATH}" ]; then
    echo "ERROR: Images directory not found at ${IMAGE_PATH}"
    exit 1
fi

# Step 1: Convert text format to binary format
echo "[1/3] Converting text format to binary format..."
mkdir -p "${SPARSE_PATH}/0"
colmap model_converter \
    --input_path "${TEXT_PATH}" \
    --output_path "${SPARSE_PATH}/0" \
    --output_type BIN

echo "✓ Binary model created at ${SPARSE_PATH}/0"
echo ""

# Step 2: Create database
echo "[2/3] Creating COLMAP database..."
if [ -f "${DATABASE_PATH}" ]; then
    echo "WARNING: Database already exists, removing..."
    rm "${DATABASE_PATH}"
fi

colmap database_creator \
    --database_path "${DATABASE_PATH}"

echo "✓ Database created at ${DATABASE_PATH}"
echo ""

# Step 3: Extract features (using camera params from cameras.txt)
echo "[3/3] Extracting features from images..."
# Read camera parameters from cameras.txt
CAMERA_PARAMS=$(grep -E "^1 PINHOLE" "${TEXT_PATH}/cameras.txt" | awk '{print $5","$6","$7","$8}')
echo "Using camera parameters: ${CAMERA_PARAMS}"

colmap feature_extractor \
    --database_path "${DATABASE_PATH}" \
    --image_path "${IMAGE_PATH}" \
    --ImageReader.camera_model PINHOLE \
    --ImageReader.single_camera 1 \
    --ImageReader.camera_params "${CAMERA_PARAMS}"

echo "✓ Features extracted"
echo ""

echo "=================================================="
echo "✓ Import completed successfully!"
echo "=================================================="
echo "Database: ${DATABASE_PATH}"
echo "Binary model: ${SPARSE_PATH}/0"
echo ""
echo "You can now:"
echo "  - View in COLMAP GUI: colmap gui --database_path ${DATABASE_PATH} --image_path ${IMAGE_PATH} --import_path ${SPARSE_PATH}/0"
echo "  - Run bundle adjustment to refine"
echo "  - Export to other formats"
