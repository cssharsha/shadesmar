#!/bin/bash
# COLMAP Reconstruction with Pose Priors
# This script uses exported camera poses as priors and lets COLMAP triangulate 3D points

set -e  # Exit on error

# Check arguments
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <colmap_output_path> [options]"
    echo "Example: $0 /tmp/colmap_output"
    echo ""
    echo "Options:"
    echo "  --sequential-matching     Use sequential matcher instead of exhaustive"
    echo ""
    echo "This script expects:"
    echo "  <colmap_output_path>/text/cameras.txt"
    echo "  <colmap_output_path>/text/images.txt (with poses, no 3D points)"
    echo "  <colmap_output_path>/images/"
    exit 1
fi

OUTPUT_PATH="$1"
shift

# Parse options
MATCHER_TYPE="exhaustive"

while [[ $# -gt 0 ]]; do
    case $1 in
        --sequential-matching)
            MATCHER_TYPE="sequential"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

TEXT_PATH="${OUTPUT_PATH}/text"
IMAGE_PATH="${OUTPUT_PATH}/images"
DATABASE_PATH="${OUTPUT_PATH}/database.db"
SPARSE_PATH="${OUTPUT_PATH}/sparse"
RECONSTRUCTED_PATH="${OUTPUT_PATH}/reconstructed"

echo "=================================================="
echo "COLMAP Reconstruction with Pose Priors"
echo "=================================================="
echo "Output path: ${OUTPUT_PATH}"
echo "Text files: ${TEXT_PATH}"
echo "Images: ${IMAGE_PATH}"
echo "Matcher: ${MATCHER_TYPE}"
echo ""

# Verify input files exist
if [ ! -f "${TEXT_PATH}/cameras.txt" ]; then
    echo "ERROR: cameras.txt not found at ${TEXT_PATH}"
    exit 1
fi

if [ ! -f "${TEXT_PATH}/images.txt" ]; then
    echo "ERROR: images.txt not found at ${TEXT_PATH}"
    exit 1
fi

if [ ! -d "${IMAGE_PATH}" ]; then
    echo "ERROR: Images directory not found at ${IMAGE_PATH}"
    exit 1
fi

# Step 1: Create database
echo "[1/5] Creating COLMAP database..."
if [ -f "${DATABASE_PATH}" ]; then
    echo "WARNING: Database already exists, removing..."
    rm "${DATABASE_PATH}"
fi

colmap database_creator \
    --database_path "${DATABASE_PATH}"

echo "✓ Database created"
echo ""

# Step 2: Extract features
echo "[2/5] Extracting features from images..."
CAMERA_PARAMS=$(grep -E "^1 PINHOLE" "${TEXT_PATH}/cameras.txt" | awk '{print $5","$6","$7","$8}')
echo "Using camera parameters: ${CAMERA_PARAMS}"

colmap feature_extractor \
    --database_path "${DATABASE_PATH}" \
    --image_path "${IMAGE_PATH}" \
    --ImageReader.camera_model PINHOLE \
    --ImageReader.single_camera 1 \
    --ImageReader.camera_params "${CAMERA_PARAMS}" \
    --FeatureExtraction.use_gpu 1

echo "✓ Features extracted"
echo ""

# Step 3: Match features
echo "[3/5] Matching features using ${MATCHER_TYPE} matcher..."

case "${MATCHER_TYPE}" in
    exhaustive)
        colmap exhaustive_matcher \
            --database_path "${DATABASE_PATH}" \
            --FeatureMatching.use_gpu 1
        ;;
    sequential)
        colmap sequential_matcher \
            --database_path "${DATABASE_PATH}" \
            --FeatureMatching.use_gpu 1 \
            --SequentialMatching.overlap 10
        ;;
    *)
        echo "ERROR: Unknown matcher type: ${MATCHER_TYPE}"
        exit 1
        ;;
esac

echo "✓ Features matched"
echo ""

# Step 4: Register image poses as priors in the database
echo "[4/5] Registering image poses as priors..."

# We need to import the poses into the database
# The easiest way is to use colmap's database utilities
# For now, we'll use image_registrator which needs an initial reconstruction

# First, create a minimal reconstruction with just cameras and images (no points)
mkdir -p "${SPARSE_PATH}/0"

# Create a minimal points3D.txt (empty but valid)
echo "# 3D point list with one line of data per point:" > "${TEXT_PATH}/points3D.txt"
echo "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)" >> "${TEXT_PATH}/points3D.txt"
echo "# Number of points: 0, mean track length: 0.00000" >> "${TEXT_PATH}/points3D.txt"

# Now convert to binary
colmap model_converter \
    --input_path "${TEXT_PATH}" \
    --output_path "${SPARSE_PATH}/0" \
    --output_type BIN

echo "✓ Initial model created"
echo ""

# Step 5: Run point triangulator to create 3D points using the pose priors
echo "[5/5] Triangulating 3D points with pose priors..."
mkdir -p "${RECONSTRUCTED_PATH}"

colmap point_triangulator \
    --database_path "${DATABASE_PATH}" \
    --image_path "${IMAGE_PATH}" \
    --input_path "${SPARSE_PATH}/0" \
    --output_path "${RECONSTRUCTED_PATH}" \
    --Mapper.ba_refine_focal_length 0 \
    --Mapper.ba_refine_principal_point 0 \
    --Mapper.ba_refine_extra_params 0

echo "✓ Triangulation completed"
echo ""

# Convert reconstructed model to text format
echo "Converting reconstructed model to text format..."
mkdir -p "${RECONSTRUCTED_PATH}_text"
colmap model_converter \
    --input_path "${RECONSTRUCTED_PATH}" \
    --output_path "${RECONSTRUCTED_PATH}_text" \
    --output_type TXT

echo "=================================================="
echo "✓ Reconstruction completed successfully!"
echo "=================================================="
echo "Database: ${DATABASE_PATH}"
echo "Initial poses: ${SPARSE_PATH}/0"
echo "Reconstructed model (binary): ${RECONSTRUCTED_PATH}"
echo "Reconstructed model (text): ${RECONSTRUCTED_PATH}_text"
echo ""

# Analyze result
echo "Reconstruction statistics:"
colmap model_analyzer --path "${RECONSTRUCTED_PATH}"
echo ""

echo "You can now:"
echo "  - View result: colmap gui --database_path ${DATABASE_PATH} --image_path ${IMAGE_PATH} --import_path ${RECONSTRUCTED_PATH}"
echo "  - Run bundle adjustment to further refine"
echo "  - Generate dense reconstruction"
