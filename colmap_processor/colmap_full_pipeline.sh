#!/bin/bash
# COLMAP Option 2: Full Pipeline from Scratch
# This script runs the complete COLMAP pipeline: feature extraction, matching, and reconstruction

set -e  # Exit on error

# Check arguments
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <colmap_output_path> [matcher_type]"
    echo "Example: $0 /tmp/colmap_output exhaustive"
    echo ""
    echo "Matcher types:"
    echo "  exhaustive  - Match all image pairs (default, best quality, slow for >100 images)"
    echo "  sequential  - Match sequential images only (faster, good for video)"
    echo "  spatial     - Match spatially nearby images (requires GPS)"
    echo "  vocab_tree  - Match using vocabulary tree (fastest for large datasets)"
    exit 1
fi

OUTPUT_PATH="$1"
MATCHER_TYPE="${2:-exhaustive}"  # Default to exhaustive
IMAGE_PATH="${OUTPUT_PATH}/images"
DATABASE_PATH="${OUTPUT_PATH}/database.db"
SPARSE_PATH="${OUTPUT_PATH}/sparse_reconstructed"

echo "=================================================="
echo "COLMAP Option 2: Full Pipeline from Scratch"
echo "=================================================="
echo "Output path: ${OUTPUT_PATH}"
echo "Images: ${IMAGE_PATH}"
echo "Matcher type: ${MATCHER_TYPE}"
echo ""

# Verify input
if [ ! -d "${IMAGE_PATH}" ]; then
    echo "ERROR: Images directory not found at ${IMAGE_PATH}"
    exit 1
fi

# Count images
NUM_IMAGES=$(ls -1 "${IMAGE_PATH}"/*.jpg 2>/dev/null | wc -l)
echo "Found ${NUM_IMAGES} images"
echo ""

# Step 1: Create database
echo "[1/4] Creating COLMAP database..."
if [ -f "${DATABASE_PATH}" ]; then
    echo "WARNING: Database already exists, removing..."
    rm "${DATABASE_PATH}"
fi

colmap database_creator \
    --database_path "${DATABASE_PATH}"

echo "✓ Database created"
echo ""

# Step 2: Extract features
echo "[2/4] Extracting features from images..."
echo "This may take a while depending on number of images..."

colmap feature_extractor \
    --database_path "${DATABASE_PATH}" \
    --image_path "${IMAGE_PATH}" \
    --ImageReader.camera_model PINHOLE \
    --ImageReader.single_camera 1 \
    --FeatureExtraction.use_gpu 1

echo "✓ Features extracted"
echo ""

# Step 3: Match features
echo "[3/4] Matching features using ${MATCHER_TYPE} matcher..."
echo "This may take a while..."

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
    spatial)
        echo "ERROR: Spatial matcher requires GPS data in images"
        exit 1
        ;;
    vocab_tree)
        echo "ERROR: Vocab tree matcher requires vocabulary tree file"
        echo "Download from: https://demuc.de/colmap/"
        exit 1
        ;;
    *)
        echo "ERROR: Unknown matcher type: ${MATCHER_TYPE}"
        exit 1
        ;;
esac

echo "✓ Features matched"
echo ""

# Step 4: Run mapper (Structure from Motion)
echo "[4/4] Running mapper (Structure from Motion)..."
echo "This will reconstruct camera poses and 3D points..."

mkdir -p "${SPARSE_PATH}"

colmap mapper \
    --database_path "${DATABASE_PATH}" \
    --image_path "${IMAGE_PATH}" \
    --output_path "${SPARSE_PATH}" \
    --Mapper.ba_refine_focal_length 0 \
    --Mapper.ba_refine_principal_point 0 \
    --Mapper.ba_refine_extra_params 0

echo "✓ Mapping completed"
echo ""

# Check if reconstruction was successful
if [ -f "${SPARSE_PATH}/0/cameras.bin" ]; then
    echo "=================================================="
    echo "✓ Full pipeline completed successfully!"
    echo "=================================================="
    echo "Database: ${DATABASE_PATH}"
    echo "Reconstruction: ${SPARSE_PATH}/0"
    echo ""

    # Print reconstruction statistics
    echo "Reconstruction statistics:"
    colmap model_analyzer \
        --path "${SPARSE_PATH}/0"
    echo ""

    echo "You can now:"
    echo "  - View in COLMAP GUI: colmap gui --database_path ${DATABASE_PATH} --image_path ${IMAGE_PATH} --import_path ${SPARSE_PATH}/0"
    echo "  - Run bundle adjustment to refine"
    echo "  - Generate dense reconstruction"
else
    echo "=================================================="
    echo "✗ Reconstruction failed or produced no models"
    echo "=================================================="
    echo "This can happen if:"
    echo "  - Not enough feature matches between images"
    echo "  - Images are too different (different scenes)"
    echo "  - Images lack texture/features"
    echo ""
    echo "Try:"
    echo "  - Using a different matcher (sequential if images are video frames)"
    echo "  - Checking image quality and overlap"
    exit 1
fi
