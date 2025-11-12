#!/bin/bash
# COLMAP Option 3: Refine Existing Poses with Bundle Adjustment
# This script uses your existing poses as a prior and refines them with COLMAP

set -e  # Exit on error

# Check arguments
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <colmap_output_path> [options]"
    echo "Example: $0 /tmp/colmap_output"
    echo ""
    echo "Options:"
    echo "  --no-refine-intrinsics    Keep camera intrinsics fixed"
    echo "  --refine-intrinsics       Allow refinement of focal length and principal point"
    echo "  --sequential-matching     Use sequential matcher instead of exhaustive"
    exit 1
fi

OUTPUT_PATH="$1"
shift

# Parse options
REFINE_FOCAL=0
REFINE_PRINCIPAL=0
MATCHER_TYPE="exhaustive"

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-refine-intrinsics)
            REFINE_FOCAL=0
            REFINE_PRINCIPAL=0
            shift
            ;;
        --refine-intrinsics)
            REFINE_FOCAL=1
            REFINE_PRINCIPAL=1
            shift
            ;;
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
REFINED_PATH="${OUTPUT_PATH}/sparse_refined"

echo "=================================================="
echo "COLMAP Option 3: Refine Existing Poses"
echo "=================================================="
echo "Output path: ${OUTPUT_PATH}"
echo "Text files: ${TEXT_PATH}"
echo "Images: ${IMAGE_PATH}"
echo "Matcher: ${MATCHER_TYPE}"
echo "Refine focal length: ${REFINE_FOCAL}"
echo "Refine principal point: ${REFINE_PRINCIPAL}"
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

# Step 1: Create database
echo "[1/6] Creating COLMAP database..."
if [ -f "${DATABASE_PATH}" ]; then
    echo "WARNING: Database already exists, removing..."
    rm "${DATABASE_PATH}"
fi

colmap database_creator \
    --database_path "${DATABASE_PATH}"

echo "✓ Database created"
echo ""

# Step 2: Extract features
echo "[2/6] Extracting features from images..."
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
echo "[3/6] Matching features using ${MATCHER_TYPE} matcher..."

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

# Step 4: Import existing reconstruction
echo "[4/6] Importing existing reconstruction..."
mkdir -p "${SPARSE_PATH}/0"

colmap model_converter \
    --input_path "${TEXT_PATH}" \
    --output_path "${SPARSE_PATH}/0" \
    --output_type BIN

echo "✓ Existing model imported"
echo ""

# Step 5: Register images with their poses in the database (as priors)
echo "[5/6] Registering existing poses in database as priors..."
mkdir -p "${SPARSE_PATH}/registered"

colmap image_registrator \
    --database_path "${DATABASE_PATH}" \
    --input_path "${SPARSE_PATH}/0" \
    --output_path "${SPARSE_PATH}/registered"

# Use the registered model for bundle adjustment
if [ -d "${SPARSE_PATH}/registered/0" ]; then
    echo "✓ Poses registered as priors, using registered model"
    # Copy registered model back to sparse/0
    rm -rf "${SPARSE_PATH}/0"
    mv "${SPARSE_PATH}/registered/0" "${SPARSE_PATH}/0"
    rmdir "${SPARSE_PATH}/registered"
else
    echo "✓ Poses registered as priors"
fi
echo ""

# Step 6: Bundle adjustment to refine
echo "[6/6] Running bundle adjustment to refine poses..."
mkdir -p "${REFINED_PATH}/0"

colmap bundle_adjuster \
    --input_path "${SPARSE_PATH}/0" \
    --output_path "${REFINED_PATH}/0" \
    --BundleAdjustment.refine_focal_length ${REFINE_FOCAL} \
    --BundleAdjustment.refine_principal_point ${REFINE_PRINCIPAL} \
    --BundleAdjustment.refine_extra_params 0

echo "✓ Bundle adjustment completed"
echo ""

# Convert refined model back to text format
echo "Converting refined model to text format..."
mkdir -p "${REFINED_PATH}/text"
colmap model_converter \
    --input_path "${REFINED_PATH}/0" \
    --output_path "${REFINED_PATH}/text" \
    --output_type TXT

echo "=================================================="
echo "✓ Refinement completed successfully!"
echo "=================================================="
echo "Database: ${DATABASE_PATH}"
echo "Original model: ${SPARSE_PATH}/0"
echo "Refined model (binary): ${REFINED_PATH}/0"
echo "Refined model (text): ${REFINED_PATH}/text"
echo ""

# Compare original vs refined
echo "Reconstruction statistics:"
echo ""
echo "Original model:"
colmap model_analyzer --path "${SPARSE_PATH}/0"
echo ""
echo "Refined model:"
colmap model_analyzer --path "${REFINED_PATH}/0"
echo ""

echo "You can now:"
echo "  - View original: colmap gui --database_path ${DATABASE_PATH} --image_path ${IMAGE_PATH} --import_path ${SPARSE_PATH}/0"
echo "  - View refined: colmap gui --database_path ${DATABASE_PATH} --image_path ${IMAGE_PATH} --import_path ${REFINED_PATH}/0"
echo "  - Compare poses in text files: diff ${TEXT_PATH}/images.txt ${REFINED_PATH}/text/images.txt"
echo "  - Generate dense reconstruction from refined model"
