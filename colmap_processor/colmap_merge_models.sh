#!/bin/bash
# COLMAP Model Merger Script
# Merges multiple reconstructions into a single unified model

set -e  # Exit on error

# Check arguments
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <sparse_path> [output_path]"
    echo "Example: $0 /tmp/colmap_output2/sparse_reconstructed"
    echo ""
    echo "This will merge all models in <sparse_path>/{0,1,2,...} into a single model"
    echo "Output will be written to [output_path] (default: <sparse_path>/merged)"
    exit 1
fi

SPARSE_PATH="$1"
OUTPUT_PATH="${2:-${SPARSE_PATH}/merged}"

echo "=================================================="
echo "COLMAP Model Merger"
echo "=================================================="
echo "Input path: ${SPARSE_PATH}"
echo "Output path: ${OUTPUT_PATH}"
echo ""

# Check if input path exists
if [ ! -d "${SPARSE_PATH}" ]; then
    echo "ERROR: Sparse path does not exist: ${SPARSE_PATH}"
    exit 1
fi

# Find all model directories
MODEL_DIRS=()
for dir in "${SPARSE_PATH}"/[0-9]*; do
    if [ -d "$dir" ] && [ -f "$dir/cameras.bin" ]; then
        MODEL_DIRS+=("$dir")
    fi
done

if [ ${#MODEL_DIRS[@]} -eq 0 ]; then
    echo "ERROR: No valid model directories found in ${SPARSE_PATH}"
    exit 1
fi

echo "Found ${#MODEL_DIRS[@]} models to merge:"
for i in "${!MODEL_DIRS[@]}"; do
    model_path="${MODEL_DIRS[$i]}"
    model_name=$(basename "$model_path")
    echo "  [$i] Model ${model_name}:"
    colmap model_analyzer --path "$model_path" 2>&1 | grep -E "(Registered images|Points|Mean reprojection)"
done
echo ""

# If only one model, just copy it
if [ ${#MODEL_DIRS[@]} -eq 1 ]; then
    echo "Only one model found, copying to output..."
    mkdir -p "${OUTPUT_PATH}"
    cp -r "${MODEL_DIRS[0]}"/* "${OUTPUT_PATH}/"
    echo "✓ Model copied to ${OUTPUT_PATH}"
    exit 0
fi

# Create output directory
mkdir -p "${OUTPUT_PATH}"

# Method 1: Try automatic merge (works if models share common images/points)
echo "Attempting automatic model merge..."
echo "This works if models have overlapping images/points"
echo ""

# Build input path list (space-separated)
INPUT_PATHS=""
for model_dir in "${MODEL_DIRS[@]}"; do
    INPUT_PATHS="${INPUT_PATHS} ${model_dir}"
done

# Try to merge
if colmap model_merger \
    --input_path1 "${MODEL_DIRS[0]}" \
    --input_path2 "${MODEL_DIRS[1]}" \
    --output_path "${OUTPUT_PATH}/temp_merge"; then

    echo "✓ Successfully merged models 0 and 1"

    # If there's a third model, merge it too
    if [ ${#MODEL_DIRS[@]} -gt 2 ]; then
        for ((i=2; i<${#MODEL_DIRS[@]}; i++)); do
            echo "Merging model $i..."

            # Move previous merge to input
            mv "${OUTPUT_PATH}/temp_merge" "${OUTPUT_PATH}/temp_merge_prev"

            if colmap model_merger \
                --input_path1 "${OUTPUT_PATH}/temp_merge_prev" \
                --input_path2 "${MODEL_DIRS[$i]}" \
                --output_path "${OUTPUT_PATH}/temp_merge"; then

                echo "✓ Successfully merged model $i"
                rm -rf "${OUTPUT_PATH}/temp_merge_prev"
            else
                echo "⚠ Warning: Could not merge model $i, skipping..."
                mv "${OUTPUT_PATH}/temp_merge_prev" "${OUTPUT_PATH}/temp_merge"
            fi
        done
    fi

    # Move final merge to output
    mv "${OUTPUT_PATH}/temp_merge"/* "${OUTPUT_PATH}/"
    rmdir "${OUTPUT_PATH}/temp_merge"

else
    echo "✗ Automatic merge failed (models may not overlap)"
    echo ""
    echo "Models are likely disconnected components."
    echo "Options:"
    echo "  1. Keep the largest model (recommended)"
    echo "  2. Concatenate all models (no alignment, separate components)"
    echo "  3. Manually align using known correspondences"
    echo ""

    # Find largest model
    LARGEST_MODEL=""
    LARGEST_SIZE=0

    for model_dir in "${MODEL_DIRS[@]}"; do
        num_images=$(colmap model_analyzer --path "$model_dir" 2>&1 | grep "Registered images" | awk '{print $3}')
        if [ -n "$num_images" ] && [ "$num_images" -gt "$LARGEST_SIZE" ]; then
            LARGEST_SIZE=$num_images
            LARGEST_MODEL=$model_dir
        fi
    done

    echo "Largest model: $(basename $LARGEST_MODEL) with $LARGEST_SIZE images"
    echo ""

    # Check if running in non-interactive mode (e.g., in docker/CI)
    if [ -t 0 ]; then
        read -p "Choose option (1/2/3) [default: 1]: " choice
        choice=${choice:-1}
    else
        echo "Running in non-interactive mode, automatically choosing option 1 (largest model)"
        choice=1
    fi

    case $choice in
        1)
            echo "Copying largest model..."
            cp -r "${LARGEST_MODEL}"/* "${OUTPUT_PATH}/"
            echo "✓ Largest model copied to ${OUTPUT_PATH}"
            ;;
        2)
            echo "Concatenating all models (warning: no alignment)..."
            # This would require custom code to renumber IDs
            echo "ERROR: Concatenation not yet implemented"
            echo "Please use Option 1 or manually align models"
            exit 1
            ;;
        3)
            echo "Manual alignment requires:"
            echo "  1. Identify common points between models"
            echo "  2. Use colmap model_aligner"
            echo "  3. Then merge aligned models"
            echo ""
            echo "See COLMAP documentation for details"
            exit 1
            ;;
        *)
            echo "Invalid option"
            exit 1
            ;;
    esac
fi

# Analyze merged result
echo ""
echo "=================================================="
echo "✓ Merge completed"
echo "=================================================="
echo "Merged model statistics:"
colmap model_analyzer --path "${OUTPUT_PATH}"
echo ""
echo "Output: ${OUTPUT_PATH}"
echo ""

# Convert to text format for easy inspection
echo "Converting merged model to text format..."
mkdir -p "${OUTPUT_PATH}_text"
colmap model_converter \
    --input_path "${OUTPUT_PATH}" \
    --output_path "${OUTPUT_PATH}_text" \
    --output_type TXT

echo "✓ Text format: ${OUTPUT_PATH}_text"
echo ""
echo "You can now:"
echo "  - View merged model: colmap gui --import_path ${OUTPUT_PATH}"
echo "  - Run bundle adjustment to further refine"
echo "  - Generate dense reconstruction"
