#!/bin/bash
# Master script to run COLMAP processing with different options
# This script helps you choose the right COLMAP pipeline for your data

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${BLUE}=================================================="
    echo -e "$1"
    echo -e "==================================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

show_usage() {
    cat << EOF
Usage: $0 <option> <colmap_output_path> [additional_args]

This script runs COLMAP processing on your exported map data.

OPTIONS:
  1, import       Import existing reconstruction (fastest, preserves your poses)
  2, full         Run full COLMAP pipeline from scratch (slow, computes new poses)
  3, refine       Refine existing poses with bundle adjustment (recommended)

EXAMPLES:
  # Option 1: Import existing reconstruction
  $0 1 /tmp/colmap_output

  # Option 2: Full pipeline with sequential matching
  $0 2 /tmp/colmap_output sequential

  # Option 3: Refine with intrinsics refinement
  $0 3 /tmp/colmap_output --refine-intrinsics

DESCRIPTION OF OPTIONS:

Option 1: Import Existing Reconstruction
  - Converts your text format to COLMAP binary format
  - Creates database and extracts features
  - Preserves your existing poses and 3D points
  - Use when: You trust your mapping system's poses
  - Speed: Fast (minutes)

Option 2: Full Pipeline from Scratch
  - Extracts features from images
  - Matches features between images
  - Computes camera poses from scratch (SfM)
  - Triangulates 3D points
  - Use when: You want COLMAP to compute everything
  - Speed: Slow (can take hours for many images)
  - Matcher types: exhaustive, sequential

Option 3: Refine Existing Poses (RECOMMENDED)
  - Starts with your existing poses
  - Extracts and matches features
  - Runs bundle adjustment to refine
  - Can optionally refine camera intrinsics
  - Use when: You want to improve/validate your poses
  - Speed: Medium (minutes to hour)
  - Options: --no-refine-intrinsics, --refine-intrinsics, --sequential-matching

For more details on each option, run the individual scripts with --help

EOF
}

# Check if colmap is available
check_colmap() {
    if ! command -v colmap &> /dev/null; then
        print_error "COLMAP is not installed or not in PATH"
        echo "Please install COLMAP: https://colmap.github.io/"
        exit 1
    fi

    COLMAP_VERSION=$(colmap -h 2>&1 | grep -oP 'COLMAP \K[0-9.]+' | head -1)
    print_success "Found COLMAP version ${COLMAP_VERSION}"
}

# Main script
if [ "$#" -lt 2 ]; then
    show_usage
    exit 1
fi

OPTION="$1"
OUTPUT_PATH="$2"
shift 2

print_header "COLMAP Processing Script"
check_colmap
echo ""

# Check if output path exists
if [ ! -d "${OUTPUT_PATH}" ]; then
    print_error "Output path does not exist: ${OUTPUT_PATH}"
    echo "Please run the map_to_colmap_converter first to generate the data"
    exit 1
fi

# Check if images directory exists
if [ ! -d "${OUTPUT_PATH}/images" ]; then
    print_error "Images directory not found: ${OUTPUT_PATH}/images"
    echo "Please run the map_to_colmap_converter first to generate the images"
    exit 1
fi

# Count images
NUM_IMAGES=$(ls -1 "${OUTPUT_PATH}/images"/*.jpg 2>/dev/null | wc -l)
echo "Data path: ${OUTPUT_PATH}"
echo "Number of images: ${NUM_IMAGES}"
echo ""

# Run the selected option
case "${OPTION}" in
    1|import)
        print_header "Running Option 1: Import Existing Reconstruction"
        bash "${SCRIPT_DIR}/colmap_import_existing.sh" "${OUTPUT_PATH}" "$@"
        ;;

    2|full)
        print_header "Running Option 2: Full COLMAP Pipeline"
        if [ ${NUM_IMAGES} -gt 200 ]; then
            print_warning "You have ${NUM_IMAGES} images. Exhaustive matching may take very long."
            echo "Consider using sequential matcher: $0 2 ${OUTPUT_PATH} sequential"
            echo ""
        fi
        bash "${SCRIPT_DIR}/colmap_full_pipeline.sh" "${OUTPUT_PATH}" "$@"
        ;;

    3|refine)
        print_header "Running Option 3: Refine Existing Poses"

        # Check if text files exist
        if [ ! -f "${OUTPUT_PATH}/text/cameras.txt" ]; then
            print_error "Text format files not found at ${OUTPUT_PATH}/text"
            echo "Please run the map_to_colmap_converter first"
            exit 1
        fi

        bash "${SCRIPT_DIR}/colmap_refine_existing.sh" "${OUTPUT_PATH}" "$@"
        ;;

    *)
        print_error "Unknown option: ${OPTION}"
        echo ""
        show_usage
        exit 1
        ;;
esac
