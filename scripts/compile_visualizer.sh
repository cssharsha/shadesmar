#!/bin/bash

# Script to compile the visualize_map_keypoints.cpp

set -e

echo "Compiling Map Keypoint Visualizer..."

# Check if we're in the workspace directory
if [[ ! -d "core" ]]; then
    echo "Error: Please run this script from the workspace root directory"
    exit 1
fi

# Find OpenCV
OPENCV_FLAGS=$(pkg-config --cflags --libs opencv4 2>/dev/null || pkg-config --cflags --libs opencv 2>/dev/null || echo "")

if [[ -z "$OPENCV_FLAGS" ]]; then
    echo "Warning: OpenCV not found via pkg-config, using manual flags"
    OPENCV_FLAGS="-I/usr/include/opencv4 -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui"
fi

# Compile command
g++ -std=c++17 \
    -I. \
    -Icore \
    scripts/visualize_map_keypoints.cpp \
    core/storage/src/map_store.cpp \
    core/types/src/*.cpp \
    -o scripts/visualize_map_keypoints \
    $OPENCV_FLAGS \
    -lprotobuf \
    -lglog \
    -lpthread \
    -lstdc++fs \
    -O2

if [[ $? -eq 0 ]]; then
    echo "✅ Compilation successful!"
    echo "Executable created: scripts/visualize_map_keypoints"
    echo ""
    echo "Usage:"
    echo "  ./scripts/visualize_map_keypoints <map_base_path> [options]"
    echo ""
    echo "Example:"
    echo "  ./scripts/visualize_map_keypoints /data/robots -o /tmp/vis -m 4 -k 20"
    echo ""
    echo "Run with --help for full options"
else
    echo "❌ Compilation failed"
    exit 1
fi