#!/bin/bash

# Quick Gaussian Splatting Test Script
# Simplified version for rapid testing

set -e

CONTAINER_NAME="docker-dev-1"

# Auto-detect map path from common bag locations
if [[ -f "/data/robot/bags/house11/house11_0.db3" ]]; then
    MAP_BASE_PATH="/data/robot/house11_map"
elif [[ -f "/data/robot/bags/office/office_0.db3" ]]; then
    MAP_BASE_PATH="/data/robot/office_map"
else
    MAP_BASE_PATH="/data/robot/test_map"
    echo "Warning: Using default test map path. Adjust if needed."
fi

echo "=== Quick Gaussian Splatting Test ==="

# Check container
if ! docker ps --filter "name=$CONTAINER_NAME" --filter "status=running" | grep -q "$CONTAINER_NAME"; then
    echo "Error: Container $CONTAINER_NAME is not running"
    exit 1
fi

# Build and run GS processor only
echo "Building Gaussian splat processor..."
docker exec "$CONTAINER_NAME" bash -c "cd /workspace && bazel build //gaussian_splatting:gs_processor"

echo "Running Gaussian splat processor..."
echo "Map path: $MAP_BASE_PATH"
echo ""

# Run in foreground for testing
docker exec -it "$CONTAINER_NAME" bash -c "cd /workspace && ./bazel-bin/gaussian_splatting/gs_processor '$MAP_BASE_PATH'"