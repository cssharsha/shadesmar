#!/bin/bash

# Quick Gaussian Splatting Test Script
# Simplified version for rapid testing

set -e

CONTAINER_NAME="docker-dev-1"
MAP_BASE_PATH="/data/robot/house11_map"

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