#!/bin/bash

# Script to generate compile_commands.json using hedron_compile_commands
# Runs bazel inside Docker container and fixes paths for host usage

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONTAINER_NAME="docker-dev-1"

echo "Generating compile_commands.json using hedron_compile_commands..."

# Check if container is running
if ! docker ps --format "table {{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Error: Container '${CONTAINER_NAME}' is not running."
    echo "Start it with: cd dev/docker && docker-compose up -d"
    exit 1
fi

# Run hedron compile commands refresh inside container with important build flags
echo "Running bazel run @hedron_compile_commands//:refresh_all with build flags..."
docker exec -w /workspace "${CONTAINER_NAME}" bazel run @hedron_compile_commands//:refresh_all -- --config=cuda

# Fix paths in compile_commands.json (from /workspace to actual project path)
if [ -f "${PROJECT_ROOT}/compile_commands.json" ]; then
    echo "Fixing paths in compile_commands.json..."
    sed -i "s|/workspace|${PROJECT_ROOT}|g" "${PROJECT_ROOT}/compile_commands.json"
    
    echo "✓ compile_commands.json generated and paths fixed for host usage"
    echo "File location: ${PROJECT_ROOT}/compile_commands.json"
else
    echo "Error: compile_commands.json not found after hedron refresh"
    exit 1
fi

echo "Done! clangd should now work properly with external dependencies."
