#!/bin/bash

# Gaussian Splatting + Visualization Runner Script
# This script runs the Gaussian splat processor and ROS visualization in tandem

set -e

# Default parameters
ROSBAG_PATH="/data/robot/bags/house11/house11_0.db3"
CONFIG="default"
MAP_BASE_PATH=""  # Will be derived from bagfile path
CONTAINER_NAME="docker-dev-1"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "OPTIONS:"
    echo "  -b, --bag PATH        Path to rosbag file (default: $ROSBAG_PATH)"
    echo "  -c, --config CONFIG   Visualization config (default: $CONFIG)"
    echo "  -m, --map PATH        Map base path for Gaussian splat processor (default: $MAP_BASE_PATH)"
    echo "  -h, --help           Show this help message"
    echo ""
    echo "EXAMPLES:"
    echo "  $0"
    echo "  $0 -b /data/robot/bags/office/office_0.db3 -m /data/robot/office_map"
    echo "  $0 --bag /data/custom.db3 --config tum --map /data/custom_map"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--bag)
            ROSBAG_PATH="$2"
            shift 2
            ;;
        -c|--config)
            CONFIG="$2"
            shift 2
            ;;
        -m|--map)
            MAP_BASE_PATH="$2"
            shift 2
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            print_usage
            exit 1
            ;;
    esac
done

# Function to check if container is running
check_container() {
    if ! docker ps --filter "name=$CONTAINER_NAME" --filter "status=running" | grep -q "$CONTAINER_NAME"; then
        echo -e "${RED}Error: Container $CONTAINER_NAME is not running${NC}"
        echo "Please start the container first:"
        echo "  docker start $CONTAINER_NAME"
        exit 1
    fi
}

# Function to build targets in container
build_targets() {
    echo -e "${BLUE}Building Gaussian splat processor and visualization...${NC}"
    
    if ! docker exec "$CONTAINER_NAME" bash -c "cd /workspace && bazel build //gaussian_splatting:gs_processor //viz:visualize_rosbag 2>&1 | tee gs_log"; then
        echo -e "${RED}Build failed!${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}Build completed successfully${NC}"
}

# Function to cleanup background processes
cleanup() {
    echo -e "${YELLOW}Cleaning up background processes...${NC}"
    
    # Kill Gaussian splat processor
    if [[ -n "$GS_PID" ]]; then
        docker exec "$CONTAINER_NAME" bash -c "pkill -f gs_processor" 2>/dev/null || true
        echo "Stopped Gaussian splat processor"
    fi
    
    # Kill visualization process  
    if [[ -n "$VIZ_PID" ]]; then
        kill "$VIZ_PID" 2>/dev/null || true
        echo "Stopped visualization process"
    fi
    
    echo -e "${GREEN}Cleanup completed${NC}"
}

# Set up signal handlers
trap cleanup EXIT INT TERM

# Function to derive map path from bagfile path
derive_map_path() {
    local bagfile_path="$1"
    
    # If MAP_BASE_PATH is already set via command line, use it
    if [[ -n "$MAP_BASE_PATH" ]]; then
        return
    fi
    
    # Extract directory and filename
    local bag_dir=$(dirname "$bagfile_path")
    local bagfile_name=$(basename "$bagfile_path" .db3)
    
    # Check if path contains "/bags/" and derive accordingly
    if [[ "$bag_dir" == *"/bags/"* ]]; then
        # Extract base directory and dataset name
        local base_dir="${bag_dir%/bags/*}"
        local dataset_name="${bagfile_name%_*}"  # Remove suffix after underscore
        MAP_BASE_PATH="${base_dir}/${dataset_name}_map"
    else
        # Fallback: use same directory as bagfile
        MAP_BASE_PATH="${bag_dir}/${bagfile_name}_map"
    fi
}

# Main execution
main() {
    echo -e "${GREEN}=== Gaussian Splatting + Visualization Runner ===${NC}"
    
    # Derive map path from bagfile path
    derive_map_path "$ROSBAG_PATH"
    
    echo -e "${BLUE}Configuration:${NC}"
    echo "  Rosbag: $ROSBAG_PATH"
    echo "  Config: $CONFIG"
    echo "  Map path: $MAP_BASE_PATH (derived from bagfile path)"
    echo "  Container: $CONTAINER_NAME"
    echo ""
    
    # Validate inputs
    check_container
    
    if [[ ! -f "$ROSBAG_PATH" ]]; then
        echo -e "${RED}Error: Rosbag file not found: $ROSBAG_PATH${NC}"
        exit 1
    fi
    
    # Build required targets
    build_targets
    
    # Start Gaussian splat processor in background
    echo -e "${BLUE}Starting Gaussian splat processor...${NC}"
    docker exec -d "$CONTAINER_NAME" bash -c "cd /workspace && ./bazel-bin/gaussian_splatting/gs_processor '$MAP_BASE_PATH' > /tmp/gs_processor.log 2>&1"
    GS_PID=1  # Mark that we started it
    
    # Give GS processor time to initialize
    sleep 2
    
    # Check if GS processor started successfully
    if ! docker exec "$CONTAINER_NAME" bash -c "pgrep -f gs_processor > /dev/null"; then
        echo -e "${RED}Failed to start Gaussian splat processor${NC}"
        echo "Check logs:"
        docker exec "$CONTAINER_NAME" bash -c "tail -20 /tmp/gs_processor.log"
        exit 1
    fi
    
    echo -e "${GREEN}Gaussian splat processor started successfully${NC}"
    
    # Show GS processor initial logs
    echo -e "${BLUE}Gaussian splat processor logs:${NC}"
    docker exec "$CONTAINER_NAME" bash -c "head -10 /tmp/gs_processor.log"
    
    # Start ROS visualization (foreground process)
    echo -e "${BLUE}Starting ROS visualization...${NC}"
    echo "Command: bazel run //viz:visualize_rosbag -- $ROSBAG_PATH $CONFIG"
    echo ""
    
    # Run visualization in foreground - this will show all output
    docker exec "$CONTAINER_NAME" bash -c "cd /workspace && bazel run //viz:visualize_rosbag -- '$ROSBAG_PATH' '$CONFIG'" &
    VIZ_PID=$!
    
    # Monitor both processes
    echo -e "${YELLOW}Both processes started. Monitoring...${NC}"
    echo "Press Ctrl+C to stop both processes"
    echo ""
    
    # Show periodic status updates
    while true; do
        sleep 10
        
        # Check if GS processor is still running
        if docker exec "$CONTAINER_NAME" bash -c "pgrep -f gs_processor > /dev/null"; then
            GS_STATUS="${GREEN}RUNNING${NC}"
        else
            GS_STATUS="${RED}STOPPED${NC}"
        fi
        
        # Check if visualization is still running
        if kill -0 "$VIZ_PID" 2>/dev/null; then
            VIZ_STATUS="${GREEN}RUNNING${NC}"
        else
            VIZ_STATUS="${RED}STOPPED${NC}"
            break
        fi
        
        echo -e "Status: GS Processor: $GS_STATUS | Visualization: $VIZ_STATUS"
        
        # Show recent GS processor logs
        echo -e "${BLUE}Recent GS processor activity:${NC}"
        docker exec "$CONTAINER_NAME" bash -c "tail -3 /tmp/gs_processor.log"
        echo ""
    done
    
    echo -e "${YELLOW}Visualization process ended${NC}"
    wait "$VIZ_PID" 2>/dev/null || true
}

# Run main function
main "$@"
