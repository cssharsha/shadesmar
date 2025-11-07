.PHONY: help check-setup build install-foxglove-deps build-foxglove package-extension build-extension-full build-colmap-converter build-colmap-viz run run-no-spatial run-colmap-converter run-colmap-viz rerun shell clean test-connection docker-build docker-rebuild docker-up docker-down docker-restart

# Default target
help:
	@echo "Gaussian Splatting with Foxglove - Makefile"
	@echo ""
	@echo "Prerequisites:"
	@echo "  - docker-dev-1 container must be running"
	@echo "  - Container initialization (.devcontainer/setup.sh) must be complete"
	@echo ""
	@echo "Available targets:"
	@echo ""
	@echo "Setup & Build:"
	@echo "  make check-setup          - Verify container is initialized (checks for zsh)"
	@echo "  make build                - Build streaming_gs_processor_main with CUDA"
	@echo "  make install-foxglove-deps- Install Foxglove extension dependencies (THREE.js, etc.)"
	@echo "  make build-foxglove       - Build Foxglove extension"
	@echo "  make package-extension    - Package Foxglove extension (.foxe file)"
	@echo "  make build-extension-full - Full extension build (deps + build + package)"
	@echo "  make build-colmap-converter - Build COLMAP converter tool"
	@echo "  make build-colmap-viz     - Build COLMAP visualization tool"
	@echo ""
	@echo "Run:"
	@echo "  make run MAP=<path>       - Run processor with map path"
	@echo "  make run-no-spatial MAP=<path> - Run without spatial partitioning"
	@echo "  make run-with-foxglove MAP=<path> - Run with Foxglove enabled (default)"
	@echo "  make run-colmap-converter COLMAP_PATH=<path> - Convert COLMAP reconstruction to map"
	@echo "  make run-colmap-viz MAP_PATH=<path> - Visualize COLMAP data with Rerun"
	@echo ""
	@echo "Utilities:"
	@echo "  make rerun                - Start Rerun web viewer (kills existing instance)"
	@echo "  make shell                - Open interactive zsh shell in container"
	@echo "  make clean                - Clean build artifacts"
	@echo "  make test-connection      - Test if Foxglove WebSocket is accessible"
	@echo ""
	@echo "Docker Management:"
	@echo "  make docker-build         - Build Docker image from Dockerfile"
	@echo "  make docker-rebuild       - Rebuild Docker image (no cache)"
	@echo "  make docker-up            - Start the container"
	@echo "  make docker-down          - Stop and remove the container"
	@echo "  make docker-restart       - Full restart (down + rebuild + up)"
	@echo ""
	@echo "Examples:"
	@echo "  make build"
	@echo "  make run MAP=/data/robot/house11_map"
	@echo "  make run-no-spatial MAP=/data/robot/house11_map"
	@echo "  make run MAP=/data/robot/house11_map FOXGLOVE_HOST=0.0.0.0 FOXGLOVE_PORT=8765"
	@echo "  make run MAP=/data/robot/house11_map ARGS='--disable-foxglove'"
	@echo "  make build-colmap-converter && make run-colmap-converter COLMAP_PATH=/data/train"
	@echo "  make build-colmap-viz && make run-colmap-viz MAP_PATH=/data/train"
	@echo "  make docker-restart  # After modifying Dockerfile"
	@echo ""
	@echo "Configuration variables:"
	@echo "  MAP               - Map base path (required for run targets)"
	@echo "  ARGS              - Additional arguments to pass to binary"
	@echo "  FOXGLOVE_HOST     - Foxglove host (default: 0.0.0.0)"
	@echo "  FOXGLOVE_PORT     - Foxglove port (default: 8765)"
	@echo "  SPATIAL_PARTITION - Enable spatial partitioning (default: true)"
	@echo "  COLMAP_PATH       - Path to COLMAP reconstruction (for colmap_converter)"
	@echo "  MAP_PATH          - Path to map data (for colmap_viz)"

# Container and workspace configuration
CONTAINER_NAME := docker-dev-1
WORKSPACE_DIR := /workspace

# Default configuration
FOXGLOVE_HOST ?= 0.0.0.0
FOXGLOVE_PORT ?= 8765
SPATIAL_PARTITION ?= true

# Check if container is fully initialized by verifying zsh is available
check-setup:
	@echo "Checking if container '$(CONTAINER_NAME)' is initialized..."
	@docker ps -q -f name=$(CONTAINER_NAME) > /dev/null 2>&1 || \
		(echo "ERROR: Container '$(CONTAINER_NAME)' is not running." && \
		 echo "Start the container first." && exit 1)
	@docker exec $(CONTAINER_NAME) which zsh > /dev/null 2>&1 || \
		(echo "ERROR: Container not fully initialized." && \
		 echo "The .devcontainer/setup.sh script hasn't completed yet." && \
		 echo "Wait for container initialization to finish." && exit 1)
	@echo "✓ Container is initialized and ready"

# Build the main streaming processor with CUDA
build: check-setup
	@echo "Building streaming_gs_processor_main with CUDA support..."
	docker exec $(CONTAINER_NAME) zsh -c "cd $(WORKSPACE_DIR) && \
		bazel build --config=cuda //gaussian_splatting:streaming_gs_processor_main"
	@echo "✓ Build complete: ./bazel-bin/gaussian_splatting/streaming_gs_processor_main"

# Install Foxglove extension dependencies (including THREE.js)
install-foxglove-deps: check-setup
	@echo "Installing Foxglove extension dependencies..."
	docker exec $(CONTAINER_NAME) zsh -c "cd $(WORKSPACE_DIR)/foxglove-extension && \
		npm install"
	@echo "✓ Dependencies installed"

# Build Foxglove extension
build-foxglove: check-setup
	@echo "Building Foxglove extension..."
	docker exec $(CONTAINER_NAME) zsh -c "cd $(WORKSPACE_DIR)/foxglove-extension && \
		npm run build"
	@echo "✓ Foxglove extension built (dist/)"

# Package Foxglove extension as .foxe file
package-extension: check-setup
	@echo "Packaging Foxglove extension..."
	docker exec $(CONTAINER_NAME) zsh -c "cd $(WORKSPACE_DIR)/foxglove-extension && \
		npm run package"
	@echo "✓ Extension packaged: foxglove-extension/shadesmar.gaussian-splat-renderer-1.0.0.foxe"

# Full build of extension (install deps + build + package)
build-extension-full: install-foxglove-deps build-foxglove package-extension
	@echo "✓ Extension fully built and packaged"

# Run the processor (basic version)
run: check-setup
ifndef MAP
	@echo "ERROR: MAP variable is required"
	@echo "Usage: make run MAP=/path/to/map [ARGS='--additional-args']"
	@exit 1
endif
	@echo "Running streaming_gs_processor_main..."
	@echo "Map path: $(MAP)"
	docker exec -it $(CONTAINER_NAME) zsh -c "source ~/.cargo/env && cd $(WORKSPACE_DIR) && \
		./bazel-bin/gaussian_splatting/streaming_gs_processor_main $(MAP) $(ARGS)"

# Run without spatial partitioning
run-no-spatial: check-setup
ifndef MAP
	@echo "ERROR: MAP variable is required"
	@echo "Usage: make run-no-spatial MAP=/path/to/map [ARGS='--additional-args']"
	@exit 1
endif
	@echo "Running streaming_gs_processor_main WITHOUT spatial partitioning..."
	@echo "Map path: $(MAP)"
	docker exec -it $(CONTAINER_NAME) zsh -c "source ~/.cargo/env && cd $(WORKSPACE_DIR) && \
		./bazel-bin/gaussian_splatting/streaming_gs_processor_main $(MAP) --no-spatial-partitioning $(ARGS)"

# Run with Foxglove configuration (explicit)
run-with-foxglove: check-setup
ifndef MAP
	@echo "ERROR: MAP variable is required"
	@echo "Usage: make run-with-foxglove MAP=/path/to/map"
	@exit 1
endif
	@echo "Running with Foxglove enabled..."
	@echo "Map path: $(MAP)"
	@echo "Foxglove host: $(FOXGLOVE_HOST)"
	@echo "Foxglove port: $(FOXGLOVE_PORT)"
	@echo "Spatial partitioning: $(SPATIAL_PARTITION)"
	@echo ""
	@echo "Connect Foxglove Studio to: ws://localhost:$(FOXGLOVE_PORT)"
	@echo ""
	docker exec -it $(CONTAINER_NAME) zsh -c "source ~/.cargo/env && cd $(WORKSPACE_DIR) && \
		./bazel-bin/gaussian_splatting/streaming_gs_processor_main $(MAP) \
		--enable-foxglove \
		--foxglove-host $(FOXGLOVE_HOST) \
		--foxglove-port $(FOXGLOVE_PORT) \
		$(if $(filter false,$(SPATIAL_PARTITION)),--no-spatial-partitioning,--spatial-partitioning) \
		$(ARGS)"

# Open interactive shell in container
shell: check-setup
	@echo "Opening zsh shell in container..."
	docker exec -it $(CONTAINER_NAME) zsh -c "cd $(WORKSPACE_DIR) && exec zsh"

# Clean build artifacts
clean: check-setup
	@echo "Cleaning build artifacts..."
	docker exec $(CONTAINER_NAME) zsh -c "cd $(WORKSPACE_DIR) && \
		bazel clean --expunge"
	@echo "✓ Build artifacts cleaned"

# Test if Foxglove WebSocket port is accessible
test-connection:
	@echo "Testing Foxglove WebSocket connection on port $(FOXGLOVE_PORT)..."
	@docker exec $(CONTAINER_NAME) zsh -c "command -v netstat > /dev/null 2>&1" && \
		docker exec $(CONTAINER_NAME) zsh -c "netstat -tulpn | grep $(FOXGLOVE_PORT)" || \
		echo "No process listening on port $(FOXGLOVE_PORT)"
	@echo ""
	@echo "If backend is running, connect Foxglove Studio to:"
	@echo "  ws://localhost:$(FOXGLOVE_PORT)"

# Start Rerun web viewer (kills existing instance if running)
rerun: check-setup
	@echo "Checking for existing Rerun instance..."
	-@docker exec $(CONTAINER_NAME) zsh -c "pkill -f 'rerun --web-viewer' 2>/dev/null" || true
	@sleep 0.5
	@echo "Starting Rerun web viewer in background..."
	@docker exec $(CONTAINER_NAME) zsh -c "source ~/.cargo/env && cd $(WORKSPACE_DIR) && nohup rerun --web-viewer > /tmp/rerun.log 2>&1 &"
	@sleep 1
	@echo "✓ Rerun web viewer started"
	@echo "Access at: http://localhost:9876 (or check rerun output for actual port)"
	@echo "Logs: docker exec $(CONTAINER_NAME) cat /tmp/rerun.log"

# ============================================================================
# COLMAP Processor Targets
# ============================================================================

# Build COLMAP converter (converts COLMAP reconstruction to map format)
build-colmap-converter: check-setup
	@echo "Building COLMAP converter with CUDA support..."
	docker exec $(CONTAINER_NAME) zsh -c "cd $(WORKSPACE_DIR) && \
		bazel build --config=cuda //colmap_processor:cconverter"
	@echo "✓ Build complete: ./bazel-bin/colmap_processor/cconverter"

# Build COLMAP visualization tool
build-colmap-viz: check-setup
	@echo "Building COLMAP visualization tool with CUDA support..."
	docker exec $(CONTAINER_NAME) zsh -c "cd $(WORKSPACE_DIR) && \
		bazel build --config=cuda //colmap_processor:colmap_viz_main"
	@echo "✓ Build complete: ./bazel-bin/colmap_processor/colmap_viz_main"

# Run COLMAP converter
run-colmap-converter: check-setup
ifndef COLMAP_PATH
	@echo "ERROR: COLMAP_PATH variable is required"
	@echo "Usage: make run-colmap-converter COLMAP_PATH=/path/to/colmap/reconstruction"
	@echo "Example: make run-colmap-converter COLMAP_PATH=/data/train"
	@exit 1
endif
	@echo "Running COLMAP converter..."
	@echo "COLMAP path: $(COLMAP_PATH)"
	docker exec -it $(CONTAINER_NAME) zsh -c "cd $(WORKSPACE_DIR) && \
		./bazel-bin/colmap_processor/cconverter $(COLMAP_PATH)"

# Run COLMAP visualization (Rerun visualizer)
run-colmap-viz: check-setup
ifndef MAP_PATH
	@echo "ERROR: MAP_PATH variable is required"
	@echo "Usage: make run-colmap-viz MAP_PATH=/path/to/map"
	@echo "Example: make run-colmap-viz MAP_PATH=/data/train"
	@exit 1
endif
	@echo "Running COLMAP visualization with Rerun..."
	@echo "Map path: $(MAP_PATH)"
	@echo "Press Ctrl+C to exit"
	docker exec -it $(CONTAINER_NAME) zsh -c "source ~/.cargo/env && cd $(WORKSPACE_DIR) && \
		./bazel-bin/colmap_processor/colmap_viz_main $(MAP_PATH)"

# ============================================================================
# Docker Management Commands
# ============================================================================

COMPOSE_FILE := dev/docker/docker-compose.yml

# Build Docker image (uses cache)
docker-build:
	@echo "Building Docker image..."
	docker-compose -f $(COMPOSE_FILE) build
	@echo "✓ Docker image built"

# Rebuild Docker image (no cache, force rebuild)
docker-rebuild:
	@echo "Rebuilding Docker image (no cache)..."
	docker-compose -f $(COMPOSE_FILE) build --no-cache
	@echo "✓ Docker image rebuilt"

# Start the container
docker-up:
	@echo "Starting container..."
	docker-compose -f $(COMPOSE_FILE) up -d
	@echo "✓ Container started"
	@echo "Waiting for container to be ready..."
	@sleep 3
	@echo "Running setup.sh (this will take a few minutes)..."
	@docker exec $(CONTAINER_NAME) bash -c "cd /workspace && bash ./.devcontainer/setup.sh" || \
		echo "Setup.sh may have already run or encountered an error"
	@echo "✓ Setup complete"
	@echo "Verifying zsh is available..."
	@docker exec $(CONTAINER_NAME) which zsh > /dev/null 2>&1 && \
		echo "✓ Container is ready" || \
		echo "⚠ Warning: zsh not found, setup may not be complete"

# Stop and remove the container
docker-down:
	@echo "Stopping and removing container..."
	docker-compose -f $(COMPOSE_FILE) down
	@echo "✓ Container stopped and removed"

# Full restart: stop, rebuild, and start
docker-restart:
	@echo "Full Docker restart (rebuild + restart)..."
	@echo "This will take several minutes (rebuilding image + running setup.sh)"
	@echo ""
	$(MAKE) docker-down
	$(MAKE) docker-rebuild
	$(MAKE) docker-up
	@echo ""
	@echo "✓ Docker restart complete!"
	@echo ""
	@echo "Verify with: make check-setup"
	@echo "Check npm: docker exec $(CONTAINER_NAME) zsh -c 'node --version && npm --version'"
