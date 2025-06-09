# Shadesmar - Visual SLAM with Bundle Adjustment

A modern C++ visual SLAM system featuring GTSAM-based bundle adjustment, protobuf serialization, and real-time visualization capabilities.

## Getting Started

### Prerequisites

- Docker and Docker Compose
- Bazel build system
- CUDA-compatible GPU (optional, for acceleration)

### Build and Run

#### Using Docker (Recommended)

1. **Build the Docker environment:**
```bash
cd dev/docker
docker-compose build
```

2. **Run with proper volume mounts:**
```bash
# Mount your dataset directory to /data/robot/
docker-compose run --rm \
  -v /path/to/your/dataset:/data/robot/ \
  -v /path/to/your/bags:/data/robot/bags/ \
  shadesmar
```

**Important:** The `/data/robot/` volume mount is critical as all outputs (trajectories, maps, visualizations) are written to this directory.

#### Local Build

```bash
# Install dependencies
./dev/scripts/setup.sh

# Build the project
bazel build //...

# Run tests
bazel test //...

# Format and lint (optional)
./dev/scripts/check_all.sh
```

### Quick Start Examples

```bash
# Process KITTI dataset
bazel run //kitti/examples:kitti_example -- /data/robot/kitti/

# Process ROS bag with visualization (main interface)
bazel run //viz:visualize_rosbag -- /data/robot/bags/house11/house11_0.db3 default 2>&1 | tee robot.log

# Live visualization
bazel run //viz/examples:visualize_live
```

### Working with Docker

After starting the Docker container, attach to it for interactive development:

```bash
# Attach to running container
docker exec -it zsh docker-dev-1
```

## Architecture

Shadesmar implements a modular visual SLAM pipeline with the following key components:

### Core Modules

#### Factor Graph (`core/graph/`)
- **GTSAM Backend**: Uses Georgia Tech's GTSAM library for factor graph optimization
- **Batch Optimization**: Processes keyframes in batches every N frames (configurable, default: 10)
- **Factor Types**: Supports odometry, loop closure, IMU preintegration, and reprojection factors
- **Threading Architecture**: Asynchronous optimization thread with lock-free coordination
- **Protobuf Serialization**: All factors serialized using `factor.proto` for persistence

#### Storage System (`core/storage/`)
- **Three-Queue Architecture**: Unprocessed → Processed Non-Optimized → Processed Optimized
- **Callback-Based Writes**: Non-blocking disk operations with automatic index generation
- **Map Persistence**: Complete map state serialization with Apache Arrow backend
- **Memory Management**: Automatic eviction of old map points and keyframes

**Map Data Storage:**
- **Keyframes**: Camera poses, RGB images, depth data (optional), camera intrinsics, timestamps
- **Map Keypoints**: 3D landmark positions, multi-view observations with pixel coordinates, descriptor data
- **Factor Graph**: Odometry constraints, loop closure factors, IMU preintegration, reprojection factors
- **Metadata**: Coordinate frame relationships, calibration parameters, optimization statistics
- **Index Files**: Fast lookup tables for keyframes, map points, and temporal queries

#### Tracking (`tracking/`)
- **ORB Feature Detection**: OpenCV-based ORB with configurable parameters
- **Triangulation Options**: 
  - Direct triangulation (current)
  - GTSAM-based triangulation (planned)
  - Multi-view robust triangulation
- **Visual Odometry**: Support for pose-free datasets (TUM format)
- **Keyframe Management**: Intelligent keyframe selection based on motion and feature distribution

#### Visualization (`viz/`)
- **Rerun Integration**: Real-time 3D visualization with Rerun SDK
- **Factor Graph Export**: VTK format output for ParaView/VisIt analysis
- **Debug Outputs**: Ray-based reprojection error visualization
- **Live Streaming**: WebRTC-based remote visualization support

### Data Flow

```
Sensors → Message Sync → Keyframe Creation → ORB Processing → Factor Graph → Bundle Adjustment → Map Storage
                                                    ↓
                                              Visualization ← Real-time Updates
```

### Threading Model

- **Main Thread**: Sensor input processing and keyframe creation
- **ORB Thread**: Feature detection and matching (2-frame delay)
- **Optimization Thread**: Asynchronous bundle adjustment every N keyframes
- **Storage Thread**: Background disk writes with callback coordination
- **Visualization Thread**: Real-time rendering and export

## Module Details

### Core Types (`core/types/`)
- **Keyframe Structure**: Pose, image data, camera info, and depth (optional)
- **Map Keypoints**: 3D landmarks with multi-view observations
- **Protobuf Messages**: Serialized geometry, IMU data, and sensor information
- **Thread-Safe Containers**: Lock-free data structures for high-throughput processing

### ROS Integration (`ros/`)
- **Main Interface**: `visualize_rosbag` - Primary entry point for ROS bag processing with live visualization
- **Bag Reader**: Processes ROS 1/2 bag files with timestamp synchronization
- **Transform Integration**: Uses ROS TF2 for coordinate frame management
- **Topic Mapping**: Configurable topic names for different robot platforms
- **Protobuf Conversion**: Automatic conversion between ROS and internal messages

### KITTI Support (`kitti/`)
- **Dataset Reader**: Native KITTI format support with calibration loading
- **Ground Truth**: Comparison with KITTI odometry ground truth
- **Conversion Utilities**: Transform KITTI coordinates to SLAM coordinate frames
- **Benchmark Integration**: Automatic trajectory evaluation and metrics

### Utilities (`utils/`)
- **Message Synchronizer**: Time-based sensor data alignment
- **Transform Tree**: Spatial relationship management between coordinate frames
- **Protobuf Utilities**: Serialization helpers and message validation

## Protobuf Serialization

The system uses Protocol Buffers for all data persistence:

- **Sensor Data**: `sensor_data.proto` - Camera images, IMU, lidar
- **Geometry**: `geometry.proto` - Poses, transforms, coordinate frames
- **Factor Graph**: `factor.proto` - All factor types with measurements
- **Map Storage**: `map_storage_index.proto` - Persistent map indexing
- **Keyframes**: `keyframe.proto` - Complete keyframe state

## Rerun Visualization

Real-time visualization powered by Rerun SDK:

- **3D Scene**: Live trajectory, map points, and camera poses
- **Factor Graph**: Interactive factor visualization with error metrics
- **Debug Views**: Reprojection errors, feature matches, optimization convergence
- **Remote Access**: Web-based visualization for headless systems
- **Export Options**: VTK, PLY, and custom formats supported

## TODO

### Critical Fixes
- [ ] **Optimization System**: GTSAM bundle adjustment currently broken, implementing robust error handling
- [ ] **Triangulation Verification**: Add reprojection error validation for newly triangulated points
- [ ] **Memory Leaks**: Fix potential memory issues in long-running sequences

### Planned Features

#### Gaussian Splatting Integration
- [ ] **3D Gaussian Representation**: Convert triangulated map points to 3D Gaussians
- [ ] **Differentiable Rendering**: CUDA-based splatting renderer for photometric optimization
- [ ] **SLAM Prior**: Use existing VSLAM map as initialization for Gaussian parameters
- [ ] **Joint Optimization**: Combine geometric SLAM factors with photometric splatting loss
- [ ] **Real-time Rendering**: Live Gaussian splatting visualization in Rerun

#### Advanced SLAM Features
- [ ] **Loop Closure Detection**: DBoW-based place recognition
- [ ] **Semantic Segmentation**: Object-aware SLAM with semantic factors
- [ ] **Multi-Session Mapping**: Map merging and lifelong SLAM
- [ ] **Dynamic Objects**: Detection and handling of moving objects
- [ ] **Robust Backend**: Switchable constraints and outlier rejection

#### Performance Optimizations
- [ ] **GPU Acceleration**: CUDA kernels for feature matching and triangulation
- [ ] **Incremental Updates**: Avoid full map recomputation in optimization
- [ ] **Adaptive Keyframing**: ML-based keyframe selection
- [ ] **Compressed Storage**: LZ4/Zstd compression for large datasets

#### Integration & Deployment
- [ ] **ROS 2 Native**: First-class ROS 2 support with lifecycle management

## Contributing

1. Follow Google C++ Style Guide
2. Add tests for new features
3. Update protobuf schemas for data changes
4. Document new modules in this README
5. Test with provided datasets before submitting

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE)
file for details.
