# MASt3R Integration Guide

This guide explains how to integrate the MASt3R tracker into your SLAM pipeline.

## Overview

MASt3R (Matching and Stereo 3D Reconstruction) is a state-of-the-art deep learning model that provides:
- Dense 3D point cloud reconstruction
- Robust feature correspondences
- Direct 3D triangulation without explicit feature detection
- Better performance in low-texture and challenging environments

## Architecture

```
C++ GraphAdapter
    ↓
MASt3RTracker (C++)
    ↓ (pipes/JSON)
Python Inference Service
    ↓
MASt3R Model (PyTorch)
```

## Installation

### 1. Install MASt3R Python Dependencies

```bash
# Install MASt3R
pip install git+https://github.com/naver/mast3r.git

# Or install from source
git clone https://github.com/naver/mast3r.git
cd mast3r
pip install -e .
```

### 2. Download Model Weights

The model weights will be automatically downloaded from HuggingFace on first use. Alternatively:

```bash
# Download manually
mkdir -p ~/.cache/huggingface/hub
cd ~/.cache/huggingface/hub
huggingface-cli download naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric
```

### 3. Verify Installation

```bash
python3 tracking/2d/mast3r_inference_service.py --model naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric --device cuda
# Should start and wait for input on stdin
# Press Ctrl+C to exit
```

## Integration into graph_adapter.cpp

### Option 1: Replace ORB Tracker (Recommended for Testing)

Edit `core/graph/include/core/graph/graph_adapter.hpp`:

```cpp
// Add include
#include "2d/mast3r_tracker.hpp"

// In GraphAdapter class, replace:
// tracking::image::OrbTracker orb_tracker_;
// with:
tracking::image::MASt3RTracker mast3r_tracker_;
```

Edit `core/graph/src/graph_adapter.cpp`:

```cpp
// In constructor, initialize MASt3R tracker:
MASt3RTracker(
    "naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric",  // model path
    "python3",                                                   // python executable
    "cuda",                                                      // device
    10000                                                        // max points
)

// Update setFrameIds method:
void GraphAdapter::setFrameIds(const std::string& base_link_frame_id,
                               const std::string& camera_frame_id) {
    base_link_frame_id_ = base_link_frame_id;
    camera_frame_id_ = camera_frame_id;
    mast3r_tracker_.setBaseFrameId(base_link_frame_id_);
}

// Update line 292 in addKeyframeToGraph:
// Replace:
// auto pose = orb_tracker_.match(*previous_frame_for_orb, *current_frame_for_orb, world_points);
// With:
auto pose = mast3r_tracker_.match(*previous_frame_for_orb, *current_frame_for_orb, world_points);
```

### Option 2: Hybrid Approach (Use Both)

Keep ORB for fast tracking and use MASt3R for dense reconstruction:

```cpp
// In graph_adapter.hpp:
tracking::image::OrbTracker orb_tracker_;
tracking::image::MASt3RTracker mast3r_tracker_;
bool use_mast3r_for_dense_reconstruction_ = true;

// In graph_adapter.cpp:
// Use ORB for tracking
auto pose = orb_tracker_.match(*previous_frame_for_orb, *current_frame_for_orb, world_points);

// Optionally use MASt3R for dense reconstruction every N frames
if (use_mast3r_for_dense_reconstruction_ && current_keyframe_id_ % 10 == 0) {
    std::vector<Eigen::Vector3d> dense_points;
    mast3r_tracker_.match(*previous_frame_for_orb, *current_frame_for_orb, dense_points);
    // Merge dense_points into world_points or map_keypoints
}
```

### Option 3: Configuration-Based Selection

```cpp
enum class TrackerType {
    ORB,
    MAST3R,
    SIFT
};

TrackerType tracker_type_ = TrackerType::MAST3R;

// In addKeyframeToGraph:
switch (tracker_type_) {
    case TrackerType::ORB:
        pose = orb_tracker_.match(...);
        break;
    case TrackerType::MAST3R:
        pose = mast3r_tracker_.match(...);
        break;
    case TrackerType::SIFT:
        pose = sift_tracker_.match(...);
        break;
}
```

## Configuration

### MASt3R Tracker Parameters

```cpp
// In constructor or configuration:
mast3r_tracker_.setConfidenceThreshold(0.5);  // Filter low-confidence matches
mast3r_tracker_.setSubsampling(true, 8);      // Subsample for performance
```

### Performance Tuning

**For Real-Time Performance:**
```cpp
MASt3RTracker(
    "naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric",
    "python3",
    "cuda",
    5000  // Reduce max points
)
mast3r_tracker_.setSubsampling(true, 16);  // Increase subsampling
```

**For Maximum Quality:**
```cpp
MASt3RTracker(
    "naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric",
    "python3",
    "cuda",
    50000  // Increase max points
)
mast3r_tracker_.setSubsampling(false);  // No subsampling
mast3r_tracker_.setConfidenceThreshold(0.3);  // Lower threshold for more points
```

## Building

```bash
cd /home/sree/Matrix/shadesmar
docker exec docker-dev-1 bash -c "bazel build //tracking:tracking"
```

## Troubleshooting

### Python Service Not Starting

Check that Python and MASt3R are installed:
```bash
python3 -c "import mast3r; print('MASt3R installed successfully')"
```

Check logs in stderr:
```bash
# The Python service writes to stderr, check container logs
docker logs docker-dev-1
```

### CUDA Out of Memory

Reduce batch size or image resolution:
```python
# In mast3r_inference_service.py, reduce image_size:
self.image_size = 256  # Default is 512
```

### Slow Performance

1. Ensure CUDA is available:
```bash
python3 -c "import torch; print(torch.cuda.is_available())"
```

2. Use CPU if needed (slower but works):
```cpp
MASt3RTracker(..., "cpu", ...)
```

3. Increase subsampling factor:
```cpp
mast3r_tracker_.setSubsampling(true, 16);  // or 32
```

### Communication Errors

If you see "Failed to read response from Python service":
- Check that the Python script path is correct
- Verify pipes are not broken
- Check for Python exceptions in stderr logs

## Advanced Usage

### Custom Map Keypoint Creation

Use `matchAndCreateKeypoints()` for full integration:

```cpp
std::map<uint32_t, core::types::Keypoint> map_keypoints;
auto pose = mast3r_tracker_.matchAndCreateKeypoints(
    *previous_frame,
    *current_frame,
    map_keypoints
);

// map_keypoints now contains proper Keypoint structures with:
// - 3D positions
// - Observations in both frames
// - Ready for optimization
```

### Dense Reconstruction Mode

For dense mapping applications:

```cpp
mast3r_tracker_.setSubsampling(false);  // Get all correspondences
mast3r_tracker_.setConfidenceThreshold(0.2);  // Accept more points

auto pose = mast3r_tracker_.match(...);
// world_points will contain dense reconstruction
```

## Comparison with ORB

| Feature | ORB | MASt3R |
|---------|-----|--------|
| Speed | Fast (~10-30ms) | Slower (~100-500ms) |
| Point Density | Sparse (100-500 points) | Dense (1000-10000+ points) |
| Low Texture | Poor | Excellent |
| Lighting Changes | Moderate | Excellent |
| GPU Required | No | Yes (recommended) |
| Memory Usage | Low | High |
| Initialization | None | Model loading (~5s) |

## Best Practices

1. **Use MASt3R for keyframe selection** rather than every frame
2. **Combine with ORB**: Use ORB for fast tracking, MASt3R for dense mapping
3. **Monitor performance**: Check inference times and adjust subsampling
4. **Handle failures gracefully**: MASt3R may fail on motion blur or extreme motion
5. **Cache results**: MASt3R results can be reused for multiple purposes

## Example Integration

See the complete example in `core/graph/src/graph_adapter.cpp` around line 292.

## References

- MASt3R Paper: https://arxiv.org/abs/2406.09756
- GitHub: https://github.com/naver/mast3r
- HuggingFace Model: https://huggingface.co/naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric
