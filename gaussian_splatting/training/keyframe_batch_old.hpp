#pragma once

#include <torch/torch.h>
#include <cmath>
#include <memory>
#include <sstream>
#include <vector>
#include "core/storage/map_store.hpp"
#include "core/types/gaussian_splat.hpp"
#include "core/types/keyframe.hpp"
#include "training_config.hpp"

namespace gaussian_splatting {
namespace training {

/**
 * Represents a batch of keyframes for training with associated camera data
 */
struct KeyframeBatch {
    // Keyframe identifiers and metadata
    std::vector<uint64_t> keyframe_ids;
    std::vector<core::types::KeyFrame::Ptr> keyframes;

    // Training data tensors
    torch::Tensor images;             // [batch_size, 3, height, width]
    torch::Tensor camera_poses;       // [batch_size, 4, 4] - world to camera transforms
    torch::Tensor camera_intrinsics;  // [batch_size, 3, 3] - camera intrinsic matrices
    torch::Tensor depths;             // [batch_size, height, width] - optional depth maps

    // Batch metadata
    uint32_t batch_id;
    size_t batch_size;
    int image_height;
    int image_width;
    torch::Device device;

    KeyframeBatch()
        : batch_id(0), batch_size(0), image_height(0), image_width(0), device(torch::kCPU) {}

    bool isValid() const;

    void print();

    void clear() {
        keyframe_ids.clear();
        keyframes.clear();
        images = torch::Tensor();
        camera_poses = torch::Tensor();
        camera_intrinsics = torch::Tensor();
        depths = torch::Tensor();
        batch_size = 0;
    }

    // Move batch to specified device
    void to(torch::Device target_device) {
        if (device == target_device)
            return;

        images = images.to(target_device);
        camera_poses = camera_poses.to(target_device);
        camera_intrinsics = camera_intrinsics.to(target_device);
        if (depths.defined()) {
            depths = depths.to(target_device);
        }
        device = target_device;
    }
};

}  // namespace training
}  // namespace gaussian_splatting
