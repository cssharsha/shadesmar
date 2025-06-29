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

/**
 * Manages loading and batching of keyframes for training
 */
class KeyframeBatchLoader {
public:
    explicit KeyframeBatchLoader(std::shared_ptr<core::storage::MapStore> map_store,
                                 const TrainingConfig& config, torch::Device device = torch::kCPU);

    ~KeyframeBatchLoader() = default;

    // Initialize loader with available keyframes
    bool initialize();

    // Load a specific batch of keyframes
    bool loadBatch(const std::vector<uint64_t>& keyframe_ids, KeyframeBatch& batch);

    // Get next batch in training sequence
    bool getNextBatch(KeyframeBatch& batch);

    // Reset batch iterator to beginning
    void resetBatchIterator();

    // Shuffle the keyframe order for next epoch
    void shuffleKeyframes();

    // Get total number of available keyframes
    size_t getTotalKeyframes() const {
        return available_keyframes_.size();
    }

    // Maybe need to actually do a sliding window rather than a batch
    size_t getTotalBatches() const;

    // Check if more batches are available
    bool hasMoreBatches() const;

    // Get current batch index
    size_t getCurrentBatchIndex() const {
        return current_batch_index_;
    }

    // Configuration accessors
    const TrainingConfig& getConfig() const {
        return config_;
    }
    torch::Device getDevice() const {
        return device_;
    }

private:
    // Core dependencies
    std::shared_ptr<core::storage::MapStore> map_store_;
    TrainingConfig config_;
    torch::Device device_;

    // Keyframe management
    std::vector<uint64_t> available_keyframes_;
    std::vector<uint64_t> shuffled_keyframes_;
    size_t current_batch_index_;
    uint32_t next_batch_id_;

    // Cache for loaded keyframes to avoid repeated disk reads
    std::unordered_map<uint64_t, core::types::KeyFrame::Ptr> keyframe_cache_;
    size_t max_cache_size_;

    // Helper methods
    bool loadAvailableKeyframes();
    bool validateKeyframe(const core::types::KeyFrame::Ptr& keyframe) const;
    bool convertKeyframesToTensors(const std::vector<core::types::KeyFrame::Ptr>& keyframes,
                                   KeyframeBatch& batch);
    torch::Tensor extractCameraPose(const core::types::KeyFrame::Ptr& keyframe);
    torch::Tensor extractCameraIntrinsics(const core::types::KeyFrame::Ptr& keyframe);
    torch::Tensor convertImageToTensor(const core::types::Image& image);
    void updateKeyframeCache(uint64_t keyframe_id, core::types::KeyFrame::Ptr keyframe);
    void pruneKeyframeCache();
};

/**
 * Training batch iterator that provides sequential and random access to batches
 */
class TrainingBatchIterator {
public:
    explicit TrainingBatchIterator(std::shared_ptr<KeyframeBatchLoader> loader);

    // Iterator interface
    bool hasNext() const;
    bool getNext(KeyframeBatch& batch);
    void reset();

    // Epoch management
    void startNewEpoch();
    size_t getCurrentEpoch() const {
        return current_epoch_;
    }

    // Batch access
    size_t getTotalBatches() const;
    size_t getCurrentBatchInEpoch() const;

    // Statistics
    struct IteratorStats {
        size_t total_epochs;
        size_t total_batches_processed;
        size_t current_epoch;
        size_t current_batch_in_epoch;
        double avg_batch_load_time_ms;
    };

    IteratorStats getStats() const {
        return stats_;
    }

private:
    std::shared_ptr<KeyframeBatchLoader> loader_;
    size_t current_epoch_;
    IteratorStats stats_;

    // Timing for performance monitoring
    std::chrono::high_resolution_clock::time_point last_batch_start_;
    void updateLoadTime();
};

}  // namespace training
}  // namespace gaussian_splatting
