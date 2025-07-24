#pragma once

#include <torch/torch.h>
#include <memory>
#include "../training/training_config.hpp"
#include "core/types/gaussian_splat.hpp"
#include "gaussian_splatting/training/gaussian_tensors.hpp"
#include "gaussian_splatting/training/keyframe_batch.hpp"

namespace gaussian_splatting {
namespace utils {

class BatchGPUManager {
public:
    explicit BatchGPUManager(const training::TrainingConfig& config);
    ~BatchGPUManager();

    // Load batch to GPU
    bool loadBatchToTensors(const core::types::GaussianSplatBatch& batch,
                            GaussianTensors& gaussian_tensors);

    // Memory management
    size_t getCurrentGPUMemoryUsage();
    bool hasEnoughMemoryForBatch(size_t estimated_batch_size);
    void clearBatchFromGPU(GaussianTensors& gaussian_tensors);
    bool loadToGPU(GaussianTensors& gaussian_tensors);
    bool loadToGPU(training::KeyframeBatch& batch);
    // Device management
    torch::Device getDevice() const {
        return device_;
    }
    bool isDeviceAvailable() const;

private:
    training::TrainingConfig config_;
    torch::Device device_;
};

}  // namespace utils
}  // namespace gaussian_splatting
