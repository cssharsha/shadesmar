#pragma once

#include <torch/torch.h>
#include <memory>
#include "core/types/gaussian_splat.hpp"
#include "../training/training_config.hpp"

namespace gaussian_splatting {
namespace utils {

// GPU data structure for a batch
struct GPUBatchData {
    torch::Tensor positions;      // [N, 3]
    torch::Tensor rotations;      // [N, 4] quaternions
    torch::Tensor scales;         // [N, 3]
    torch::Tensor opacities;      // [N, 1]
    torch::Tensor sh_coeffs;      // [N, SH_DEGREE^2, 3]
    torch::Tensor colors;         // [N, 3] fallback colors
    
    int num_gaussians = 0;
    uint32_t batch_id = 0;
    torch::Device device = torch::kCPU;
    
    bool isValid() const {
        return num_gaussians > 0 && positions.defined() && 
               rotations.defined() && scales.defined() && opacities.defined();
    }
    
    void clear() {
        positions = torch::Tensor();
        rotations = torch::Tensor();
        scales = torch::Tensor();
        opacities = torch::Tensor();
        sh_coeffs = torch::Tensor();
        colors = torch::Tensor();
        num_gaussians = 0;
    }
};

class BatchGPUManager {
public:
    explicit BatchGPUManager(const training::TrainingConfig& config);
    ~BatchGPUManager();
    
    // Load batch to GPU
    bool loadBatchToGPU(const core::types::GaussianSplatBatch& batch, GPUBatchData& gpu_data);
    
    // Memory management
    size_t getCurrentGPUMemoryUsage();
    bool hasEnoughMemoryForBatch(size_t estimated_batch_size);
    void clearBatchFromGPU(GPUBatchData& gpu_data);
    
    // Device management
    torch::Device getDevice() const { return device_; }
    bool isDeviceAvailable() const;
    
private:
    training::TrainingConfig config_;
    torch::Device device_;
    
    // Convert splat batch to tensors
    bool convertBatchToTensors(const core::types::GaussianSplatBatch& batch, GPUBatchData& gpu_data);
    
    // Initialize SH coefficients
    void initializeSHCoefficients(torch::Tensor& sh_coeffs, const torch::Tensor& colors, int num_gaussians);
};

} // namespace utils
} // namespace gaussian_splatting