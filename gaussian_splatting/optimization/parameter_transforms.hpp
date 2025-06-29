#pragma once

#include <torch/torch.h>

namespace gaussian_splatting {
namespace optimization {

class ParameterTransforms {
public:
    // Apply sigmoid activation to opacity parameters
    static torch::Tensor applySigmoidOpacity(const torch::Tensor& raw_opacity);
    
    // Apply exponential activation to scaling parameters
    static torch::Tensor applyExponentialScaling(const torch::Tensor& raw_scaling);
    
    // Normalize quaternion rotations
    static torch::Tensor normalizeRotations(const torch::Tensor& quaternions);
    
    // Inverse transforms for optimization
    static torch::Tensor inverseSigmoid(const torch::Tensor& opacity);
    static torch::Tensor inverseExponential(const torch::Tensor& scaling);
    
    // Gradient clipping utilities
    static torch::Tensor clipGradients(const torch::Tensor& gradients, float max_norm = 1.0f);
};

} // namespace optimization
} // namespace gaussian_splatting