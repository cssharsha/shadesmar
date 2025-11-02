#include "parameter_transforms.hpp"
#include <logging/logging.hpp>
#include "gaussian_splatting/common/tensor_config.hpp"

namespace gaussian_splatting {
namespace optimization {

torch::Tensor ParameterTransforms::applySigmoidOpacity(const torch::Tensor& raw_opacity) {
    // Sigmoid activation: 1 / (1 + exp(-x))
    return torch::sigmoid(raw_opacity);
}

torch::Tensor ParameterTransforms::applyExponentialScaling(const torch::Tensor& raw_scaling) {
    // Exponential activation: exp(x)
    return torch::exp(raw_scaling);
}

torch::Tensor ParameterTransforms::normalizeRotations(const torch::Tensor& quaternions) {
    // Normalize quaternions to unit length
    return torch::nn::functional::normalize(quaternions, torch::nn::functional::NormalizeFuncOptions().p(2).dim(1));
}

torch::Tensor ParameterTransforms::inverseSigmoid(const torch::Tensor& opacity) {
    // Inverse sigmoid: log(x / (1 - x))
    auto clamped = torch::clamp(opacity, 1e-6f, 1.0f - 1e-6f);
    return torch::log(clamped / (1.0f - clamped));
}

torch::Tensor ParameterTransforms::inverseExponential(const torch::Tensor& scaling) {
    // Inverse exponential: log(x)
    auto clamped = torch::clamp(scaling, 1e-6f, 1e6f);
    return torch::log(clamped);
}

torch::Tensor ParameterTransforms::clipGradients(const torch::Tensor& gradients, float max_norm) {
    auto grad_norm = torch::norm(gradients);
    if (common::itemAs(grad_norm) > max_norm) {
        return gradients * (max_norm / grad_norm);
    }
    return gradients;
}

} // namespace optimization
} // namespace gaussian_splatting