#pragma once

#include <torch/torch.h>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>
#include "core/types/gaussian_splat.hpp"

namespace gaussian_splatting {

class GaussianTensors {
public:
    // Default constructor
    GaussianTensors() = default;

    // Constructor from a vector of GaussianSplat objects
    bool fromSplats(const std::vector<core::types::GaussianSplat>& splats);

    bool isValid() const;

    // // Constructor from GPUBatchData
    // explicit GaussianTensors(const GPUBatchData& gpu_data);

    void to(const torch::Device& device);

    // Tensors holding the Gaussian attributes
    torch::Tensor& get_positions() {
        return positions;
    }
    const torch::Tensor& get_positions() const {
        return positions;
    }
    torch::Tensor& get_covariances() {
        return covariances;
    }
    const torch::Tensor& get_covariances() const {
        return covariances;
    }
    torch::Tensor& get_colors() {
        return colors;
    }
    const torch::Tensor& get_colors() const {
        return colors;
    }
    torch::Tensor& get_opacities() {
        return opacities;
    }
    const torch::Tensor& get_opacities() const {
        return opacities;
    }
    torch::Tensor& get_scales() {
        return scales;
    }
    const torch::Tensor& get_scales() const {
        return scales;
    }
    torch::Tensor& get_rotations() {
        return rotations;
    }
    const torch::Tensor& get_rotations() const {
        return rotations;
    }
    torch::Tensor& get_sh_0() {
        return sh_0;
    }
    torch::Tensor& get_sh_N() {
        return sh_N;
    }
    torch::Tensor& get_confidences() {
        return confidences;
    }
    const torch::Tensor& get_confidences() const {
        return confidences;
    }
    uint32_t& number_of_splats() {
        return num_splats;
    }
    const uint32_t& number_of_splats() const {
        return num_splats;
    }
    const int32_t& get_sh_degree() const {
        return sh_degree;
    }
    torch::Tensor get_sh_coefficients() const {
        return torch::cat({sh_0, sh_N}, 1);
    }

    void printGradInfo() const;
    void setRequiresGrad(bool requires_grad = true);

    std::vector<core::types::GaussianSplat> toSplats();
    uint32_t check_stuff = 0;

    // Partial loading support for bounding box optimization
    // Load only a subset of splats specified by indices
    bool fromSplatsSubset(const std::vector<core::types::GaussianSplat>& all_splats,
                          const std::vector<int>& indices);

    // Sync GPU changes back to CPU subset
    // Updates only the splats at the specified global indices
    void syncToCPU(std::vector<core::types::GaussianSplat>& cpu_splats,
                   const std::vector<int>& global_indices);

    // Extract a subset of splats for partial loading
    static std::vector<core::types::GaussianSplat> extractSubset(
        const std::vector<core::types::GaussianSplat>& all_splats,
        const std::vector<int>& indices);

private:
    torch::Tensor positions;
    torch::Tensor covariances;
    torch::Tensor colors;
    torch::Tensor opacities;
    torch::Tensor scales;
    torch::Tensor rotations;
    torch::Tensor sh_0;
    torch::Tensor sh_N;
    torch::Tensor confidences;
    int32_t sh_degree = 0;
    uint32_t num_splats;
};
}  // namespace gaussian_splatting
