#include "densification_controller.hpp"
#include <logging/logging.hpp>

namespace gaussian_splatting {
namespace optimization {

DensificationController::DensificationController(const training::TrainingConfig& config)
    : config_(config) {
    LOG(INFO) << "Initializing DensificationController";
}

bool DensificationController::shouldDensify(int iteration) const {
    return iteration > 0 && iteration % config_.densification_interval == 0;
}

bool DensificationController::densifyAndPrune(utils::GPUBatchData& gpu_data,
                                            const torch::Tensor& position_gradients,
                                            DensificationStats& stats) {
    LOG(INFO) << "Performing densification and pruning";
    
    if (!gpu_data.isValid()) {
        LOG(ERROR) << "Invalid GPU data for densification";
        return false;
    }
    
    // Accumulate gradients
    accumulateGradients(position_gradients);
    
    // Compute average gradients
    auto avg_gradients = computeAverageGradients();
    
    // Identify operations
    auto indices_to_remove = identifyGaussiansToRemove(gpu_data);
    auto indices_to_split = identifyGaussiansToSplit(avg_gradients, gpu_data.scales);
    auto indices_to_clone = identifyGaussiansToClone(avg_gradients, gpu_data.scales);
    
    int initial_count = gpu_data.num_gaussians;
    
    // Apply operations in order: remove, split, clone
    if (!removeGaussians(gpu_data, indices_to_remove)) {
        LOG(ERROR) << "Failed to remove gaussians";
        return false;
    }
    
    if (!splitGaussians(gpu_data, indices_to_split)) {
        LOG(ERROR) << "Failed to split gaussians";
        return false;
    }
    
    if (!cloneGaussians(gpu_data, indices_to_clone)) {
        LOG(ERROR) << "Failed to clone gaussians";
        return false;
    }
    
    // Update stats
    stats.gaussians_removed = indices_to_remove.size();
    stats.gaussians_added = indices_to_split.size() + indices_to_clone.size();
    stats.total_gaussians = gpu_data.num_gaussians;
    stats.avg_opacity = gpu_data.opacities.mean().item<float>();
    stats.avg_gradient_norm = avg_gradients.norm().item<float>();
    
    LOG(INFO) << "Densification complete: " << initial_count << " -> " << gpu_data.num_gaussians
              << " (removed " << stats.gaussians_removed << ", added " << stats.gaussians_added << ")";
    
    // Reset for next interval
    resetForNewBatch();
    
    return true;
}

void DensificationController::resetForNewBatch() {
    accumulated_gradients_ = torch::Tensor();
    gradient_counts_ = torch::Tensor();
    accumulation_steps_ = 0;
}

void DensificationController::accumulateGradients(const torch::Tensor& position_gradients) {
    if (!position_gradients.defined() || position_gradients.numel() == 0) {
        return;
    }
    
    auto grad_norms = position_gradients.norm(2, -1);
    
    if (!accumulated_gradients_.defined()) {
        accumulated_gradients_ = grad_norms.clone();
        gradient_counts_ = torch::ones_like(grad_norms);
    } else {
        accumulated_gradients_ += grad_norms;
        gradient_counts_ += 1;
    }
    
    accumulation_steps_++;
}

std::vector<int> DensificationController::identifyGaussiansToRemove(const utils::GPUBatchData& gpu_data) const {
    std::vector<int> indices_to_remove;
    
    auto opacity_accessor = gpu_data.opacities.accessor<float, 2>();
    
    for (int i = 0; i < gpu_data.num_gaussians; ++i) {
        if (opacity_accessor[i][0] < config_.opacity_threshold) {
            indices_to_remove.push_back(i);
        }
    }
    
    return indices_to_remove;
}

std::vector<int> DensificationController::identifyGaussiansToSplit(const torch::Tensor& avg_gradients,
                                                                  const torch::Tensor& scales) const {
    std::vector<int> indices_to_split;
    
    if (!avg_gradients.defined() || !scales.defined()) {
        return indices_to_split;
    }
    
    auto grad_accessor = avg_gradients.accessor<float, 1>();
    auto scale_accessor = scales.accessor<float, 2>();
    
    for (int i = 0; i < avg_gradients.size(0); ++i) {
        float max_scale = std::max({scale_accessor[i][0], scale_accessor[i][1], scale_accessor[i][2]});
        
        if (grad_accessor[i] > config_.densify_grad_threshold && 
            max_scale > config_.densify_size_threshold) {
            indices_to_split.push_back(i);
        }
    }
    
    return indices_to_split;
}

std::vector<int> DensificationController::identifyGaussiansToClone(const torch::Tensor& avg_gradients,
                                                                  const torch::Tensor& scales) const {
    std::vector<int> indices_to_clone;
    
    if (!avg_gradients.defined() || !scales.defined()) {
        return indices_to_clone;
    }
    
    auto grad_accessor = avg_gradients.accessor<float, 1>();
    auto scale_accessor = scales.accessor<float, 2>();
    
    for (int i = 0; i < avg_gradients.size(0); ++i) {
        float max_scale = std::max({scale_accessor[i][0], scale_accessor[i][1], scale_accessor[i][2]});
        
        if (grad_accessor[i] > config_.densify_grad_threshold && 
            max_scale <= config_.densify_size_threshold) {
            indices_to_clone.push_back(i);
        }
    }
    
    return indices_to_clone;
}

bool DensificationController::removeGaussians(utils::GPUBatchData& gpu_data,
                                            const std::vector<int>& indices_to_remove) {
    if (indices_to_remove.empty()) {
        return true;
    }
    
    // Create mask for keeping gaussians
    auto keep_mask = torch::ones({gpu_data.num_gaussians}, torch::kBool);
    for (int idx : indices_to_remove) {
        keep_mask[idx] = false;
    }
    
    // Filter tensors
    gpu_data.positions = gpu_data.positions.index_select(0, keep_mask.nonzero().squeeze());
    gpu_data.rotations = gpu_data.rotations.index_select(0, keep_mask.nonzero().squeeze());
    gpu_data.scales = gpu_data.scales.index_select(0, keep_mask.nonzero().squeeze());
    gpu_data.opacities = gpu_data.opacities.index_select(0, keep_mask.nonzero().squeeze());
    gpu_data.sh_coeffs = gpu_data.sh_coeffs.index_select(0, keep_mask.nonzero().squeeze());
    gpu_data.colors = gpu_data.colors.index_select(0, keep_mask.nonzero().squeeze());
    
    gpu_data.num_gaussians = gpu_data.positions.size(0);
    
    return true;
}

bool DensificationController::splitGaussians(utils::GPUBatchData& gpu_data,
                                           const std::vector<int>& indices_to_split) {
    if (indices_to_split.empty()) {
        return true;
    }
    
    // Create copies of gaussians to split
    std::vector<torch::Tensor> new_positions, new_rotations, new_scales, new_opacities, new_sh_coeffs, new_colors;
    
    for (int idx : indices_to_split) {
        // Split into two gaussians with smaller scales
        auto pos = gpu_data.positions[idx];
        auto rot = gpu_data.rotations[idx];
        auto scale = gpu_data.scales[idx] * 0.8f;  // Reduce scale
        auto opacity = gpu_data.opacities[idx];
        auto sh = gpu_data.sh_coeffs[idx];
        auto color = gpu_data.colors[idx];
        
        // Add small random offset to positions
        auto offset = torch::randn_like(pos) * 0.01f;
        
        new_positions.push_back(pos + offset);
        new_positions.push_back(pos - offset);
        new_rotations.push_back(rot);
        new_rotations.push_back(rot);
        new_scales.push_back(scale);
        new_scales.push_back(scale);
        new_opacities.push_back(opacity);
        new_opacities.push_back(opacity);
        new_sh_coeffs.push_back(sh);
        new_sh_coeffs.push_back(sh);
        new_colors.push_back(color);
        new_colors.push_back(color);
    }
    
    // Concatenate new gaussians
    if (!new_positions.empty()) {
        gpu_data.positions = torch::cat({gpu_data.positions, torch::stack(new_positions)}, 0);
        gpu_data.rotations = torch::cat({gpu_data.rotations, torch::stack(new_rotations)}, 0);
        gpu_data.scales = torch::cat({gpu_data.scales, torch::stack(new_scales)}, 0);
        gpu_data.opacities = torch::cat({gpu_data.opacities, torch::stack(new_opacities)}, 0);
        gpu_data.sh_coeffs = torch::cat({gpu_data.sh_coeffs, torch::stack(new_sh_coeffs)}, 0);
        gpu_data.colors = torch::cat({gpu_data.colors, torch::stack(new_colors)}, 0);
        
        gpu_data.num_gaussians = gpu_data.positions.size(0);
    }
    
    return true;
}

bool DensificationController::cloneGaussians(utils::GPUBatchData& gpu_data,
                                           const std::vector<int>& indices_to_clone) {
    if (indices_to_clone.empty()) {
        return true;
    }
    
    // Create copies of gaussians to clone
    std::vector<torch::Tensor> new_positions, new_rotations, new_scales, new_opacities, new_sh_coeffs, new_colors;
    
    for (int idx : indices_to_clone) {
        auto pos = gpu_data.positions[idx];
        auto offset = torch::randn_like(pos) * 0.01f;
        
        new_positions.push_back(pos + offset);
        new_rotations.push_back(gpu_data.rotations[idx]);
        new_scales.push_back(gpu_data.scales[idx]);
        new_opacities.push_back(gpu_data.opacities[idx]);
        new_sh_coeffs.push_back(gpu_data.sh_coeffs[idx]);
        new_colors.push_back(gpu_data.colors[idx]);
    }
    
    // Concatenate cloned gaussians
    if (!new_positions.empty()) {
        gpu_data.positions = torch::cat({gpu_data.positions, torch::stack(new_positions)}, 0);
        gpu_data.rotations = torch::cat({gpu_data.rotations, torch::stack(new_rotations)}, 0);
        gpu_data.scales = torch::cat({gpu_data.scales, torch::stack(new_scales)}, 0);
        gpu_data.opacities = torch::cat({gpu_data.opacities, torch::stack(new_opacities)}, 0);
        gpu_data.sh_coeffs = torch::cat({gpu_data.sh_coeffs, torch::stack(new_sh_coeffs)}, 0);
        gpu_data.colors = torch::cat({gpu_data.colors, torch::stack(new_colors)}, 0);
        
        gpu_data.num_gaussians = gpu_data.positions.size(0);
    }
    
    return true;
}

torch::Tensor DensificationController::computeAverageGradients() const {
    if (!accumulated_gradients_.defined() || !gradient_counts_.defined()) {
        return torch::Tensor();
    }
    
    return accumulated_gradients_ / gradient_counts_;
}

void DensificationController::updateGaussianCounts(utils::GPUBatchData& gpu_data) {
    gpu_data.num_gaussians = gpu_data.positions.size(0);
}

} // namespace optimization
} // namespace gaussian_splatting