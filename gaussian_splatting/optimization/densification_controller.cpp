#include "densification_controller.hpp"
#include <logging/logging.hpp>
#include "gaussian_splatting/common/tensor_config.hpp"

namespace gaussian_splatting {
namespace optimization {

DensificationController::DensificationController(const training::TrainingConfig& config)
    : config_(config) {
    LOG(INFO) << "Initializing DensificationController";
}

bool DensificationController::shouldDensify(int iteration) const {
    return iteration > 0 && iteration % config_.densification_interval == 0;
}

bool DensificationController::densifyAndPrune(GaussianTensors& gaussians,
                                              const torch::Tensor& position_gradients,
                                              DensificationStats& stats) {
    LOG(INFO) << "Performing densification and pruning";

    if (!gaussians.isValid()) {
        LOG(ERROR) << "Invalid GPU data for densification";
        return false;
    }

    // Accumulate gradients
    accumulateGradients(position_gradients);

    // Compute average gradients
    auto avg_gradients = computeAverageGradients();

    // Identify operations
    auto indices_to_remove = identifyGaussiansToRemove(gaussians);
    auto indices_to_split = identifyGaussiansToSplit(avg_gradients, gaussians.get_scales());
    auto indices_to_clone = identifyGaussiansToClone(avg_gradients, gaussians.get_scales());

    int initial_count = gaussians.number_of_splats();

    // Apply operations in order: remove, split, clone
    if (!removeGaussians(gaussians, indices_to_remove)) {
        LOG(ERROR) << "Failed to remove gaussians";
        return false;
    }

    if (!splitGaussians(gaussians, indices_to_split)) {
        LOG(ERROR) << "Failed to split gaussians";
        return false;
    }

    if (!cloneGaussians(gaussians, indices_to_clone)) {
        LOG(ERROR) << "Failed to clone gaussians";
        return false;
    }

    // Update stats
    stats.gaussians_removed = indices_to_remove.size();
    stats.gaussians_added = indices_to_split.size() + indices_to_clone.size();
    stats.total_gaussians = gaussians.number_of_splats();
    stats.avg_opacity = common::itemAs(gaussians.get_opacities().mean());
    stats.avg_gradient_norm = common::itemAs(avg_gradients.norm());

    LOG(INFO) << "Densification complete: " << initial_count << " -> "
              << gaussians.number_of_splats() << " (removed " << stats.gaussians_removed
              << ", added " << stats.gaussians_added << ")";

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

std::vector<int> DensificationController::identifyGaussiansToRemove(
    const GaussianTensors& gaussians) const {
    std::vector<int> indices_to_remove;

    auto opacity_accessor = gaussians.get_opacities().accessor<float, 2>();

    for (int i = 0; i < gaussians.number_of_splats(); ++i) {
        if (opacity_accessor[i][0] < config_.opacity_threshold) {
            indices_to_remove.push_back(i);
        }
    }

    return indices_to_remove;
}

std::vector<int> DensificationController::identifyGaussiansToSplit(
    const torch::Tensor& avg_gradients, const torch::Tensor& scales) const {
    std::vector<int> indices_to_split;

    if (!avg_gradients.defined() || !scales.defined()) {
        return indices_to_split;
    }

    auto grad_accessor = avg_gradients.accessor<float, 1>();
    auto scale_accessor = scales.accessor<float, 2>();

    for (int i = 0; i < avg_gradients.size(0); ++i) {
        float max_scale =
            std::max({scale_accessor[i][0], scale_accessor[i][1], scale_accessor[i][2]});

        if (grad_accessor[i] > config_.densify_grad_threshold &&
            max_scale > config_.densify_size_threshold) {
            indices_to_split.push_back(i);
        }
    }

    return indices_to_split;
}

std::vector<int> DensificationController::identifyGaussiansToClone(
    const torch::Tensor& avg_gradients, const torch::Tensor& scales) const {
    std::vector<int> indices_to_clone;

    if (!avg_gradients.defined() || !scales.defined()) {
        return indices_to_clone;
    }

    auto grad_accessor = avg_gradients.accessor<float, 1>();
    auto scale_accessor = scales.accessor<float, 2>();

    for (int i = 0; i < avg_gradients.size(0); ++i) {
        float max_scale =
            std::max({scale_accessor[i][0], scale_accessor[i][1], scale_accessor[i][2]});

        if (grad_accessor[i] > config_.densify_grad_threshold &&
            max_scale <= config_.densify_size_threshold) {
            indices_to_clone.push_back(i);
        }
    }

    return indices_to_clone;
}

bool DensificationController::removeGaussians(GaussianTensors& gaussians,
                                              const std::vector<int>& indices_to_remove) {
    if (indices_to_remove.empty()) {
        return true;
    }

    // Create mask for keeping gaussians
    auto keep_mask = torch::ones({gaussians.number_of_splats()}, torch::kBool);
    for (int idx : indices_to_remove) {
        keep_mask[idx] = false;
    }

    // Filter tensors
    gaussians.get_positions() =
        gaussians.get_positions().index_select(0, keep_mask.nonzero().squeeze());
    gaussians.get_rotations() =
        gaussians.get_rotations().index_select(0, keep_mask.nonzero().squeeze());
    gaussians.get_scales() = gaussians.get_scales().index_select(0, keep_mask.nonzero().squeeze());
    gaussians.get_opacities() =
        gaussians.get_opacities().index_select(0, keep_mask.nonzero().squeeze());
    gaussians.get_sh_coefficients() =
        gaussians.get_sh_coefficients().index_select(0, keep_mask.nonzero().squeeze());
    gaussians.get_colors() = gaussians.get_colors().index_select(0, keep_mask.nonzero().squeeze());

    gaussians.number_of_splats() = gaussians.get_positions().size(0);

    return true;
}

bool DensificationController::splitGaussians(GaussianTensors& gaussians,
                                             const std::vector<int>& indices_to_split) {
    if (indices_to_split.empty()) {
        return true;
    }

    // Create copies of gaussians to split
    std::vector<torch::Tensor> new_positions, new_rotations, new_scales, new_opacities,
        new_sh_coeffs, new_colors;

    for (int idx : indices_to_split) {
        // Split into two gaussians with smaller scales
        auto pos = gaussians.get_positions()[idx];
        auto rot = gaussians.get_rotations()[idx];
        auto scale = gaussians.get_scales()[idx] * 0.8f;  // Reduce scale
        auto opacity = gaussians.get_opacities()[idx];
        auto sh = gaussians.get_sh_coefficients()[idx];
        auto color = gaussians.get_colors()[idx];

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
        gaussians.get_positions() =
            torch::cat({gaussians.get_positions(), torch::stack(new_positions)}, 0);
        gaussians.get_rotations() =
            torch::cat({gaussians.get_rotations(), torch::stack(new_rotations)}, 0);
        gaussians.get_scales() = torch::cat({gaussians.get_scales(), torch::stack(new_scales)}, 0);
        gaussians.get_opacities() =
            torch::cat({gaussians.get_opacities(), torch::stack(new_opacities)}, 0);
        gaussians.get_sh_coefficients() =
            torch::cat({gaussians.get_sh_coefficients(), torch::stack(new_sh_coeffs)}, 0);
        gaussians.get_colors() = torch::cat({gaussians.get_colors(), torch::stack(new_colors)}, 0);

        gaussians.number_of_splats() = gaussians.get_positions().size(0);
    }

    return true;
}

bool DensificationController::cloneGaussians(GaussianTensors& gaussians,
                                             const std::vector<int>& indices_to_clone) {
    if (indices_to_clone.empty()) {
        return true;
    }

    // Create copies of gaussians to clone
    std::vector<torch::Tensor> new_positions, new_rotations, new_scales, new_opacities,
        new_sh_coeffs, new_colors;

    for (int idx : indices_to_clone) {
        auto pos = gaussians.get_positions()[idx];
        auto offset = torch::randn_like(pos) * 0.01f;

        new_positions.push_back(pos + offset);
        new_rotations.push_back(gaussians.get_rotations()[idx]);
        new_scales.push_back(gaussians.get_scales()[idx]);
        new_opacities.push_back(gaussians.get_opacities()[idx]);
        new_sh_coeffs.push_back(gaussians.get_sh_coefficients()[idx]);
        new_colors.push_back(gaussians.get_colors()[idx]);
    }

    // Concatenate cloned gaussians
    if (!new_positions.empty()) {
        gaussians.get_positions() =
            torch::cat({gaussians.get_positions(), torch::stack(new_positions)}, 0);
        gaussians.get_rotations() =
            torch::cat({gaussians.get_rotations(), torch::stack(new_rotations)}, 0);
        gaussians.get_scales() = torch::cat({gaussians.get_scales(), torch::stack(new_scales)}, 0);
        gaussians.get_opacities() =
            torch::cat({gaussians.get_opacities(), torch::stack(new_opacities)}, 0);
        gaussians.get_sh_coefficients() =
            torch::cat({gaussians.get_sh_coefficients(), torch::stack(new_sh_coeffs)}, 0);
        gaussians.get_colors() = torch::cat({gaussians.get_colors(), torch::stack(new_colors)}, 0);

        gaussians.number_of_splats() = gaussians.get_positions().size(0);
    }

    return true;
}

torch::Tensor DensificationController::computeAverageGradients() const {
    if (!accumulated_gradients_.defined() || !gradient_counts_.defined()) {
        return torch::Tensor();
    }

    return accumulated_gradients_ / gradient_counts_;
}

void DensificationController::updateGaussianCounts(GaussianTensors& gaussians) {
    gaussians.number_of_splats() = gaussians.get_positions().size(0);
}

}  // namespace optimization
}  // namespace gaussian_splatting
