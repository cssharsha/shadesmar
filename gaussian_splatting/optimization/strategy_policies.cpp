#include "gaussian_splatting/optimization/strategy_policies.hpp"
#include "gaussian_splatting/optimization/strategy.hpp"
#include "gaussian_splatting/common/tensor_config.hpp"
#include "gaussian_splatting/utils/torch_utils.hpp"
#include <logging/logging.hpp>

namespace gaussian_splatting {
namespace optimization {

template<typename StrategyType>
void DefaultGrowPolicy::operator()(StrategyType* strategy, int iter) {
    torch::NoGradGuard no_grad;
    LOG(INFO) << "Doing grow splats";

    // Check if we're at max capacity
    int64_t current_splat_count = strategy->getGaussians()->get_positions().size(0);
    if (current_splat_count >= strategy->getConfig().max_splat_count) {
        LOG(WARNING) << "Reached max splat count: " << current_splat_count
                     << ", skipping densification";
        return;
    }

    // Average gradient per gaussian accross all iterations
    const torch::Tensor grads = strategy->getGrad2d() / strategy->getCount().clamp_min(1);
    LOG(INFO) << "Done getting the average gradient";

    // Higher gradients -> unable to fit -> needs refinement
    // Currently Im setting the scene scale to .75. Might need to
    // revisit this to change later.
    const torch::Tensor is_grad_high = grads > strategy->getConfig().grad_threshold;
    LOG(INFO) << "Grads: " << grads.is_cuda() << ", is_grad_high: " << is_grad_high.is_cuda()
              << "gaussians->get_scales(): " << strategy->getGaussians()->get_scales().is_cuda();
    LOG(INFO) << "Check stuff: " << strategy->getGaussians()->check_stuff;

    // Convert scales from log-space to linear space before comparison
    const auto scales_linear = torch::exp(strategy->getGaussians()->get_scales());
    const auto max_values = std::get<0>(torch::max(scales_linear, -1));
    LOG(INFO) << "Max values: " << max_values.is_cuda();
    const torch::Tensor is_small =
        max_values <= strategy->getConfig().grow_scale3d * strategy->getGaussians()->get_scene_scale();
    const torch::Tensor is_duplicated = is_grad_high & is_small;
    auto duplicate_count = is_duplicated.sum().item<int64_t>();
    LOG(INFO) << "Duplicate count: " << duplicate_count;

    const torch::Tensor is_large = ~is_small;
    torch::Tensor is_split = is_grad_high & is_large;
    is_split |= strategy->getRadii() > strategy->getConfig().grow_scale2d;
    auto split_count = is_split.sum().item<int64_t>();
    LOG(INFO) << "Split count: " << split_count;

    if (duplicate_count > 0) {
        strategy->duplicateSplats(is_duplicated);
    }

    LOG(INFO) << "Is split: " << is_split.sizes();
    auto duplicates = torch::zeros(duplicate_count,
                                   c10::TensorOptions().dtype(torch::kBool).device(grads.device()));
    LOG(INFO) << "Duplicates: " << duplicates.sizes();
    // Set the duplicated splats to zero so as to not split them
    is_split = torch::cat(
        {is_split, torch::zeros(duplicate_count,
                                c10::TensorOptions().dtype(torch::kBool).device(grads.device()))});
    if (split_count > 0) {
        LOG(INFO) << "Calling split splats with is split: " << is_split.sizes();
        strategy->splitSplats(is_split);
    }

    LOG(INFO) << "Duplicated: " << duplicate_count /*<< ", split: " << split_count*/;

    return;
}

template<typename StrategyType>
void DefaultPrunePolicy::operator()(StrategyType* strategy, int iter) {
    torch::NoGradGuard no_grad;
    LOG(INFO) << "Prune splats";

    // Opacities are stored in logit space, so apply sigmoid before comparing to threshold
    auto opacities_sigmoid = torch::sigmoid(strategy->getGaussians()->get_opacities());
    torch::Tensor is_prune = opacities_sigmoid < strategy->getConfig().prune_opacity;

    // Printing mean and std of opacities (in sigmoid space for interpretability)
    LOG(INFO) << "Mean opacity: " << common::itemAs(torch::mean(opacities_sigmoid));
    LOG(INFO) << "Std opacity: " << common::itemAs(torch::std(opacities_sigmoid));
    LOG(INFO) << "Is prune: " << is_prune.sizes();

    LOG(INFO) << iter << " is less than " << strategy->getConfig().reset_after_iterations;

    LOG(INFO) << "Some comp: " << is_prune.sum().item();
    const int64_t num_prunes = is_prune.sum().item<int64_t>();
    if (num_prunes > 0) {
        LOG(INFO) << "Need to remove splats";
        strategy->removeSplats(is_prune);
    }
    return;
}

// Explicit template instantiations for the default Strategy type
template class Strategy<DefaultGrowPolicy, DefaultPrunePolicy>;
template void DefaultGrowPolicy::operator()(Strategy<DefaultGrowPolicy, DefaultPrunePolicy>* strategy, int iter);
template void DefaultPrunePolicy::operator()(Strategy<DefaultGrowPolicy, DefaultPrunePolicy>* strategy, int iter);

}  // namespace optimization
}  // namespace gaussian_splatting
