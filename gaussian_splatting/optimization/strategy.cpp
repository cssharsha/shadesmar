#include "gaussian_splatting/optimization/strategy.hpp"
#include <logging/logging.hpp>
#include "gaussian_splatting/common/tensor_config.hpp"
#include "gaussian_splatting/optimization/optimizer.hpp"
#include "gaussian_splatting/utils/torch_utils.hpp"

namespace gaussian_splatting {
namespace optimization {

// Define the static config member for all Strategy template instantiations
template <typename GrowPolicy, typename PrunePolicy>
typename Strategy<GrowPolicy, PrunePolicy>::Config Strategy<GrowPolicy, PrunePolicy>::config;

// Note: Policy-specific configs are defined in strategy_policies.cpp

template <typename GrowPolicy, typename PrunePolicy>
void Strategy<GrowPolicy, PrunePolicy>::postBackward(rendering::RasterizationOutput& r_output,
                                                     int iter) {
    // Increment SH degree every 1000 iterations
    torch::NoGradGuard no_grad;
    // TODO: Increase the SH degree. Change it in GaussianTensors
    LOG(INFO) << "Doing postBackward " << iter;

    if (iter >= Optimizer::config.max_iterations) {
        return;
    }

    updateState(r_output);

    if (isRefining(iter)) {
        LOG(INFO) << "Refining";
        growSplats(iter);
        pruneSplats(iter);

        // reset all the state tensors
        grad2d_.zero_();
        count_.zero_();
        radii_.zero_();
    }
    LOG(INFO) << "Done with posetBackward";

    // TODO: reset the opacity after a certain number of iterations
    // gaussians_->reset_opacities();
}

template <typename GrowPolicy, typename PrunePolicy>
void Strategy<GrowPolicy, PrunePolicy>::step(int iter) {
    if (iter < config.max_iterations) {
        optimizer_->step();
        scheduler_->step();
    }
}

template <typename GrowPolicy, typename PrunePolicy>
void Strategy<GrowPolicy, PrunePolicy>::updateState(rendering::RasterizationOutput& r_output) {
    torch::Tensor grads;
    grads = r_output.means2d.grad().clone();

    // Thought this was required as I suspected the libtorch does not
    // automatically zero out the gradients. But it does not seem to be the case.
    // r_output.means2d.grad().zero_();

    LOG(INFO) << "Doing updateState";
    if (!torch::isfinite(grads).all().item<bool>()) {
        LOG(INFO) << "Gradient contains NaN or Inf values.";
        throw std::runtime_error("Gradient contains NaN or Inf values.");
    }

    // Convert the normalized gradients to pixel space
    const float scale_x = r_output.width / 2.F;
    const float scale_y = r_output.height / 2.F;
    grads.select(-1, 0).mul_(scale_x);
    grads.select(-1, 1).mul_(scale_y);

    // On the first iteration we arent storing anything.
    const size_t num_gaussians = gaussians_->get_positions().size(0);
    const c10::Device device = grads.device();
    if (!grad2d_.defined()) {
        grad2d_ = torch::zeros(num_gaussians, torch::kFloat32).to(device);
    }
    if (!radii_.defined()) {
        radii_ = torch::zeros(num_gaussians, torch::kFloat32).to(device);
    }
    if (!count_.defined()) {
        count_ = torch::zeros(num_gaussians, torch::kFloat32).to(device);
    }
    LOG(INFO) << "Set all the internal state tensors: grad2d: " << grad2d_.sizes() << ", "
              << grad2d_.is_cuda() << ", radii: " << radii_.sizes() << ", " << radii_.is_cuda()
              << ", count: " << count_.sizes() << ", " << count_.is_cuda();

    // Indices of the gaussians that are visible in the current frame
    torch::Tensor gaussian_ids;
    torch::Tensor radii;
    const torch::Tensor valid_mask = r_output.radii > 0;
    gaussian_ids = valid_mask.nonzero().squeeze(-1);
    grads = grads.squeeze(0).index_select(0, gaussian_ids);
    radii = r_output.radii.index_select(0, gaussian_ids);
    LOG(INFO) << "Done selecting the gaussians";

    // running sum of the gradients for all visible gaussians
    grad2d_.index_add_(0, gaussian_ids, grads.norm(2, -1));
    // running sum of the visibility count of the gaussians across iterations
    count_.index_add_(0, gaussian_ids, torch::ones_like(gaussian_ids, torch::kFloat32));
    // running max of the radii of the gaussians across iterations
    const double max_wh = static_cast<double>(std::max(r_output.width, r_output.height));
    radii_.index_put_({gaussian_ids},
                      torch::max(radii_.index_select(0, gaussian_ids), radii / max_wh));
    LOG(INFO) << "Done updating the state";
}

template <typename GrowPolicy, typename PrunePolicy>
void Strategy<GrowPolicy, PrunePolicy>::duplicateSplats(const torch::Tensor& is_duplicated) {
    torch::NoGradGuard no_grad;
    torch::Tensor sampled_idxs = is_duplicated.nonzero().squeeze(-1);

    // Enforce max_splat_count: limit duplication to not exceed the threshold
    int64_t current_count = gaussians_->get_positions().size(0);
    int64_t num_to_duplicate = sampled_idxs.size(0);
    int64_t available_capacity = config.max_splat_count - current_count;

    if (available_capacity <= 0) {
        LOG(WARNING) << "Already at max splat count: " << current_count << ", skipping duplication";
        return;
    }

    if (num_to_duplicate > available_capacity) {
        LOG(WARNING) << "Limiting duplication from " << num_to_duplicate << " to "
                     << available_capacity << " to stay within max_splat_count";
        sampled_idxs = sampled_idxs.index({torch::indexing::Slice(0, available_capacity)});
        num_to_duplicate = available_capacity;
    }

    LOG(INFO) << "Duplicating " << num_to_duplicate << " splats (current: " << current_count
              << ", max: " << config.max_splat_count << ")";

    const auto param_fn = [&sampled_idxs](const int i, const torch::Tensor param) {
        const torch::Tensor new_param = param.index_select(0, sampled_idxs);
        return torch::cat({param, new_param}).set_requires_grad(param.requires_grad());
    };

    const auto optimizer_fn =
        [&sampled_idxs](
            torch::optim::OptimizerParamState& state,
            const torch::Tensor full_param) -> std::unique_ptr<torch::optim::OptimizerParamState> {
        auto new_shape = full_param.sizes().vec();
        new_shape[0] = sampled_idxs.size(0);
        auto* adam_state = dynamic_cast<torch::optim::AdamParamState*>(&state);
        auto zeros_to_add = torch::zeros(new_shape, adam_state->exp_avg().options());
        auto new_exp_avg = torch::cat({adam_state->exp_avg(), zeros_to_add}, 0);
        auto new_exp_avg_sq = torch::cat({adam_state->exp_avg_sq(), zeros_to_add}, 0);

        // Create new state
        auto new_state = std::make_unique<torch::optim::AdamParamState>();
        new_state->step(adam_state->step());
        new_state->exp_avg(new_exp_avg);
        new_state->exp_avg_sq(new_exp_avg_sq);
        if (adam_state->max_exp_avg_sq().defined()) {
            auto new_max_exp_avg_sq = torch::cat({adam_state->max_exp_avg_sq(), zeros_to_add}, 0);
            new_state->max_exp_avg_sq(new_max_exp_avg_sq);
        }
        return new_state;
    };

    optimizer_->updateParamAndState(param_fn, optimizer_fn, gaussians_);

    if (grad2d_.defined()) {
        grad2d_ = torch::cat({grad2d_, grad2d_.index_select(0, sampled_idxs)});
    }
    if (radii_.defined()) {
        radii_ = torch::cat({radii_, radii_.index_select(0, sampled_idxs)});
    }
    if (count_.defined()) {
        count_ = torch::cat({count_, count_.index_select(0, sampled_idxs)});
    }
}

template <typename GrowPolicy, typename PrunePolicy>
void Strategy<GrowPolicy, PrunePolicy>::splitSplats(torch::Tensor& is_split) {
    torch::NoGradGuard no_grad;
    const c10::Device device = is_split.device();
    torch::Tensor sampled_idxs = is_split.nonzero().squeeze(-1);

    // Enforce max_splat_count: limit splitting to not exceed the threshold
    // Splitting creates split_size (2) new splats for each original, so net gain is +1 per split
    const auto split_size = 2;
    int64_t current_count = gaussians_->get_positions().size(0);
    int64_t num_to_split = sampled_idxs.size(0);
    int64_t new_splats_per_split = split_size - 1;  // net gain: 1 original becomes 2
    int64_t total_new_splats = num_to_split * new_splats_per_split;
    int64_t available_capacity = config.max_splat_count - current_count;

    if (available_capacity <= 0) {
        LOG(WARNING) << "Already at max splat count: " << current_count << ", skipping splitting";
        return;
    }

    if (total_new_splats > available_capacity) {
        int64_t max_splits_allowed = available_capacity / new_splats_per_split;
        LOG(WARNING) << "Limiting splits from " << num_to_split << " to " << max_splits_allowed
                     << " to stay within max_splat_count";
        sampled_idxs = sampled_idxs.index({torch::indexing::Slice(0, max_splits_allowed)});
        num_to_split = max_splits_allowed;
    }

    LOG(INFO) << "Splitting " << num_to_split << " splats into " << split_size
              << " each (current: " << current_count
              << ", will add: " << (num_to_split * new_splats_per_split)
              << ", max: " << config.max_splat_count << ")";

    // Recompute rest_idxs based on potentially limited sampled_idxs
    torch::Tensor rest_idxs;
    if (num_to_split < is_split.sum().item<int64_t>()) {
        // Some splits were excluded, need to recompute rest_idxs
        auto all_idxs = torch::arange(is_split.size(0), is_split.options().dtype(torch::kLong));
        auto split_mask = torch::zeros_like(is_split);
        split_mask.index_put_({sampled_idxs}, true);
        rest_idxs = all_idxs.index({split_mask.logical_not()});
    } else {
        rest_idxs = is_split.logical_not().nonzero().squeeze(-1);
    }

    const torch::Tensor sampled_scales = gaussians_->get_scales().index_select(0, sampled_idxs);
    // Convert scales from log-space to linear space for einsum operation
    const torch::Tensor sampled_scales_linear = torch::exp(sampled_scales);

    const torch::Tensor sampled_quats = gaussians_->get_rotations().index_select(0, sampled_idxs);
    LOG(INFO) << "Converting quats to rotation matrix: " << sampled_quats.sizes();
    const torch::Tensor rotmats = utils::quaternion_to_rotation_matrix(sampled_quats);

    LOG(INFO) << "All sizes until here: " << sampled_idxs.sizes() << ", " << rest_idxs.sizes()
              << ", " << sampled_scales.sizes() << ", " << sampled_quats.sizes() << ", "
              << rotmats.sizes();

    const auto num_split_gaussians = sampled_idxs.size(0);
    // einsum seems to be super vague syntax but just fetching it from
    // gsplat. From what I understood following are the details:
    // 1. Get a random matrix of shape [split_size, N, 3]
    // 2. bnj,nj -> bni: Element-wise multiply random vectors by scales
    //   - Scales the random vectors by the Gaussian's scale parameters
    //   - these are in the local space of the stretched gaussian ellipsoid
    // 3. nij,bni -> bnj: Rotate by rotation matrix
    //   - Applies the Gaussian's rotation which transform from
    //     local space to world space
    const torch::Tensor samples = torch::einsum(  // [split_size, N, 3]
        "nij,nj,bnj->bni", {rotmats, sampled_scales_linear,
                            torch::randn({split_size, num_split_gaussians, 3},
                                         sampled_quats.options().device(device))});
    LOG(INFO) << "Sampled splats with scale and rotation: " << samples.sizes();

    const auto param_fn = [&sampled_idxs, &rest_idxs, &samples, &split_size, &sampled_scales](
                              const int i, const torch::Tensor param) {
        std::vector<int64_t> repeats(param.dim(), 1);
        repeats[0] = split_size;

        LOG(INFO) << "Param sizes: " << param.sizes() << " " << param.device();

        const torch::Tensor sampled_param = param.index_select(0, sampled_idxs);
        LOG(INFO) << "Sampled param: " << sampled_param.sizes();
        torch::Tensor split_param;
        // Split positions. Essentially the end would be split_size * N.
        if (i == 0) {
            LOG(INFO) << "Splitting positions";
            split_param = (sampled_param.unsqueeze(0) + samples).reshape({-1, 3});
            LOG(INFO) << "Split positions: " << split_param.sizes() << " " << split_param.device();
        }
        // Split scales.
        else if (i == 1) {
            LOG(INFO) << "Splitting scales";
            // sampled_scales is already in log-space, so: log(s/1.6) = log(s) - log(1.6)
            split_param =
                (sampled_scales - std::log(1.6f)).repeat({split_size, 1});  // [split_size * N, 3]
        }
        // Split opactiries.
        else if (i == 3) {  // gsplat sets revised_opacity to do this operation
            LOG(INFO) << "Splitting opacities";
            const torch::Tensor new_opacities =
                1.0 - torch::sqrt(1.0 - torch::sigmoid(sampled_param));
            split_param = torch::logit(new_opacities).repeat(repeats);  // [split_size * N]
        }
        // Split the rest of the parameters(rotations, sh_coefficients)
        else {
            LOG(INFO) << "Splitting the rest i: " << i;
            split_param = sampled_param.repeat(repeats);
        }

        LOG(INFO) << "Doing the rest";
        // Concatenate the rest of the parameters that were not split.
        const torch::Tensor rest_param = param.index_select(0, rest_idxs);
        LOG(INFO) << "Concat: " << rest_param.sizes() << ", " << split_param.sizes();
        auto cat_params =
            torch::cat({rest_param, split_param}, 0).set_requires_grad(param.requires_grad());
        LOG(INFO) << "Cat params: " << cat_params.sizes();
        return cat_params;
        // return torch::cat({rest_param, split_param}, 0).set_requires_grad(param.requires_grad());
    };

    const auto state_fn =
        [&sampled_idxs, &rest_idxs, &split_size](
            torch::optim::OptimizerParamState& state,
            const torch::Tensor full_param) -> std::unique_ptr<torch::optim::OptimizerParamState> {
        LOG(INFO) << "Calling state_fn in split";
        LOG(INFO) << "Full param: " << full_param.sizes() << " " << full_param.device();
        auto zero_shape = full_param.sizes().vec();
        zero_shape[0] = sampled_idxs.size(0) * split_size;
        auto* adam_state = dynamic_cast<torch::optim::AdamParamState*>(&state);
        // Standard Adam state
        auto rest_exp_avg = adam_state->exp_avg().index_select(0, rest_idxs);
        auto rest_exp_avg_sq = adam_state->exp_avg_sq().index_select(0, rest_idxs);
        LOG(INFO) << "Rest exp avg: " << rest_exp_avg.sizes() << " " << rest_exp_avg_sq.device();

        // New state for all the splits
        auto zeros_to_add = torch::zeros(zero_shape, adam_state->exp_avg().options());
        LOG(INFO) << "Zeros to add: " << zeros_to_add.sizes() << " " << zeros_to_add.device();
        auto new_exp_avg = torch::cat({rest_exp_avg, zeros_to_add}, 0);
        LOG(INFO) << "New exp avg: " << new_exp_avg.sizes() << " " << new_exp_avg.device();
        auto new_exp_avg_sq = torch::cat({rest_exp_avg_sq, zeros_to_add}, 0);
        LOG(INFO) << "New exp avg sq: " << new_exp_avg_sq.sizes() << " " << new_exp_avg_sq.device();

        auto new_state = std::make_unique<torch::optim::AdamParamState>();
        new_state->step(adam_state->step());
        new_state->exp_avg(new_exp_avg);
        new_state->exp_avg_sq(new_exp_avg_sq);
        if (adam_state->max_exp_avg_sq().defined()) {
            auto rest_max_exp_avg_sq = adam_state->max_exp_avg_sq().index_select(0, rest_idxs);
            auto new_max_exp_avg_sq = torch::cat({rest_max_exp_avg_sq, zeros_to_add}, 0);
            new_state->max_exp_avg_sq(new_max_exp_avg_sq);
        }
        return new_state;
    };

    optimizer_->updateParamAndState(param_fn, state_fn, gaussians_);

    // Update the extra running state
    const auto make_repeats = [&split_size](const at::Tensor& t) {
        std::vector<int64_t> v(t.dim(), 1);
        v[0] = split_size;
        return v;
    };
    if (grad2d_.defined()) {
        grad2d_ = torch::cat({grad2d_.index_select(0, rest_idxs),
                              grad2d_.index_select(0, sampled_idxs).repeat(make_repeats(grad2d_))});
    }
    if (radii_.defined()) {
        radii_ = torch::cat({radii_.index_select(0, rest_idxs),
                             radii_.index_select(0, sampled_idxs).repeat(make_repeats(radii_))});
    }
    if (count_.defined()) {
        count_ = torch::cat({count_.index_select(0, rest_idxs),
                             count_.index_select(0, sampled_idxs).repeat(make_repeats(count_))});
    }
}

template <typename GrowPolicy, typename PrunePolicy>
void Strategy<GrowPolicy, PrunePolicy>::removeSplats(const torch::Tensor& is_prune) {
    torch::NoGradGuard no_grad;
    LOG(INFO) << "Remove splats";

    // Flatten to 1D before nonzero to get proper 1D indices
    const torch::Tensor sampled_idxs = is_prune.flatten().logical_not().nonzero().squeeze(-1);
    LOG(INFO) << "Sampled idxs: " << sampled_idxs.sizes();

    const auto param_fn = [&sampled_idxs](const int i, const torch::Tensor param) {
        LOG(INFO) << "Param fn " << i << " " << param.sizes();
        return param.index_select(0, sampled_idxs).set_requires_grad(param.requires_grad());
    };

    const auto state_fn =
        [&sampled_idxs](
            torch::optim::OptimizerParamState& state,
            const torch::Tensor new_param) -> std::unique_ptr<torch::optim::OptimizerParamState> {
        auto* adam_state = dynamic_cast<torch::optim::AdamParamState*>(&state);
        LOG(INFO) << "Doing state update";
        // Standard Adam state
        auto new_exp_avg = adam_state->exp_avg().index_select(0, sampled_idxs);
        auto new_exp_avg_sq = adam_state->exp_avg_sq().index_select(0, sampled_idxs);

        // Create new state
        auto new_state = std::make_unique<torch::optim::AdamParamState>();
        new_state->step(adam_state->step());
        new_state->exp_avg(new_exp_avg);
        new_state->exp_avg_sq(new_exp_avg_sq);
        if (adam_state->max_exp_avg_sq().defined()) {
            auto new_max_exp_avg_sq = adam_state->max_exp_avg_sq().index_select(0, sampled_idxs);
            new_state->max_exp_avg_sq(new_max_exp_avg_sq);
        }
        return new_state;
    };

    optimizer_->updateParamAndState(param_fn, state_fn, gaussians_);

    // Update the extra running state
    if (grad2d_.defined()) {
        grad2d_ = grad2d_.index_select(0, sampled_idxs);
    }
    if (radii_.defined()) {
        radii_ = radii_.index_select(0, sampled_idxs);
    }
    if (count_.defined()) {
        count_ = count_.index_select(0, sampled_idxs);
    }
}

// Explicit template instantiation for DefaultStrategy
template class Strategy<DefaultGrowPolicy, DefaultPrunePolicy>;

}  // namespace optimization
}  // namespace gaussian_splatting
