#include "gaussian_splatting/optimization/strategy.hpp"
#include <logging/logging.hpp>
#include "gaussian_splatting/optimization/optimizer.hpp"
#include "gaussian_splatting/utils/torch_utils.hpp"

namespace gaussian_splatting {
namespace optimization {
Strategy::Config Strategy::config;

void Strategy::postBackward(rendering::RasterizationOutput& r_output, int iter) {
    // Increment SH degree every 1000 iterations
    torch::NoGradGuard no_grad;
    // TODO: Increase the SH degree. Change it in GaussianTensors
    std::cout << "Doing postBackward " << iter << std::endl;

    if (iter >= Optimizer::config.max_iterations) {
        return;
    }

    updateState(r_output);

    if (isRefining(iter)) {
        std::cout << "Refining" << std::endl;
        growSplats(iter);
        pruneSplats(iter);

        // reset all the state tensors
        grad2d_.zero_();
        count_.zero_();
        radii_.zero_();
    }
    std::cout << "Done with posetBackward" << std::endl;

    // TODO: reset the opacity after a certain number of iterations
    // gaussians_->reset_opacities();
}

void Strategy::step(int iter) {
    if (iter < config.max_iterations) {
        optimizer_->step();
        scheduler_->step();
    }
}

void Strategy::updateState(rendering::RasterizationOutput& r_output) {
    torch::Tensor grads;
    grads = r_output.means2d.grad().clone();
    std::cout << "Doing updateState" << std::endl;
    if (!torch::isfinite(grads).all().item<bool>()) {
        std::cout << "Gradient contains NaN or Inf values." << std::endl;
        throw std::runtime_error("Gradient contains NaN or Inf values.");
    }

    std::cout << "Cleared grads NaN or Inf check" << std::endl;

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
    std::cout << "Set all the internal state tensors: grad2d: " << grad2d_.sizes() << ", "
              << grad2d_.is_cuda() << ", radii: " << radii_.sizes() << ", " << radii_.is_cuda()
              << ", count: " << count_.sizes() << ", " << count_.is_cuda() << std::endl;

    // Indices of the gaussians that are visible in the current frame
    torch::Tensor gaussian_ids;
    torch::Tensor radii;
    const torch::Tensor valid_mask = r_output.radii > 0;
    gaussian_ids = valid_mask.nonzero().squeeze(-1);
    grads = grads.squeeze(0).index_select(0, gaussian_ids);
    radii = r_output.radii.index_select(0, gaussian_ids);
    std::cout << "Done selecting the gaussians" << std::endl;

    // running sum of the gradients for all visible gaussians
    grad2d_.index_add_(0, gaussian_ids, grads.norm(2, -1));
    // running sum of the visibility count of the gaussians across iterations
    count_.index_add_(0, gaussian_ids, torch::ones_like(gaussian_ids, torch::kFloat32));
    // running max of the radii of the gaussians across iterations
    const double max_wh = static_cast<double>(std::max(r_output.width, r_output.height));
    radii_.index_put_({gaussian_ids},
                      torch::max(radii_.index_select(0, gaussian_ids), radii / max_wh));
    std::cout << "Done updating the state" << std::endl;
}

void Strategy::growSplats(int iter) {
    torch::NoGradGuard no_grad;
    std::cout << "Doing grow splats" << std::endl;

    // Check if we're at max capacity
    int64_t current_splat_count = gaussians_->get_positions().size(0);
    if (current_splat_count >= config.max_splat_count) {
        LOG(WARNING) << "Reached max splat count: " << current_splat_count
                     << ", skipping densification";
        return;
    }

    // Average gradient per gaussian accross all iterations
    const torch::Tensor grads = grad2d_ / count_.clamp_min(1);
    std::cout << "Done getting the average gradient" << std::endl;

    // Higher gradients -> unable to fit -> needs refinement
    // Currently Im setting the scene scale to .75. Might need to
    // revisit this to change later.
    const torch::Tensor is_grad_high = grads > config.grad_threshold;
    std::cout << "Grads: " << grads.is_cuda() << ", is_grad_high: " << is_grad_high.is_cuda()
              << "gaussians->get_scales(): " << gaussians_->get_scales().is_cuda() << std::endl;
    std::cout << "Check stuff: " << gaussians_->check_stuff << std::endl;

    const auto max_values = std::get<0>(torch::max(gaussians_->get_scales(), -1));
    std::cout << "Max values: " << max_values.is_cuda() << std::endl;
    const torch::Tensor is_small = max_values <= config.grow_scale3d * 0.75F;
    const torch::Tensor is_duplicated = is_grad_high & is_small;
    auto duplicate_count = is_duplicated.sum().item<int64_t>();
    std::cout << "Duplicate count: " << duplicate_count << std::endl;

    const torch::Tensor is_large = ~is_small;
    torch::Tensor is_split = is_grad_high & is_large;
    is_split |= radii_ > config.grow_scale2d;
    auto split_count = is_split.sum().item<int64_t>();
    std::cout << "Split count: " << split_count << std::endl;

    if (duplicate_count > 0) {
        duplicateSplats(is_duplicated);
    }

    std::cout << "Is split: " << is_split.sizes() << std::endl;
    auto duplicates = torch::zeros(duplicate_count,
                                   c10::TensorOptions().dtype(torch::kBool).device(grads.device()));
    std::cout << "Duplicates: " << duplicates.sizes() << std::endl;
    // Set the duplicated splats to zero so as to not split them
    is_split = torch::cat(
        {is_split, torch::zeros(duplicate_count,
                                c10::TensorOptions().dtype(torch::kBool).device(grads.device()))});
    if (split_count > 0) {
        std::cout << "Calling split splats with is split: " << is_split.sizes() << std::endl;
        splitSplats(is_split);
    }

    std::cout << "Duplicated: " << duplicate_count << ", split: " << split_count << std::endl;

    return;
}

void Strategy::duplicateSplats(const torch::Tensor& is_duplicated) {
    torch::NoGradGuard no_grad;
    torch::Tensor sampled_idxs = is_duplicated.nonzero().squeeze(-1);

    // Enforce max_splat_count: limit duplication to not exceed the threshold
    int64_t current_count = gaussians_->get_positions().size(0);
    int64_t num_to_duplicate = sampled_idxs.size(0);
    int64_t available_capacity = config.max_splat_count - current_count;

    if (available_capacity <= 0) {
        LOG(WARNING) << "Already at max splat count: " << current_count
                     << ", skipping duplication";
        return;
    }

    if (num_to_duplicate > available_capacity) {
        LOG(WARNING) << "Limiting duplication from " << num_to_duplicate
                     << " to " << available_capacity << " to stay within max_splat_count";
        sampled_idxs = sampled_idxs.index({torch::indexing::Slice(0, available_capacity)});
        num_to_duplicate = available_capacity;
    }

    std::cout << "Duplicating " << num_to_duplicate << " splats (current: " << current_count
              << ", max: " << config.max_splat_count << ")" << std::endl;

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

void Strategy::splitSplats(torch::Tensor& is_split) {
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
        LOG(WARNING) << "Already at max splat count: " << current_count
                     << ", skipping splitting";
        return;
    }

    if (total_new_splats > available_capacity) {
        int64_t max_splits_allowed = available_capacity / new_splats_per_split;
        LOG(WARNING) << "Limiting splits from " << num_to_split
                     << " to " << max_splits_allowed << " to stay within max_splat_count";
        sampled_idxs = sampled_idxs.index({torch::indexing::Slice(0, max_splits_allowed)});
        num_to_split = max_splits_allowed;
    }

    std::cout << "Splitting " << num_to_split << " splats into " << split_size
              << " each (current: " << current_count << ", will add: "
              << (num_to_split * new_splats_per_split) << ", max: " << config.max_splat_count
              << ")" << std::endl;

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
    const torch::Tensor sampled_quats = gaussians_->get_rotations().index_select(0, sampled_idxs);
    std::cout << "Converting quats to rotation matrix: " << sampled_quats.sizes() << std::endl;
    const torch::Tensor rotmats = utils::quaternion_to_rotation_matrix(sampled_quats);

    std::cout << "All sizes until here: " << sampled_idxs.sizes() << ", " << rest_idxs.sizes()
              << ", " << sampled_scales.sizes() << ", " << sampled_quats.sizes() << ", "
              << rotmats.sizes() << std::endl;

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
        "nij,nj,bnj->bni", {rotmats, sampled_scales,
                            torch::randn({split_size, num_split_gaussians, 3},
                                         sampled_quats.options().device(device))});
    std::cout << "Sampled splats with scale and rotation: " << samples.sizes() << std::endl;

    const auto param_fn = [&sampled_idxs, &rest_idxs, &samples, &split_size, &sampled_scales](
                              const int i, const torch::Tensor param) {
        std::vector<int64_t> repeats(param.dim(), 1);
        repeats[0] = split_size;

        std::cout << "Param sizes: " << param.sizes() << " " << param.device() << std::endl;

        const torch::Tensor sampled_param = param.index_select(0, sampled_idxs);
        std::cout << "Sampled param: " << sampled_param.sizes() << std::endl;
        torch::Tensor split_param;
        // Split positions. Essentially the end would be split_size * N.
        if (i == 0) {
            std::cout << "Splitting positions" << std::endl;
            split_param = (sampled_param.unsqueeze(0) + samples).reshape({-1, 3});
            std::cout << "Split positions: " << split_param.sizes() << " " << split_param.device()
                      << std::endl;
        }
        // Split scales.
        else if (i == 1) {
            std::cout << "Splitting scales" << std::endl;
            split_param =
                torch::log(sampled_scales / 1.6).repeat({split_size, 1});  // [split_size * N, 3]
        }
        // Split opactiries.
        else if (i == 3) {  // gsplat sets revised_opacity to do this operation
            std::cout << "Splitting opacities" << std::endl;
            const torch::Tensor new_opacities =
                1.0 - torch::sqrt(1.0 - torch::sigmoid(sampled_param));
            split_param = torch::logit(new_opacities).repeat(repeats);  // [split_size * N]
        }
        // Split the rest of the parameters(rotations, sh_coefficients)
        else {
            std::cout << "Splitting the rest i: " << i << std::endl;
            split_param = sampled_param.repeat(repeats);
        }

        std::cout << "Doing the rest" << std::endl;
        // Concatenate the rest of the parameters that were not split.
        const torch::Tensor rest_param = param.index_select(0, rest_idxs);
        std::cout << "Concat: " << rest_param.sizes() << ", " << split_param.sizes() << std::endl;
        auto cat_params =
            torch::cat({rest_param, split_param}, 0).set_requires_grad(param.requires_grad());
        std::cout << "Cat params: " << cat_params.sizes() << std::endl;
        return cat_params;
        // return torch::cat({rest_param, split_param}, 0).set_requires_grad(param.requires_grad());
    };

    const auto state_fn =
        [&sampled_idxs, &rest_idxs, &split_size](
            torch::optim::OptimizerParamState& state,
            const torch::Tensor full_param) -> std::unique_ptr<torch::optim::OptimizerParamState> {
        std::cout << "Calling state_fn in split" << std::endl;
        std::cout << "Full param: " << full_param.sizes() << " " << full_param.device()
                  << std::endl;
        auto zero_shape = full_param.sizes().vec();
        zero_shape[0] = sampled_idxs.size(0) * split_size;
        auto* adam_state = dynamic_cast<torch::optim::AdamParamState*>(&state);
        // Standard Adam state
        auto rest_exp_avg = adam_state->exp_avg().index_select(0, rest_idxs);
        auto rest_exp_avg_sq = adam_state->exp_avg_sq().index_select(0, rest_idxs);
        std::cout << "Rest exp avg: " << rest_exp_avg.sizes() << " " << rest_exp_avg_sq.device()
                  << std::endl;

        // New state for all the splits
        auto zeros_to_add = torch::zeros(zero_shape, adam_state->exp_avg().options());
        std::cout << "Zeros to add: " << zeros_to_add.sizes() << " " << zeros_to_add.device()
                  << std::endl;
        auto new_exp_avg = torch::cat({rest_exp_avg, zeros_to_add}, 0);
        std::cout << "New exp avg: " << new_exp_avg.sizes() << " " << new_exp_avg.device()
                  << std::endl;
        auto new_exp_avg_sq = torch::cat({rest_exp_avg_sq, zeros_to_add}, 0);
        std::cout << "New exp avg sq: " << new_exp_avg_sq.sizes() << " " << new_exp_avg_sq.device()
                  << std::endl;

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

void Strategy::pruneSplats(int iter) {
    torch::NoGradGuard no_grad;
    std::cout << "Prune splats" << std::endl;

    // Opacities are stored in logit space, so apply sigmoid before comparing to threshold
    auto opacities_sigmoid = torch::sigmoid(gaussians_->get_opacities());
    torch::Tensor is_prune = opacities_sigmoid < config.prune_opacity;

    // Printing mean and std of opacities (in sigmoid space for interpretability)
    std::cout << "Mean opacity: " << torch::mean(opacities_sigmoid).item<float>() << std::endl;
    std::cout << "Std opacity: " << torch::std(opacities_sigmoid).item<float>() << std::endl;
    std::cout << "Is prune: " << is_prune.sizes() << std::endl;
    if (iter > config.reset_after_iterations) {
        std::cout << "Reset after iterations" << std::endl;
        const auto max_values = std::get<0>(torch::max(gaussians_->get_scales(), -1));
        // TODO: Remove the hard coded scene scale
        torch::Tensor is_too_big = max_values > config.prune_scale3d * 0.75F;

        is_too_big |= radii_ > config.prune_scale2d;

        is_prune |= is_too_big;
    }

    std::cout << iter << " is less than " << config.reset_after_iterations << std::endl;

    std::cout << "Some comp: " << is_prune.sum().item() << std::endl;
    const int64_t num_prunes = is_prune.sum().item<int64_t>();
    if (num_prunes > 0) {
        std::cout << "Need to remove splats" << std::endl;
        removeSplats(is_prune);
    }
    return;
}

void Strategy::removeSplats(const torch::Tensor& is_prune) {
    torch::NoGradGuard no_grad;
    std::cout << "Remove splats" << std::endl;

    // Flatten to 1D before nonzero to get proper 1D indices
    const torch::Tensor sampled_idxs = is_prune.flatten().logical_not().nonzero().squeeze(-1);
    std::cout << "Sampled idxs: " << sampled_idxs.sizes() << std::endl;

    const auto param_fn = [&sampled_idxs](const int i, const torch::Tensor param) {
        std::cout << "Param fn " << i << " " << param.sizes() << std::endl;
        return param.index_select(0, sampled_idxs).set_requires_grad(param.requires_grad());
    };

    const auto state_fn =
        [&sampled_idxs](
            torch::optim::OptimizerParamState& state,
            const torch::Tensor new_param) -> std::unique_ptr<torch::optim::OptimizerParamState> {
        auto* adam_state = dynamic_cast<torch::optim::AdamParamState*>(&state);
        std::cout << "Doing state update" << std::endl;
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

}  // namespace optimization
}  // namespace gaussian_splatting
