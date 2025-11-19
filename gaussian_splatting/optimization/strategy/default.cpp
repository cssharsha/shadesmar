#include <c10/cuda/CUDACachingAllocator.h>
#include "gaussian_splatting/optimization/strategy/default.hpp"
#include <logging/logging.hpp>
#include "gaussian_splatting/common/tensor_config.hpp"
#include "gaussian_splatting/gsplat/gsplat/cuda/include/Ops.h"
#include "gaussian_splatting/optimization/strategy/strategy.hpp"
#include "gaussian_splatting/training/gaussian_tensors.hpp"
#include "gaussian_splatting/utils/torch_utils.hpp"



namespace gaussian_splatting {
namespace optimization {
namespace strategy {

void Default::operator()(GaussianTensors* gaussians, rendering::RasterizationOutput& render_output,
                         int iter) {
    torch::NoGradGuard no_grad;
    // TODO: Increase the SH degree. Change it in GaussianTensors
    LOG(INFO) << "Doing postBackward " << iter;

    if (iter >= config.stop_refine) {
        return;
    }

    update(gaussians, render_output);

    if (isRefining(iter)) {
        LOG(INFO) << "Refining";
        growSplats(gaussians, iter);
        pruneSplats(gaussians, iter);

        // reset all the state tensors
        grads_.zero_();
        count_.zero_();
        radii_.zero_();

        c10::cuda::CUDACachingAllocator::emptyCache();
    }
    LOG(INFO) << "Done with posetBackward";

    // TODO: reset the opacity after a certain number of iterations
    // gaussians_->reset_opacities();
}

void Default::update(GaussianTensors* gaussians, rendering::RasterizationOutput& render_output) {
    torch::Tensor grad2d = render_output.means2d.grad().clone();

    const float scale_x = render_output.width / 2.F;
    const float scale_y = render_output.height / 2.F;
    grad2d.select(-1, 0).mul_(scale_x);
    grad2d.select(-1, 1).mul_(scale_y);

    // On the first iteration we arent storing anything.
    const size_t num_gaussians = gaussians->number_of_splats();
    const c10::Device device = grad2d.device();
    if (!grads_.defined()) {
        grads_ = torch::zeros(num_gaussians, torch::kFloat32).to(device);
    }
    if (!radii_.defined()) {
        radii_ = torch::zeros(num_gaussians, torch::kFloat32).to(device);
    }
    if (!count_.defined()) {
        count_ = torch::zeros(num_gaussians, torch::kFloat32).to(device);
    }
    LOG(INFO) << "Set all the internal state tensors: grad2d: " << grads_.sizes() << ", "
              << grads_.is_cuda() << ", radii: " << radii_.sizes() << ", " << radii_.is_cuda()
              << ", count: " << count_.sizes() << ", " << count_.is_cuda();

    // Indices of the gaussians that are visible in the current frame
    torch::Tensor gaussian_ids;
    torch::Tensor radii;
    const torch::Tensor valid_mask = render_output.radii > 0;

    LOG(INFO) << "Shapes: grad2d: " << grad2d.sizes() << ", valid_mask: " << valid_mask.sizes()
              << ", radii: " << radii.sizes();
    gaussian_ids = valid_mask.nonzero().squeeze(-1);
    grad2d = grad2d.squeeze(0).index_select(0, gaussian_ids);
    radii = render_output.radii.index_select(0, gaussian_ids);
    LOG(INFO) << "Done selecting the gaussians";

    // running sum of the gradients for all visible gaussians
    grads_.index_add_(0, gaussian_ids, grad2d.norm(2, -1));
    count_.index_add_(0, gaussian_ids, torch::ones_like(gaussian_ids, torch::kFloat32));
    const double max_wh = static_cast<double>(std::max(render_output.width, render_output.height));
    radii_.index_put_({gaussian_ids},
                      torch::max(radii_.index_select(0, gaussian_ids), radii / max_wh));
    LOG(INFO) << "Done updating the state";
}

void Default::growSplats(GaussianTensors* gaussians, int iter) {
    torch::NoGradGuard no_grad;
    LOG(INFO) << "Doing grow splats";

    // Check if we're at max capacity
    int64_t current_splat_count = gaussians->number_of_splats();
    if (current_splat_count >= config.max_splat_count) {
        LOG(WARNING) << "Reached max splat count: " << current_splat_count
                     << ", skipping densification";
        return;
    }

    // Average gradient per gaussian accross all iterations
    const torch::Tensor grads = grads_ / count_.clamp_min(1);
    LOG(INFO) << "Done getting the average gradient";

    // Higher gradients -> unable to fit -> needs refinement
    // Currently Im setting the scene scale to .75. Might need to
    // revisit this to change later.
    const torch::Tensor is_grad_high = grads > config.grad_threshold;
    LOG(INFO) << "Grads: " << grads.is_cuda() << ", is_grad_high: " << is_grad_high.is_cuda()
              << "gaussians->get_scales(): " << gaussians->get_scales().is_cuda();
    LOG(INFO) << "Check stuff: " << gaussians->check_stuff;

    const auto max_values = std::get<0>(torch::max(gaussians->get_scales(), -1));
    LOG(INFO) << "Max values: " << max_values.is_cuda();
    const torch::Tensor is_small = max_values <= config.grow_scale3d * gaussians->get_scene_scale();
    const torch::Tensor is_duplicated = is_grad_high & is_small;
    auto duplicate_count = is_duplicated.sum().item<int64_t>();
    LOG(INFO) << "Duplicate count: " << duplicate_count;

    const torch::Tensor is_large = ~is_small;
    torch::Tensor is_split = is_grad_high & is_large;
    is_split |= radii_ > config.grow_scale2d;
    auto split_count = is_split.sum().item<int64_t>();
    LOG(INFO) << "Split count: " << split_count;

    if (duplicate_count > 0) {
        duplicateSplats(gaussians, is_duplicated);
    }

    LOG(INFO) << "Is split: " << is_split.sizes();
    auto duplicates = torch::zeros(duplicate_count,
                                   c10::TensorOptions().dtype(torch::kBool).device(grads.device()));
    LOG(INFO) << "Duplicates: " << duplicates.sizes();
    // Set the duplicated splats to zero so as to not split them
    is_split = torch::cat({is_split, duplicates});
    if (split_count > 0) {
        LOG(INFO) << "Calling split splats with is split: " << is_split.sizes();
        splitSplats(gaussians, is_split);
    }

    LOG(INFO) << "Duplicated: " << duplicate_count /*<< ", split: " << split_count*/;

    return;
}

void Default::duplicateSplats(GaussianTensors* gaussians, const torch::Tensor& is_duplicated) {
    torch::NoGradGuard no_grad;
    const c10::Device device = is_duplicated.device();
    const torch::Tensor sampled_idxs = is_duplicated.nonzero().squeeze(-1);

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

    optimizer_->updateParamAndState(param_fn, optimizer_fn, gaussians);

    // Update the extra running state
    const int num_new_gaussians = sampled_idxs.size(0);
    if (!grads_.defined()) {
        grads_ = torch::cat({grads_, grads_.index_select(0, sampled_idxs)});
    }
    if (!radii_.defined()) {
        radii_ = torch::cat({radii_, radii_.index_select(0, sampled_idxs)});
    }
    if (!count_.defined()) {
        count_ = torch::cat({count_, count_.index_select(0, sampled_idxs)});
    }
    LOG(INFO) << "Set all the internal state tensors: grads: " << grads_.sizes() << ", "
              << grads_.is_cuda() << ", radii: " << radii_.sizes() << ", " << radii_.is_cuda()
              << ", count: " << count_.sizes() << ", " << count_.is_cuda();
}

void Default::splitSplats(GaussianTensors* gaussians, const torch::Tensor& is_split) {
    torch::NoGradGuard no_grad;
    const c10::Device device = is_split.device();
    const torch::Tensor sampled_idxs = is_split.nonzero().squeeze(-1);
    const torch::Tensor rest_idxs = is_split.logical_not().nonzero().squeeze(-1);

    const torch::Tensor sampled_scales = gaussians->get_scales().index_select(0, sampled_idxs);
    const torch::Tensor sampled_quats = gaussians->get_rotations().index_select(0, sampled_idxs);
    const torch::Tensor rotmats = utils::quaternion_to_rotation_matrix(sampled_quats);  // [N, 3, 3]

    const auto num_split_gaussians = sampled_idxs.size(0);
    const auto split_size = 2;
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

    const auto param_fn = [this, &sampled_idxs, &rest_idxs, &samples, &split_size, &sampled_scales](
                              const int i, const torch::Tensor param) {
        std::vector<int64_t> repeats(param.dim(), 1);
        repeats[0] = split_size;

        const torch::Tensor sampled_param = param.index_select(0, sampled_idxs);
        torch::Tensor split_param;
        if (i == 0) {  // means
            split_param =
                (sampled_param.unsqueeze(0) + samples).reshape({-1, 3});  // [split_size * N, 3]
        } else if (i == 3) {                                              // scaling
            split_param =
                torch::log(sampled_scales / 1.6).repeat({split_size, 1});  // [split_size * N, 3]
        } else if (i == 5 && config.revised_opacity) {                     // opacity
            const torch::Tensor new_opacities =
                1.0 - torch::sqrt(1.0 - torch::sigmoid(sampled_param));
            split_param = torch::logit(new_opacities).repeat(repeats);  // [split_size * N]
        } else {
            split_param = sampled_param.repeat(repeats);
        }

        const torch::Tensor rest_param = param.index_select(0, rest_idxs);
        return torch::cat({rest_param, split_param}, 0).set_requires_grad(param.requires_grad());
    };

    const auto optimizer_fn =
        [&sampled_idxs, &rest_idxs, &split_size](
            torch::optim::OptimizerParamState& state,
            const torch::Tensor full_param) -> std::unique_ptr<torch::optim::OptimizerParamState> {
        auto zero_shape = full_param.sizes().vec();
        zero_shape[0] = sampled_idxs.size(0) * split_size;
        if (auto* adam_state = dynamic_cast<torch::optim::AdamParamState*>(&state)) {
            // Standard Adam state
            auto rest_exp_avg = adam_state->exp_avg().index_select(0, rest_idxs);
            auto rest_exp_avg_sq = adam_state->exp_avg_sq().index_select(0, rest_idxs);

            auto zeros_to_add = torch::zeros(zero_shape, adam_state->exp_avg().options());
            auto new_exp_avg = torch::cat({rest_exp_avg, zeros_to_add}, 0);
            auto new_exp_avg_sq = torch::cat({rest_exp_avg_sq, zeros_to_add}, 0);

            // Create new state
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
        }
        return nullptr;
    };

    optimizer_->updateParamAndState(param_fn, optimizer_fn, gaussians);

    // Update the extra running state
    const auto make_repeats = [&split_size](const at::Tensor& t) {
        std::vector<int64_t> v(t.dim(), 1);
        v[0] = split_size;
        return v;
    };
    if (grads_.defined()) {
        grads_ = torch::cat({grads_.index_select(0, rest_idxs),
                             grads_.index_select(0, sampled_idxs).repeat(make_repeats(grads_))});
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

void Default::pruneSplats(GaussianTensors* gaussians, int iter) {
    torch::NoGradGuard no_grad;
    LOG(INFO) << "Prune splats";

    // Opacities are stored in logit space, so apply sigmoid before comparing to threshold
    torch::Tensor is_prune = gaussians->get_opacities() < config.prune_opacity;

    if (iter > config.refine_every) {
        const auto max_values = std::get<0>(torch::max(gaussians->get_scales(), -1));
        torch::Tensor is_too_big = max_values > config.prune_scale3d * gaussians->get_scene_scale();
        if (iter < config.stop_refine_scale2d) {
            is_too_big |= radii_ > config.prune_scale2d;
        }
        is_prune |= is_too_big;
    }

    const int64_t num_prunes = is_prune.sum().item<int64_t>();
    if (num_prunes > 0) {
        LOG(INFO) << "Need to remove splats";
        removeSplats(gaussians, is_prune);
    }
    return;
}

void Default::removeSplats(GaussianTensors* gaussians, const torch::Tensor& is_prune) {
    torch::NoGradGuard no_grad;
    const torch::Tensor sampled_idxs = is_prune.logical_not().nonzero().squeeze(-1);

    const auto param_fn = [&sampled_idxs](const int i, const torch::Tensor param) {
        return param.index_select(0, sampled_idxs).set_requires_grad(param.requires_grad());
    };

    const auto optimizer_fn =
        [&sampled_idxs](
            torch::optim::OptimizerParamState& state,
            const torch::Tensor new_param) -> std::unique_ptr<torch::optim::OptimizerParamState> {
        if (auto* adam_state = dynamic_cast<torch::optim::AdamParamState*>(&state)) {
            // Standard Adam state
            auto new_exp_avg = adam_state->exp_avg().index_select(0, sampled_idxs);
            auto new_exp_avg_sq = adam_state->exp_avg_sq().index_select(0, sampled_idxs);

            // Create new state
            auto new_state = std::make_unique<torch::optim::AdamParamState>();
            new_state->step(adam_state->step());
            new_state->exp_avg(new_exp_avg);
            new_state->exp_avg_sq(new_exp_avg_sq);
            if (adam_state->max_exp_avg_sq().defined()) {
                auto new_max_exp_avg_sq =
                    adam_state->max_exp_avg_sq().index_select(0, sampled_idxs);
                new_state->max_exp_avg_sq(new_max_exp_avg_sq);
            }
            return new_state;
        }
        return nullptr;
    };

    optimizer_->updateParamAndState(param_fn, optimizer_fn, gaussians);

    // Update the extra running state
    if (grads_.defined()) {
        grads_ = grads_.index_select(0, sampled_idxs);
    }
    if (radii_.defined()) {
        radii_ = radii_.index_select(0, sampled_idxs);
    }
    if (count_.defined()) {
        count_ = count_.index_select(0, sampled_idxs);
    }
}

}  // namespace strategy
}  // namespace optimization
}  // namespace gaussian_splatting
