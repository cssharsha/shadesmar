#include "gaussian_splatting/optimization/strategy_policies.hpp"
#include <logging/logging.hpp>
#include "gaussian_splatting/common/tensor_config.hpp"
#include "gaussian_splatting/optimization/strategy.hpp"
#include "gaussian_splatting/utils/torch_utils.hpp"
#include "gaussian_splatting/gsplat/gsplat/cuda/include/Ops.h"

namespace gaussian_splatting {
namespace optimization {

template <typename StrategyType>
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
    const torch::Tensor is_small = max_values <= strategy->getConfig().grow_scale3d *
                                                     strategy->getGaussians()->get_scene_scale();
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

template <typename StrategyType>
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

// Define the static config member for Default specialization (must come before template instantiation)
template<>
typename Strategy<DefaultGrowPolicy, DefaultPrunePolicy>::Config Strategy<DefaultGrowPolicy, DefaultPrunePolicy>::config = {};

// Explicit template instantiations for the default Strategy type
template class Strategy<DefaultGrowPolicy, DefaultPrunePolicy>;

template void DefaultGrowPolicy::operator()(
    Strategy<DefaultGrowPolicy, DefaultPrunePolicy>* strategy, int iter);
template void DefaultPrunePolicy::operator()(
    Strategy<DefaultGrowPolicy, DefaultPrunePolicy>* strategy, int iter);

MCMCGrowPolicy::Config MCMCGrowPolicy::config;

void MCMCGrowPolicy::initializeBinomialTable(const c10::Device& device) {
    const int n_max = 51;
    binoms_ =
        torch::zeros({n_max, n_max}, torch::TensorOptions().dtype(torch::kFloat32).device(device));

    // Compute binomial coefficients C(n, k) = n! / (k! * (n-k)!)
    auto binoms_cpu = binoms_.cpu();
    for (int n = 0; n < n_max; ++n) {
        for (int k = 0; k <= n; ++k) {
            // Use Pascal's triangle for numerical stability
            if (k == 0 || k == n) {
                binoms_cpu[n][k] = 1.0f;
            } else {
                binoms_cpu[n][k] = binoms_cpu[n - 1][k - 1] + binoms_cpu[n - 1][k];
            }
        }
    }
    binoms_ = binoms_cpu.to(device);
    initialized_ = true;

    LOG(INFO) << "Initialized binomial coefficient table [" << n_max << ", " << n_max << "]";
}

// Helper: Compute ratios (how many times each index was sampled)
torch::Tensor MCMCGrowPolicy::computeRatios(const torch::Tensor& sampled_idxs, int max_idx) {
    // Equivalent to torch.bincount(sampled_idxs)[sampled_idxs] + 1
    auto bincount = torch::bincount(sampled_idxs, torch::Tensor(), max_idx);
    return bincount.index_select(0, sampled_idxs) + 1;
}

// Compute new opacity and scales using Equation 9 from the paper
std::pair<torch::Tensor, torch::Tensor> MCMCGrowPolicy::computeRelocation(
    const torch::Tensor& opacities, const torch::Tensor& scales, const torch::Tensor& ratios) {
    torch::NoGradGuard no_grad;

    const int n_max = binoms_.size(0);

    // Clamp ratios to valid range [1, n_max-1]
    auto ratios_clamped = ratios.clamp(1, n_max - 1).to(torch::kInt32);

    // Use gsplat's optimized CUDA kernel for relocation (Equation 9)
    auto [new_opacities, new_scales] = gsplat::relocation(
        opacities,
        scales,
        ratios_clamped,
        binoms_,
        n_max
    );

    return {new_opacities, new_scales};
}

// Main MCMC operator
template <typename StrategyType>
void MCMCGrowPolicy::operator()(StrategyType* strategy, int iter) {
    torch::NoGradGuard no_grad;

    // Initialize binomial table on first call
    if (!initialized_) {
        c10::Device device = strategy->getGaussians()->get_positions().device();
        initializeBinomialTable(device);
    }

    // Only refine within the specified iteration range
    if (iter >= config.refine_start_iter && iter < config.refine_stop_iter &&
        iter % config.refine_every == 0) {
        LOG(INFO) << "MCMC refinement at iteration " << iter;

        // Might be better suited to add this in MCMCPrunePolicy, but keeping
        // it here for now.
        relocateGaussians(strategy);

        // Step 2: Add new Gaussians (5% growth)
        addNewGaussians(strategy);
    }

    // Always inject noise for exploration (even outside refinement iterations)
    // Use learning rate from optimizer (would need to be passed in)
    float lr = 1e-3f;  // TODO: Get actual learning rate from optimizer
    injectNoise(strategy, lr);
}

// Relocate dead Gaussians to high-opacity locations
template <typename StrategyType>
void MCMCGrowPolicy::relocateGaussians(StrategyType* strategy) {
    torch::NoGradGuard no_grad;

    auto gaussians = strategy->getGaussians();
    auto optimizer = strategy->getOptimizer();

    // Get current opacities in sigmoid space [0, 1]
    auto opacities_logit = gaussians->get_opacities();
    auto opacities = torch::sigmoid(opacities_logit.flatten());

    // Identify dead Gaussians
    auto dead_mask = opacities <= config.min_opacity;
    int n_dead = dead_mask.sum().template item<int>();

    if (n_dead == 0) {
        LOG(INFO) << "No dead Gaussians to relocate";
        return;
    }

    LOG(INFO) << "Relocating " << n_dead << " dead Gaussians";

    // Get indices of dead and alive Gaussians
    auto dead_indices = dead_mask.nonzero().squeeze(-1);
    auto alive_mask = ~dead_mask;
    auto alive_indices = alive_mask.nonzero().squeeze(-1);

    if (alive_indices.size(0) == 0) {
        LOG(WARNING) << "No alive Gaussians to sample from!";
        return;
    }

    // Sample from alive Gaussians weighted by their opacity
    auto probs = opacities.index_select(0, alive_indices);
    probs = probs / probs.sum();  // Normalize to probability distribution

    // Use multinomial sampling
    torch::Tensor local_sampled_idxs;
    // Assuming the number of alive gaussians is 16 million or less
    // since torch::multinomial limit is 2^24
    if (alive_indices.size(0) <= (1 << 24)) {
        local_sampled_idxs = torch::multinomial(probs, n_dead, true);
    } else {
        // Fallback for very large numbers (unlikely in practice)
        LOG(WARNING) << "Using random sampling fallback";
        local_sampled_idxs =
            torch::randint(0, alive_indices.size(0), {n_dead}, alive_indices.options());
    }

    // Convert local indices to global indices
    auto sampled_idxs = alive_indices.index_select(0, local_sampled_idxs);

    // Compute how many times each Gaussian was sampled
    int max_idx = opacities.size(0);
    auto ratios = computeRatios(sampled_idxs, max_idx);

    // Compute new opacities and scales using relocation formula
    auto sampled_opacities = opacities.index_select(0, sampled_idxs);
    auto sampled_scales_log = gaussians->get_scales().index_select(0, sampled_idxs);
    auto sampled_scales_linear = torch::exp(sampled_scales_log);

    auto [new_opacities, new_scales] =
        computeRelocation(sampled_opacities, sampled_scales_linear, ratios);

    // Clamp opacities to valid range
    float eps = std::numeric_limits<float>::epsilon();
    new_opacities = new_opacities.clamp(config.min_opacity, 1.0f - eps);

    // Convert back to logit space for opacities and log space for scales
    auto new_opacities_logit = torch::logit(new_opacities).unsqueeze(-1);  // Add dimension to match [N, 1]
    auto new_scales_log = torch::log(new_scales);

    // Update parameters:
    // 1. First update sampled indices with new values
    // 2. Then copy sampled parameters to dead indices

    // Update opacities
    auto updated_opacities = opacities_logit.clone();
    updated_opacities.index_put_({sampled_idxs}, new_opacities_logit);
    updated_opacities.index_put_({dead_indices}, new_opacities_logit);
    gaussians->get_opacities().copy_(updated_opacities);

    // Update scales
    auto updated_scales = gaussians->get_scales().clone();
    updated_scales.index_put_({sampled_idxs}, new_scales_log);
    updated_scales.index_put_({dead_indices}, new_scales_log);
    gaussians->get_scales().copy_(updated_scales);

    // Copy all other parameters from sampled to dead
    auto positions = gaussians->get_positions();
    positions.index_put_({dead_indices}, positions.index_select(0, sampled_idxs));

    auto rotations = gaussians->get_rotations();
    rotations.index_put_({dead_indices}, rotations.index_select(0, sampled_idxs));

    auto sh_coeffs = gaussians->get_sh_coefficients();
    sh_coeffs.index_put_({dead_indices}, sh_coeffs.index_select(0, sampled_idxs));

    // Reset optimizer state for relocated Gaussians
    // This ensures the relocated Gaussians start fresh with zero momentum
    auto& optimizer_state = optimizer->getOptimizer()->state();

    // Get parameter keys for each Gaussian parameter
    auto& param_groups = optimizer->getOptimizer()->param_groups();

    // Iterate through all parameter groups and reset state for relocated indices
    for (const auto& param_group : param_groups) {
        for (const auto& param : param_group.params()) {
            std::string param_key = c10::guts::to_string(param.unsafeGetTensorImpl());
            auto state_it = optimizer_state.find(param_key);

            if (state_it != optimizer_state.end()) {
                auto* adam_state = dynamic_cast<torch::optim::AdamParamState*>(state_it->second.get());
                if (adam_state) {
                    // Reset exp_avg and exp_avg_sq for sampled and dead indices
                    auto indices_to_reset = torch::cat({sampled_idxs, dead_indices});
                    adam_state->exp_avg().index_fill_(0, indices_to_reset, 0.0);
                    adam_state->exp_avg_sq().index_fill_(0, indices_to_reset, 0.0);
                }
            }
        }
    }

    LOG(INFO) << "Reset optimizer state for " << sampled_idxs.size(0)
              << " sampled and " << n_dead << " dead Gaussians";

    LOG(INFO) << "Successfully relocated " << n_dead << " Gaussians";
}

template <typename StrategyType>
void MCMCGrowPolicy::addNewGaussians(StrategyType* strategy) {
    torch::NoGradGuard no_grad;

    auto gaussians = strategy->getGaussians();
    int64_t current_n = gaussians->get_positions().size(0);

    // Target 5% growth, capped at max
    int64_t target_n = std::min((int64_t)config.cap_max, (int64_t)(1.05 * current_n));
    int64_t n_to_add = std::max((int64_t)0, target_n - current_n);

    if (n_to_add == 0) {
        LOG(INFO) << "Already at target size, not adding new Gaussians";
        return;
    }

    // Check if we're at capacity
    if (current_n >= config.cap_max) {
        LOG(WARNING) << "Reached max Gaussian count: " << current_n;
        return;
    }

    LOG(INFO) << "Adding " << n_to_add << " new Gaussians (current: " << current_n
              << ", target: " << target_n << ")";

    // Sample from all Gaussians weighted by their opacity
    auto opacities_logit = gaussians->get_opacities();
    auto opacities = torch::sigmoid(opacities_logit.flatten());

    // Normalize to probability distribution
    auto probs = opacities / opacities.sum();

    // Sample indices
    torch::Tensor sampled_idxs;
    if (opacities.size(0) <= (1 << 24)) {
        sampled_idxs = torch::multinomial(probs, n_to_add, true);
    } else {
        LOG(WARNING) << "Using random sampling fallback for large Gaussian count";
        sampled_idxs = torch::randint(0, opacities.size(0), {n_to_add},
                                      opacities.options().dtype(torch::kLong));
    }

    // Compute ratios for relocation formula
    int max_idx = opacities.size(0);
    auto ratios = computeRatios(sampled_idxs, max_idx);

    // Compute new opacities and scales
    auto sampled_opacities = opacities.index_select(0, sampled_idxs);
    auto sampled_scales_log = gaussians->get_scales().index_select(0, sampled_idxs);
    auto sampled_scales_linear = torch::exp(sampled_scales_log);

    auto [new_opacities, new_scales] =
        computeRelocation(sampled_opacities, sampled_scales_linear, ratios);

    // Clamp and convert
    float eps = std::numeric_limits<float>::epsilon();
    new_opacities = new_opacities.clamp(config.min_opacity, 1.0f - eps);
    auto new_opacities_logit = torch::logit(new_opacities).unsqueeze(-1);  // Add dimension to match [N, 1]
    auto new_scales_log = torch::log(new_scales);

    // First update the sampled indices with new values
    auto updated_opacities = opacities_logit.clone();
    updated_opacities.index_put_({sampled_idxs}, new_opacities_logit);

    auto updated_scales = gaussians->get_scales().clone();
    updated_scales.index_put_({sampled_idxs}, new_scales_log);

    // Concatenate new Gaussians to all parameters
    auto new_positions = gaussians->get_positions().index_select(0, sampled_idxs);
    gaussians->get_positions() = torch::cat({gaussians->get_positions(), new_positions}, 0);

    gaussians->get_scales() = torch::cat({updated_scales, new_scales_log}, 0);

    auto new_rotations = gaussians->get_rotations().index_select(0, sampled_idxs);
    gaussians->get_rotations() = torch::cat({gaussians->get_rotations(), new_rotations}, 0);

    gaussians->get_opacities() = torch::cat({updated_opacities, new_opacities_logit}, 0);

    // Update sh_0 and sh_N separately (they are tracked separately by the optimizer)
    auto new_sh_0 = gaussians->get_sh_0().index_select(0, sampled_idxs);
    gaussians->get_sh_0() = torch::cat({gaussians->get_sh_0(), new_sh_0}, 0);

    auto new_sh_N = gaussians->get_sh_N().index_select(0, sampled_idxs);
    gaussians->get_sh_N() = torch::cat({gaussians->get_sh_N(), new_sh_N}, 0);

    // Update num_splats to reflect the new total
    gaussians->number_of_splats() = current_n + n_to_add;

    // Update strategy state tensors (grad2d, radii, count)
    auto& grad2d = strategy->getGrad2d();
    if (grad2d.defined()) {
        auto new_grad2d = torch::zeros({n_to_add}, grad2d.options());
        strategy->getGrad2d() = torch::cat({grad2d, new_grad2d}, 0);
    }

    auto& radii = strategy->getRadii();
    if (radii.defined()) {
        auto new_radii = torch::zeros({n_to_add}, radii.options());
        strategy->getRadii() = torch::cat({radii, new_radii}, 0);
    }

    auto& count = strategy->getCount();
    if (count.defined()) {
        auto new_count = torch::zeros({n_to_add}, count.options());
        strategy->getCount() = torch::cat({count, new_count}, 0);
    }

    // Update optimizer state to include new parameters
    // We need to update both the parameter references and expand the optimizer state
    auto optimizer = strategy->getOptimizer();
    auto& param_groups = optimizer->getOptimizer()->param_groups();
    auto& optimizer_state = optimizer->getOptimizer()->state();

    // Update parameter references and expand optimizer state
    for (size_t i = 0; i < param_groups.size(); ++i) {
        auto& param = param_groups[i].params()[0];
        std::string old_param_key = c10::guts::to_string(param.unsafeGetTensorImpl());

        // Get the current state if it exists
        auto state_it = optimizer_state.find(old_param_key);

        // Determine which Gaussian parameter this is and get the updated tensor
        torch::Tensor* new_param_ptr = nullptr;
        if (i == 0) new_param_ptr = &gaussians->get_positions();
        else if (i == 1) new_param_ptr = &gaussians->get_scales();
        else if (i == 2) new_param_ptr = &gaussians->get_rotations();
        else if (i == 3) new_param_ptr = &gaussians->get_opacities();
        else if (i == 4) new_param_ptr = &gaussians->get_sh_0();
        else if (i == 5) new_param_ptr = &gaussians->get_sh_N();

        if (new_param_ptr) {
            // Remove old state
            if (state_it != optimizer_state.end()) {
                auto* adam_state = dynamic_cast<torch::optim::AdamParamState*>(state_it->second.get());
                if (adam_state) {
                    // Expand exp_avg and exp_avg_sq with zeros for new Gaussians
                    auto zeros = torch::zeros_like(new_param_ptr->slice(0, current_n, current_n + n_to_add));
                    adam_state->exp_avg(torch::cat({adam_state->exp_avg(), zeros}, 0));
                    adam_state->exp_avg_sq(torch::cat({adam_state->exp_avg_sq(), zeros}, 0));
                }
                optimizer_state.erase(old_param_key);
            }

            // Update parameter reference
            param = *new_param_ptr;

            // Re-add state with new key
            if (state_it != optimizer_state.end()) {
                std::string new_param_key = c10::guts::to_string(new_param_ptr->unsafeGetTensorImpl());
                optimizer_state[new_param_key] = std::move(state_it->second);
            }
        }
    }

    LOG(INFO) << "Successfully added " << n_to_add
              << " new Gaussians. Total: " << gaussians->get_positions().size(0);
}

// Helper: Compute covariance matrices from quaternions and scales
torch::Tensor MCMCGrowPolicy::quaternionScaleToCovariance(const torch::Tensor& quats,
                                                          const torch::Tensor& scales) {
    torch::NoGradGuard no_grad;

    // Convert quaternions to rotation matrices [N, 3, 3]
    auto rotmats = utils::quaternion_to_rotation_matrix(quats);

    // Create scale squared matrices [N, 3, 3] as diagonal
    // scales: [N, 3] -> scales_sq: [N, 3, 3] diagonal
    int N = scales.size(0);
    auto scales_sq = torch::zeros({N, 3, 3}, scales.options());

    // Fill diagonal with squared scales
    for (int i = 0; i < 3; ++i) {
        scales_sq.select(-1, i).select(-1, i).copy_(scales.select(-1, i).pow(2));
    }

    // Compute covariance: Σ = R @ S² @ R^T
    // Using batch matrix multiplication
    auto temp = torch::bmm(rotmats, scales_sq);                 // [N, 3, 3]
    auto covars = torch::bmm(temp, rotmats.transpose(-2, -1));  // [N, 3, 3]

    return covars;
}

// Inject noise to Gaussian positions for exploration
template <typename StrategyType>
void MCMCGrowPolicy::injectNoise(StrategyType* strategy, float lr) {
    torch::NoGradGuard no_grad;

    auto gaussians = strategy->getGaussians();

    // Get parameters
    auto opacities_logit = gaussians->get_opacities();
    auto opacities = torch::sigmoid(opacities_logit.flatten());
    auto scales_log = gaussians->get_scales();
    auto scales = torch::exp(scales_log);
    auto quats = gaussians->get_rotations();

    // Normalize quaternions
    quats = torch::nn::functional::normalize(quats,
                                             torch::nn::functional::NormalizeFuncOptions().dim(-1));

    // Compute covariance matrices [N, 3, 3]
    auto covars = quaternionScaleToCovariance(quats, scales);

    // Compute noise scaling factor based on opacity
    // Higher noise for transparent Gaussians (encourage exploration)
    float scaler = lr * config.noise_lr;

    // Apply sigmoid weighting: higher weight for low-opacity Gaussians
    auto opacity_weight = torch::zeros_like(opacities);
    auto opacities_cpu = opacities.cpu();
    auto opacity_weight_cpu = opacity_weight.cpu();

    for (int i = 0; i < opacities.size(0); ++i) {
        float opa = opacities_cpu[i].template item<float>();
        opacity_weight_cpu[i] = opacitySigmoid(1.0f - opa);
    }
    opacity_weight = opacity_weight_cpu.to(opacities.device());

    // Generate random noise [N, 3]
    auto noise =
        torch::randn_like(gaussians->get_positions()) * opacity_weight.unsqueeze(-1) * scaler;

    // Transform noise by covariance: noise_transformed = Σ @ noise
    // noise: [N, 3] -> [N, 3, 1] for batch matrix-vector multiplication
    auto noise_expanded = noise.unsqueeze(-1);                                // [N, 3, 1]
    auto noise_transformed = torch::bmm(covars, noise_expanded).squeeze(-1);  // [N, 3]

    // Add transformed noise to positions
    gaussians->get_positions().add_(noise_transformed);

    LOG(INFO) << "Injected noise to " << gaussians->get_positions().size(0)
              << " Gaussian positions (scaler=" << scaler << ")";
}

// Define the static config member for MCMC specialization (must come before template instantiation)
template<>
typename Strategy<MCMCGrowPolicy, MCMCPrunePolicy>::Config Strategy<MCMCGrowPolicy, MCMCPrunePolicy>::config = {};

// Explicit template instantiations for MCMC Strategy
template class Strategy<MCMCGrowPolicy, MCMCPrunePolicy>;

template void MCMCGrowPolicy::operator()(Strategy<MCMCGrowPolicy, MCMCPrunePolicy>* strategy,
                                         int iter);
template void MCMCGrowPolicy::relocateGaussians(
    Strategy<MCMCGrowPolicy, MCMCPrunePolicy>* strategy);
template void MCMCGrowPolicy::addNewGaussians(Strategy<MCMCGrowPolicy, MCMCPrunePolicy>* strategy);
template void MCMCGrowPolicy::injectNoise(Strategy<MCMCGrowPolicy, MCMCPrunePolicy>* strategy,
                                          float lr);

}  // namespace optimization
}  // namespace gaussian_splatting
