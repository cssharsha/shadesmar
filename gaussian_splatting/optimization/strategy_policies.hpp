#pragma once

#include <torch/torch.h>

namespace gaussian_splatting {
namespace optimization {

// Default grow policy implementation
// Uses the original densification algorithm: duplicate small splats with high gradients,
// split large splats with high gradients
class DefaultGrowPolicy {
public:
    template <typename StrategyType>
    void operator()(StrategyType* strategy, int iter);
};

// Default prune policy implementation
// Removes splats with low opacity
class DefaultPrunePolicy {
public:
    template <typename StrategyType>
    void operator()(StrategyType* strategy, int iter);
};

// MCMC grow policy implementation
// Based on "3D Gaussian Splatting as Markov Chain Monte Carlo"
// https://arxiv.org/abs/2404.09591
class MCMCGrowPolicy {
public:
    static struct Config {
        int cap_max = 1000000;         // Maximum number of Gaussians
        float noise_lr = 5e5;          // Noise learning rate for position perturbation
        int refine_start_iter = 1;     // Start MCMC refinement after this iteration
        int refine_stop_iter = 25000;  // Stop MCMC refinement after this iteration
        int refine_every = 10;         // Refine every N iterations
        float min_opacity = 0.005;     // Minimum opacity threshold for dead Gaussians
    } config;

    template <typename StrategyType>
    void operator()(StrategyType* strategy, int iter);

private:
    // Binomial coefficient lookup table [n_max, n_max]
    torch::Tensor binoms_;
    bool initialized_ = false;

    // Initialize binomial coefficient table for Equation 9
    void initializeBinomialTable(const c10::Device& device);

    // Relocate dead Gaussians (opacity < min_opacity) to high-opacity locations
    template <typename StrategyType>
    void relocateGaussians(StrategyType* strategy);

    // Add new Gaussians by sampling from opacity distribution (5% growth)
    template <typename StrategyType>
    void addNewGaussians(StrategyType* strategy);

    // Inject noise to Gaussian positions for exploration
    template <typename StrategyType>
    void injectNoise(StrategyType* strategy, float lr);

    // Compute new opacity and scales using Equation 9 from the paper
    std::pair<torch::Tensor, torch::Tensor> computeRelocation(
        const torch::Tensor& opacities,  // [N] in sigmoid space
        const torch::Tensor& scales,     // [N, 3] in linear space
        const torch::Tensor& ratios      // [N] - number of times each was sampled
    );

    // Helper: Compute ratios (equivalent to torch.bincount()[sampled_idxs])
    torch::Tensor computeRatios(const torch::Tensor& sampled_idxs, int max_idx);

    // Helper: Compute 3x3 covariance matrices from quaternions and scales
    // Returns: [N, 3, 3] covariance matrices
    torch::Tensor quaternionScaleToCovariance(
        const torch::Tensor& quats,  // [N, 4] normalized quaternions
        const torch::Tensor& scales  // [N, 3] scales in linear space
    );

    // Helper: Sigmoid function for opacity-based weighting
    static float opacitySigmoid(float x, float k = 100.0f, float x0 = 0.995f) {
        return 1.0f / (1.0f + std::exp(-k * (x - x0)));
    }
};

// MCMC prune policy implementation
// MCMC doesn't prune - it relocates dead Gaussians instead
class MCMCPrunePolicy {
public:
    template <typename StrategyType>
    void operator()(StrategyType* strategy, int iter) {
        // No-op: MCMC handles "dead" Gaussians via relocation in the grow policy
    }
};

}  // namespace optimization
}  // namespace gaussian_splatting
