#pragma once

#include <torch/torch.h>
#include <memory>
#include "gaussian_splatting/optimization/optimizer.hpp"
#include "gaussian_splatting/optimization/scheduler.hpp"
#include "gaussian_splatting/optimization/strategy_policies.hpp"
#include "gaussian_splatting/rendering/rasterizer.hpp"

namespace gaussian_splatting {
namespace optimization {

// Strategy class template with pluggable grow and prune policies
template <typename GrowPolicy = DefaultGrowPolicy, typename PrunePolicy = DefaultPrunePolicy>
class Strategy {
public:
    static struct Config {
        int max_iterations = 1000;        // set from the training config rather than from here
        int reset_after_iterations = 50;  // Increased from 10 to reduce densification frequency
        int refine_start_iteration = 50;  // Don't densify before this iteration
        int64_t max_splat_count = 75000;  // Maximum number of splats to prevent OOM
        double grad_threshold = 0.0002;   // from the paper
        double grow_scale3d = 0.1;        // from the paper
        double grow_scale2d = 0.1;        // from the paper
        double prune_opacity = 0.005;
        double prune_scale3d = 0.1;
        double prune_scale2d = 0.15;
    } config;

    Strategy(std::unique_ptr<Optimizer> optimizer, std::unique_ptr<Scheduler> scheduler,
             GaussianTensors* gaussians)
        : optimizer_(std::move(optimizer)),
          scheduler_(std::move(scheduler)),
          gaussians_(gaussians),
          grow_policy_(),
          prune_policy_() {}

    Strategy(std::unique_ptr<Optimizer> optimizer, std::unique_ptr<Scheduler> scheduler,
             GaussianTensors* gaussians, GrowPolicy grow_policy, PrunePolicy prune_policy)
        : optimizer_(std::move(optimizer)),
          scheduler_(std::move(scheduler)),
          gaussians_(gaussians),
          grow_policy_(std::move(grow_policy)),
          prune_policy_(std::move(prune_policy)) {}

    void postBackward(rendering::RasterizationOutput& r_output, int iter);
    void step(int iter);

    // Public accessors for policies to use
    torch::Tensor& getGrad2d() {
        return grad2d_;
    }
    torch::Tensor& getRadii() {
        return radii_;
    }
    torch::Tensor& getCount() {
        return count_;
    }
    optimization::Optimizer* getOptimizer() {
        return optimizer_.get();
    }
    torch::optim::Optimizer* getActualOptimizer() {
        return optimizer_->getOptimizer();
    }
    GaussianTensors* getGaussians() {
        return gaussians_;
    }
    const Config& getConfig() const {
        return config;
    }

    // Helper methods that policies can use
    void duplicateSplats(const torch::Tensor& is_duplicated);
    void splitSplats(torch::Tensor& is_split);
    void removeSplats(const torch::Tensor& is_prune);

private:
    void updateState(rendering::RasterizationOutput& r_output);

    void growSplats(int iter) {
        grow_policy_(this, iter);
    }

    void pruneSplats(int iter) {
        prune_policy_(this, iter);
    }

    bool isRefining(int iter) const {
        // Don't refine at iteration 0 or before refine_start_iteration
        if (iter < config.refine_start_iteration) {
            return false;
        }
        return iter % config.reset_after_iterations == 0;
    }

    // store some state tensors which are used in prune/densify
    // that need to be retained across iterations
    torch::Tensor grad2d_;  // accumulated 2D gradients over all iterations
    torch::Tensor radii_;   // maximum pixel space of gaussians observed so far
    torch::Tensor count_;   // counter for number of times each gaussian was observed

    std::unique_ptr<optimization::Optimizer> optimizer_;
    std::unique_ptr<optimization::Scheduler> scheduler_;
    GaussianTensors* gaussians_;

    GrowPolicy grow_policy_;
    PrunePolicy prune_policy_;
};

// Type aliases for different strategies
using DefaultStrategy = Strategy<DefaultGrowPolicy, DefaultPrunePolicy>;
using MCMCStrategy = Strategy<MCMCGrowPolicy, MCMCPrunePolicy>;

}  // namespace optimization

}  // namespace gaussian_splatting
