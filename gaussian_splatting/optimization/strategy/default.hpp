#pragma once

#include <torch/torch.h>
#include "gaussian_splatting/optimization/config.hpp"
#include "gaussian_splatting/optimization/optimizer.hpp"
#include "gaussian_splatting/rendering/rasterizer.hpp"
#include "gaussian_splatting/training/gaussian_tensors.hpp"

namespace gaussian_splatting {
namespace optimization {
namespace strategy {

// Default grow policy implementation
// Uses the original densification algorithm: duplicate small splats with high gradients,
// split large splats with high gradients
class Default {
public:
    Default(std::unique_ptr<Optimizer> optimizer) : optimizer_(std::move(optimizer)) {}
    void operator()(GaussianTensors* gaussians, rendering::RasterizationOutput& render_output,
                    int iter);
    void update(GaussianTensors* gaussians, rendering::RasterizationOutput& render_output);
    void growSplats(GaussianTensors* gaussians, int iter);
    void pruneSplats(GaussianTensors* gaussians, int iter);

    bool isRefining(int iter) const {
        // Don't refine at iteration 0 or before refine_start_iteration
        if (iter < config.refine_start_iteration) {
            return false;
        }
        return iter % config.refine_every == 0;
    }

    void step(int iter) {
        if (iter < config.max_iterations) {
            optimizer_->step();
        }
    }

private:
    strategy::Config config;
    torch::Tensor grads_;
    torch::Tensor radii_;
    torch::Tensor count_;
    std::unique_ptr<Optimizer> optimizer_;

    // Helper functions
    void duplicateSplats(GaussianTensors* gaussians, const torch::Tensor& is_duplicated);
    void splitSplats(GaussianTensors* gaussians, const torch::Tensor& is_split);
    void removeSplats(GaussianTensors* gaussians, const torch::Tensor& is_prune);
};

}  // namespace strategy
}  // namespace optimization
}  // namespace gaussian_splatting
