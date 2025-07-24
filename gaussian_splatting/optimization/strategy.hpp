#pragma once

#include <torch/torch.h>
#include <memory>
#include "gaussian_splatting/optimization/optimizer.hpp"
#include "gaussian_splatting/optimization/scheduler.hpp"
#include "gaussian_splatting/rendering/rasterizer.hpp"

namespace gaussian_splatting {
namespace optimization {

class Strategy {
public:
    static struct Config {
        int max_iterations = 1000;        // set from the training config rather than from here
        int reset_after_iterations = 10;  // Increased from 10 to reduce densification frequency
        int refine_start_iteration = 10;  // Don't densify before this iteration
        int64_t max_splat_count = 75000;  // Maximum number of splats to prevent OOM
        double grad_threshold = 0.0002;   // from the paper
        double grow_scale3d = 0.1;        // from the paper
        double grow_scale2d = 0.1;        // from the paper
        double prune_opacity = 0.005;
        double prune_scale3d = 0.1;
        double prune_scale2d = 0.15;
    } config;

    // Not really required since in the gsplat code it only
    // calls info[self.key_for_gradient].retain_grad(). We
    // have means2d tensor which should retain its grad?
    // void stepPreBackward();
    Strategy(std::unique_ptr<Optimizer> optimizer, std::unique_ptr<Scheduler> scheduler,
             GaussianTensors* gaussians)
        : optimizer_(std::move(optimizer)),
          scheduler_(std::move(scheduler)),
          gaussians_(gaussians) {}

    void postBackward(rendering::RasterizationOutput& r_output, int iter);
    void step(int iter);

private:
    void updateState(rendering::RasterizationOutput& r_output);

    void growSplats(int iter);
    void duplicateSplats(const torch::Tensor& is_duplicated);
    void splitSplats(torch::Tensor& is_split);

    void pruneSplats(int iter);
    void removeSplats(const torch::Tensor& is_prune);

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
};

}  // namespace optimization

}  // namespace gaussian_splatting
