#pragma once

namespace gaussian_splatting {
namespace optimization {

namespace strategy {
struct Config {
    int max_iterations = 1000;        // set from the training config rather than from here
    int refine_start_iteration = 50;  // Don't densify before this iteration
    int refine_every = 100;           // Densify every this many iterations
    int stop_refine = 1000;           // Stop refining after this iteration
    int stop_refine_scale2d = 37500;
    int reset_after_iterations = 100;
    int max_splat_count = 75000;     // Maximum number of splats to prevent OOM
    double grad_threshold = 0.0002;  // from the paper
    double grow_scale3d = 0.1;       // from the paper
    double grow_scale2d = 0.1;       // from the paper
    double prune_opacity = 0.005;
    double prune_scale3d = 0.1;
    double prune_scale2d = 0.15;
    bool revised_opacity = true;
};
}  // namespace strategy

namespace optimizer {

static struct Config {
    int max_iterations = 100;
    float learning_rate = 0.01f;  // General LR, not used for gaussians
    // Standard learning rates from gsplat reference implementation
    // Note: positions_lr will be scaled by scene_scale in initialize()
    float positions_lr = 1.6e-4f;        // 0.00016 - scaled by scene_scale
    float rotations_lr = 1.0e-3f;        // 0.001
    float scales_lr = 5.0e-3f;           // 0.005
    float opacities_lr = 5.0e-2f;        // 0.05
    float sh_coefficients_lr = 2.5e-3f;  // 0.0025
    // Note: sh_N uses sh_coefficients_lr / 20 in initialize()
};
}  // namespace optimizer

}  // namespace optimization
}  // namespace gaussian_splatting
