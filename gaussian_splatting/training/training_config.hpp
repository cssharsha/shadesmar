#pragma once

#include <cstdint>

namespace gaussian_splatting {
namespace training {

struct TrainingConfig {
    // Conservative initial resolution
    int initial_width = 512;
    int initial_height = 384;
    
    // Batch processing
    int max_iterations_per_batch = 1000;
    int densification_interval = 100;
    int keyframes_per_batch = 10;
    
    // Memory management
    size_t max_gpu_memory_per_batch = 2ULL * 1024 * 1024 * 1024; // 2GB
    int max_gaussians_per_batch = 100000;
    
    // Loss function
    float d_ssim_lambda = 0.2f;
    float learning_rate = 0.01f;
    float opacity_threshold = 0.005f;
    
    // Optimization
    float position_lr = 0.00016f;
    float opacity_lr = 0.05f;
    float scaling_lr = 0.005f;
    float rotation_lr = 0.001f;
    float sh_lr = 0.0025f;
    
    // Spherical harmonics
    int sh_degree = 3;
    int max_sh_degree = 3;
    
    // Densification
    float densify_grad_threshold = 0.0002f;
    float densify_size_threshold = 20.0f;

    // Bounding box optimization
    float bounding_box_radius = 5.0f;  // Radius in meters for loading splats around keyframes
    int sync_interval = 10;             // Sync GPU→CPU every N iterations
    int visualization_interval = 20;    // Visualize every N*2 iterations

    // Validation
    bool isValid() const {
        return initial_width > 0 && initial_height > 0 &&
               max_iterations_per_batch > 0 && learning_rate > 0.0f &&
               bounding_box_radius > 0.0f && sync_interval > 0;
    }
};

} // namespace training
} // namespace gaussian_splatting