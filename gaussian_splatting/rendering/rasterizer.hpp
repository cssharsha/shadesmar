#pragma once

#include <torch/torch.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

#include "core/types/gaussian_splat.hpp"

namespace gaussian_splatting {
namespace rendering {

struct RasterizationConfig {
    int image_width = 512;
    int image_height = 384;
    float near_plane = 0.1f;
    float far_plane = 100.0f;
    bool enable_depth_culling = true;
    bool enable_frustum_culling = true;
    float tile_size = 16.0f;  // Tile-based rasterization
    torch::Device device = torch::kCPU;
    
    RasterizationConfig() = default;
    RasterizationConfig(int w, int h, torch::Device dev) 
        : image_width(w), image_height(h), device(dev) {}
};

struct RasterizationInput {
    torch::Tensor splat_positions;     // [N, 3] - 3D positions
    torch::Tensor splat_colors;       // [N, 3] - RGB colors
    torch::Tensor splat_opacities;    // [N, 1] - opacity values
    torch::Tensor splat_covariances;  // [N, 3, 3] - 3D covariance matrices
    torch::Tensor camera_pose;        // [4, 4] - camera pose matrix
    torch::Tensor camera_intrinsics;  // [3, 3] - camera K matrix
    
    bool isValid() const {
        return splat_positions.defined() && splat_colors.defined() && 
               splat_opacities.defined() && splat_covariances.defined() &&
               camera_pose.defined() && camera_intrinsics.defined();
    }
    
    int getNumSplats() const {
        return splat_positions.size(0);
    }
};

struct RasterizationOutput {
    torch::Tensor rendered_image;     // [3, H, W] - rendered RGB image
    torch::Tensor depth_buffer;      // [H, W] - depth values
    torch::Tensor alpha_buffer;      // [H, W] - accumulated alpha
    torch::Tensor visibility_mask;   // [N] - which splats are visible
    
    bool isValid() const {
        return rendered_image.defined() && depth_buffer.defined() && 
               alpha_buffer.defined() && visibility_mask.defined();
    }
};

class DifferentiableRasterizer {
public:
    explicit DifferentiableRasterizer(const RasterizationConfig& config);
    ~DifferentiableRasterizer() = default;
    
    RasterizationOutput rasterize(const RasterizationInput& input);
    
    void setConfig(const RasterizationConfig& config) { config_ = config; }
    const RasterizationConfig& getConfig() const { return config_; }
    
    static torch::Tensor projectSplats(const torch::Tensor& positions_3d,
                                       const torch::Tensor& camera_pose,
                                       const torch::Tensor& camera_intrinsics);
    
    static torch::Tensor compute2DCovariance(const torch::Tensor& covariances_3d,
                                             const torch::Tensor& camera_pose,
                                             const torch::Tensor& camera_intrinsics,
                                             const torch::Tensor& positions_3d);
    
private:
    RasterizationConfig config_;
    
    torch::Tensor frustumCulling(const torch::Tensor& positions_3d,
                                 const torch::Tensor& camera_pose) const;
    
    torch::Tensor depthCulling(const torch::Tensor& positions_3d,
                               const torch::Tensor& camera_pose) const;
    
    torch::Tensor tileBasedRasterization(const torch::Tensor& positions_2d,
                                         const torch::Tensor& covariances_2d,
                                         const torch::Tensor& colors,
                                         const torch::Tensor& opacities,
                                         const torch::Tensor& depths) const;
    
    torch::Tensor computeGaussianWeights(const torch::Tensor& pixel_coords,
                                         const torch::Tensor& splat_center,
                                         const torch::Tensor& covariance_2d) const;
    
    torch::Tensor alphaBlending(const torch::Tensor& colors,
                                const torch::Tensor& alphas,
                                const torch::Tensor& weights) const;
};

class TileRasterizer {
public:
    explicit TileRasterizer(const RasterizationConfig& config);
    
    torch::Tensor rasterizeTile(const torch::Tensor& tile_splats,
                                const torch::Tensor& tile_colors,
                                const torch::Tensor& tile_opacities,
                                const torch::Tensor& tile_covariances,
                                int tile_x, int tile_y) const;
    
private:
    RasterizationConfig config_;
    int tile_size_;
};

torch::Tensor convertGaussianSplatsToTensors(
    const std::vector<core::types::GaussianSplat>& splats,
    torch::Device device);

RasterizationInput prepareSplatsForRasterization(
    const std::vector<core::types::GaussianSplat>& splats,
    const torch::Tensor& camera_pose,
    const torch::Tensor& camera_intrinsics,
    torch::Device device);

} // namespace rendering
} // namespace gaussian_splatting