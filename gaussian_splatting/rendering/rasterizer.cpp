#include "rasterizer.hpp"
#include <logging/logging.hpp>
#include <algorithm>
#include <cmath>

namespace gaussian_splatting {
namespace rendering {

DifferentiableRasterizer::DifferentiableRasterizer(const RasterizationConfig& config) 
    : config_(config) {
    LOG(INFO) << "Initializing DifferentiableRasterizer with resolution: " 
              << config_.image_width << "x" << config_.image_height
              << " on device: " << (config_.device.is_cuda() ? "CUDA" : "CPU");
}

RasterizationOutput DifferentiableRasterizer::rasterize(const RasterizationInput& input) {
    if (!input.isValid()) {
        LOG(ERROR) << "Invalid rasterization input";
        return RasterizationOutput{};
    }
    
    LOG(INFO) << "Rasterizing " << input.getNumSplats() << " splats";
    
    try {
        // Project 3D splats to 2D screen space
        torch::Tensor positions_2d = projectSplats(
            input.splat_positions, input.camera_pose, input.camera_intrinsics);
        
        // Compute 2D covariance matrices for projected splats
        torch::Tensor covariances_2d = compute2DCovariance(
            input.splat_covariances, input.camera_pose, 
            input.camera_intrinsics, input.splat_positions);
        
        // Perform culling operations
        torch::Tensor visibility_mask = torch::ones({input.getNumSplats()}, 
                                                   torch::dtype(torch::kBool).device(config_.device));
        
        if (config_.enable_frustum_culling) {
            torch::Tensor frustum_mask = frustumCulling(input.splat_positions, input.camera_pose);
            visibility_mask = visibility_mask & frustum_mask;
        }
        
        if (config_.enable_depth_culling) {
            torch::Tensor depth_mask = depthCulling(input.splat_positions, input.camera_pose);
            visibility_mask = visibility_mask & depth_mask;
        }
        
        // Filter visible splats
        torch::Tensor visible_indices = torch::nonzero(visibility_mask).squeeze(-1);
        torch::Tensor visible_positions_2d = positions_2d.index_select(0, visible_indices);
        torch::Tensor visible_covariances_2d = covariances_2d.index_select(0, visible_indices);
        torch::Tensor visible_colors = input.splat_colors.index_select(0, visible_indices);
        torch::Tensor visible_opacities = input.splat_opacities.index_select(0, visible_indices);
        
        // Compute depths for sorting
        torch::Tensor camera_positions = torch::matmul(input.camera_pose, 
            torch::cat({input.splat_positions, torch::ones({input.getNumSplats(), 1}, 
                       torch::dtype(torch::kFloat32).device(config_.device))}, 1).t()).t();
        torch::Tensor depths = camera_positions.select(1, 2);  // Z coordinate in camera space
        torch::Tensor visible_depths = depths.index_select(0, visible_indices);
        
        // Sort by depth (back to front for alpha blending)
        auto sorted_result = torch::sort(visible_depths, 0, true);  // descending
        torch::Tensor depth_indices = std::get<1>(sorted_result);
        
        visible_positions_2d = visible_positions_2d.index_select(0, depth_indices);
        visible_covariances_2d = visible_covariances_2d.index_select(0, depth_indices);
        visible_colors = visible_colors.index_select(0, depth_indices);
        visible_opacities = visible_opacities.index_select(0, depth_indices);
        visible_depths = visible_depths.index_select(0, depth_indices);
        
        LOG(INFO) << "Rendering " << visible_indices.size(0) << " visible splats after culling";
        
        // Perform tile-based rasterization
        torch::Tensor rendered_image = tileBasedRasterization(
            visible_positions_2d, visible_covariances_2d, 
            visible_colors, visible_opacities, visible_depths);
        
        // Create depth and alpha buffers
        torch::Tensor depth_buffer = torch::zeros({config_.image_height, config_.image_width}, 
                                                 torch::dtype(torch::kFloat32).device(config_.device));
        torch::Tensor alpha_buffer = torch::zeros({config_.image_height, config_.image_width}, 
                                                 torch::dtype(torch::kFloat32).device(config_.device));
        
        RasterizationOutput output;
        output.rendered_image = rendered_image;
        output.depth_buffer = depth_buffer;
        output.alpha_buffer = alpha_buffer;
        output.visibility_mask = visibility_mask;
        
        LOG(INFO) << "Rasterization completed successfully";
        return output;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "Error during rasterization: " << e.what();
        return RasterizationOutput{};
    }
}

torch::Tensor DifferentiableRasterizer::projectSplats(
    const torch::Tensor& positions_3d,
    const torch::Tensor& camera_pose,
    const torch::Tensor& camera_intrinsics) {
    
    // Transform 3D points to camera coordinate system
    torch::Tensor homogeneous_positions = torch::cat({
        positions_3d, 
        torch::ones({positions_3d.size(0), 1}, positions_3d.options())
    }, 1);
    
    // Apply camera pose transformation (world to camera)
    torch::Tensor camera_inverse = torch::inverse(camera_pose);
    torch::Tensor camera_positions = torch::matmul(homogeneous_positions, camera_inverse.t());
    
    // Extract 3D coordinates in camera frame
    torch::Tensor camera_coords = camera_positions.slice(1, 0, 3);
    
    // Project to 2D using camera intrinsics
    torch::Tensor z_coords = camera_coords.select(1, 2);
    torch::Tensor xy_normalized = camera_coords.slice(1, 0, 2) / z_coords.unsqueeze(1);
    
    // Apply camera matrix
    torch::Tensor homogeneous_2d = torch::cat({
        xy_normalized,
        torch::ones({xy_normalized.size(0), 1}, xy_normalized.options())
    }, 1);
    
    torch::Tensor projected_2d = torch::matmul(homogeneous_2d, camera_intrinsics.t());
    
    return projected_2d.slice(1, 0, 2);  // Return only x, y coordinates
}

torch::Tensor DifferentiableRasterizer::compute2DCovariance(
    const torch::Tensor& covariances_3d,
    const torch::Tensor& camera_pose,
    const torch::Tensor& camera_intrinsics,
    const torch::Tensor& positions_3d) {
    
    int num_splats = covariances_3d.size(0);
    torch::Tensor covariances_2d = torch::zeros({num_splats, 2, 2}, covariances_3d.options());
    
    // Get camera rotation and translation
    torch::Tensor R_cam = camera_pose.slice(0, 0, 3).slice(1, 0, 3);
    torch::Tensor t_cam = camera_pose.slice(0, 0, 3).select(1, 3);
    
    for (int i = 0; i < num_splats; ++i) {
        // Transform 3D covariance to camera frame
        torch::Tensor cov_3d = covariances_3d[i];
        torch::Tensor cov_camera = torch::matmul(torch::matmul(R_cam, cov_3d), R_cam.t());
        
        // Project to 2D using Jacobian of projection
        torch::Tensor pos_3d = positions_3d[i];
        torch::Tensor pos_camera = torch::matmul(R_cam, pos_3d) + t_cam;
        
        float z = pos_camera[2].item<float>();
        if (z <= 0) continue;  // Behind camera
        
        // Jacobian matrix for perspective projection
        torch::Tensor J = torch::zeros({2, 3}, covariances_3d.options());
        float fx = camera_intrinsics[0][0].item<float>();
        float fy = camera_intrinsics[1][1].item<float>();
        
        J[0][0] = fx / z;
        J[0][2] = -fx * pos_camera[0].item<float>() / (z * z);
        J[1][1] = fy / z;
        J[1][2] = -fy * pos_camera[1].item<float>() / (z * z);
        
        // Project covariance: Σ_2D = J * Σ_3D * J^T
        covariances_2d[i] = torch::matmul(torch::matmul(J, cov_camera), J.t());
    }
    
    return covariances_2d;
}

torch::Tensor DifferentiableRasterizer::frustumCulling(
    const torch::Tensor& positions_3d, const torch::Tensor& camera_pose) const {
    
    // Transform positions to camera coordinate system
    torch::Tensor homogeneous_positions = torch::cat({
        positions_3d, 
        torch::ones({positions_3d.size(0), 1}, positions_3d.options())
    }, 1);
    
    torch::Tensor camera_inverse = torch::inverse(camera_pose);
    torch::Tensor camera_positions = torch::matmul(homogeneous_positions, camera_inverse.t());
    torch::Tensor z_coords = camera_positions.select(1, 2);
    
    // Basic frustum culling - check if points are in front of camera
    torch::Tensor mask = (z_coords > config_.near_plane) & (z_coords < config_.far_plane);
    
    return mask;
}

torch::Tensor DifferentiableRasterizer::depthCulling(
    const torch::Tensor& positions_3d, const torch::Tensor& camera_pose) const {
    
    // Simple depth culling - remove points that are too far
    torch::Tensor homogeneous_positions = torch::cat({
        positions_3d, 
        torch::ones({positions_3d.size(0), 1}, positions_3d.options())
    }, 1);
    
    torch::Tensor camera_inverse = torch::inverse(camera_pose);
    torch::Tensor camera_positions = torch::matmul(homogeneous_positions, camera_inverse.t());
    torch::Tensor distances = torch::norm(camera_positions.slice(1, 0, 3), 2, 1);
    
    torch::Tensor mask = distances < config_.far_plane;
    
    return mask;
}

torch::Tensor DifferentiableRasterizer::tileBasedRasterization(
    const torch::Tensor& positions_2d,
    const torch::Tensor& covariances_2d,
    const torch::Tensor& colors,
    const torch::Tensor& opacities,
    const torch::Tensor& depths) const {
    
    int tiles_x = static_cast<int>(std::ceil(config_.image_width / config_.tile_size));
    int tiles_y = static_cast<int>(std::ceil(config_.image_height / config_.tile_size));
    
    LOG(INFO) << "Using " << tiles_x << "x" << tiles_y << " tiles for rasterization";
    
    // Initialize output image
    torch::Tensor rendered_image = torch::zeros({3, config_.image_height, config_.image_width}, 
                                               torch::dtype(torch::kFloat32).device(config_.device));
    
    TileRasterizer tile_rasterizer(config_);
    
    // Process each tile
    for (int tile_y = 0; tile_y < tiles_y; ++tile_y) {
        for (int tile_x = 0; tile_x < tiles_x; ++tile_x) {
            
            // Calculate tile bounds
            int x_start = tile_x * static_cast<int>(config_.tile_size);
            int y_start = tile_y * static_cast<int>(config_.tile_size);
            int x_end = std::min(x_start + static_cast<int>(config_.tile_size), config_.image_width);
            int y_end = std::min(y_start + static_cast<int>(config_.tile_size), config_.image_height);
            
            // Find splats that affect this tile
            torch::Tensor x_coords = positions_2d.select(1, 0);
            torch::Tensor y_coords = positions_2d.select(1, 1);
            
            torch::Tensor tile_mask = (x_coords >= x_start) & (x_coords < x_end) & 
                                     (y_coords >= y_start) & (y_coords < y_end);
            
            torch::Tensor tile_indices = torch::nonzero(tile_mask).squeeze(-1);
            
            if (tile_indices.numel() == 0) continue;  // No splats in this tile
            
            // Extract tile-specific data
            torch::Tensor tile_positions = positions_2d.index_select(0, tile_indices);
            torch::Tensor tile_covariances = covariances_2d.index_select(0, tile_indices);
            torch::Tensor tile_colors = colors.index_select(0, tile_indices);
            torch::Tensor tile_opacities = opacities.index_select(0, tile_indices);
            
            // Rasterize this tile
            torch::Tensor tile_result = tile_rasterizer.rasterizeTile(
                tile_positions, tile_colors, tile_opacities, tile_covariances, tile_x, tile_y);
            
            // Copy tile result to output image
            if (tile_result.defined() && tile_result.numel() > 0) {
                rendered_image.slice(1, y_start, y_end).slice(2, x_start, x_end) = 
                    tile_result.slice(1, 0, y_end - y_start).slice(2, 0, x_end - x_start);
            }
        }
    }
    
    return rendered_image;
}

torch::Tensor DifferentiableRasterizer::computeGaussianWeights(
    const torch::Tensor& pixel_coords,
    const torch::Tensor& splat_center,
    const torch::Tensor& covariance_2d) const {
    
    // Compute offset from splat center
    torch::Tensor offset = pixel_coords - splat_center.unsqueeze(0);
    
    // Compute Gaussian weight: exp(-0.5 * offset^T * Σ^(-1) * offset)
    torch::Tensor cov_inv = torch::inverse(covariance_2d + 1e-6 * torch::eye(2, covariance_2d.options()));
    torch::Tensor mahalanobis = torch::sum(offset * torch::matmul(offset, cov_inv), 1);
    torch::Tensor weights = torch::exp(-0.5 * mahalanobis);
    
    return weights;
}

torch::Tensor DifferentiableRasterizer::alphaBlending(
    const torch::Tensor& colors,
    const torch::Tensor& alphas,
    const torch::Tensor& weights) const {
    
    // Alpha blending with depth ordering
    torch::Tensor effective_alphas = alphas * weights;
    torch::Tensor transmittance = torch::cumprod(1.0 - effective_alphas, 0);
    torch::Tensor alpha_weights = effective_alphas * torch::cat({
        torch::ones({1}, effective_alphas.options()),
        transmittance.slice(0, 0, -1)
    }, 0);
    
    torch::Tensor blended_color = torch::sum(colors * alpha_weights.unsqueeze(1), 0);
    
    return blended_color;
}

// TileRasterizer implementation
TileRasterizer::TileRasterizer(const RasterizationConfig& config) 
    : config_(config), tile_size_(static_cast<int>(config.tile_size)) {
}

torch::Tensor TileRasterizer::rasterizeTile(
    const torch::Tensor& tile_splats,
    const torch::Tensor& tile_colors,
    const torch::Tensor& tile_opacities,
    const torch::Tensor& tile_covariances,
    int tile_x, int tile_y) const {
    
    int x_start = tile_x * tile_size_;
    int y_start = tile_y * tile_size_;
    int x_end = std::min(x_start + tile_size_, config_.image_width);
    int y_end = std::min(y_start + tile_size_, config_.image_height);
    
    int tile_width = x_end - x_start;
    int tile_height = y_end - y_start;
    
    // Initialize tile output
    torch::Tensor tile_image = torch::zeros({3, tile_height, tile_width}, 
                                           torch::dtype(torch::kFloat32).device(config_.device));
    
    if (tile_splats.size(0) == 0) return tile_image;
    
    // Generate pixel coordinates for this tile
    auto y_indices = torch::arange(y_start, y_end, torch::dtype(torch::kFloat32).device(config_.device));
    auto x_indices = torch::arange(x_start, x_end, torch::dtype(torch::kFloat32).device(config_.device));
    auto meshgrid_result = torch::meshgrid({y_indices, x_indices});
    torch::Tensor pixel_y = meshgrid_result[0].flatten();
    torch::Tensor pixel_x = meshgrid_result[1].flatten();
    torch::Tensor pixel_coords = torch::stack({pixel_x, pixel_y}, 1);
    
    // Render each splat in the tile
    torch::Tensor accumulated_alpha = torch::zeros({tile_height * tile_width}, 
                                                  torch::dtype(torch::kFloat32).device(config_.device));
    torch::Tensor accumulated_color = torch::zeros({3, tile_height * tile_width}, 
                                                   torch::dtype(torch::kFloat32).device(config_.device));
    
    for (int i = 0; i < tile_splats.size(0); ++i) {
        torch::Tensor splat_center = tile_splats[i];
        torch::Tensor splat_color = tile_colors[i];
        torch::Tensor splat_opacity = tile_opacities[i];
        torch::Tensor splat_cov = tile_covariances[i];
        
        // Compute Gaussian weights for all pixels
        torch::Tensor offset = pixel_coords - splat_center.unsqueeze(0);
        torch::Tensor cov_inv = torch::inverse(splat_cov + 1e-6 * torch::eye(2, splat_cov.options()));
        torch::Tensor mahalanobis = torch::sum(offset * torch::matmul(offset, cov_inv), 1);
        torch::Tensor gaussian_weights = torch::exp(-0.5 * mahalanobis);
        
        // Apply opacity
        torch::Tensor effective_alpha = splat_opacity.item<float>() * gaussian_weights;
        
        // Alpha blending
        torch::Tensor transmittance = 1.0 - accumulated_alpha;
        torch::Tensor alpha_contribution = effective_alpha * transmittance;
        
        accumulated_color += splat_color.unsqueeze(1) * alpha_contribution.unsqueeze(0);
        accumulated_alpha += alpha_contribution;
        
        // Early termination if tile is fully opaque
        if (torch::all(accumulated_alpha > 0.99).item<bool>()) {
            break;
        }
    }
    
    // Reshape to tile dimensions
    tile_image = accumulated_color.view({3, tile_height, tile_width});
    
    return tile_image;
}

// Utility functions
torch::Tensor convertGaussianSplatsToTensors(
    const std::vector<core::types::GaussianSplat>& splats,
    torch::Device device) {
    
    if (splats.empty()) {
        return torch::empty({0, 3}, torch::dtype(torch::kFloat32).device(device));
    }
    
    torch::Tensor positions = torch::zeros({static_cast<int>(splats.size()), 3}, 
                                          torch::dtype(torch::kFloat32).device(device));
    
    for (size_t i = 0; i < splats.size(); ++i) {
        positions[i][0] = splats[i].position.x();
        positions[i][1] = splats[i].position.y();
        positions[i][2] = splats[i].position.z();
    }
    
    return positions;
}

RasterizationInput prepareSplatsForRasterization(
    const std::vector<core::types::GaussianSplat>& splats,
    const torch::Tensor& camera_pose,
    const torch::Tensor& camera_intrinsics,
    torch::Device device) {
    
    RasterizationInput input;
    
    if (splats.empty()) {
        LOG(WARNING) << "No splats provided for rasterization";
        return input;
    }
    
    int num_splats = static_cast<int>(splats.size());
    
    // Convert positions
    input.splat_positions = torch::zeros({num_splats, 3}, torch::dtype(torch::kFloat32).device(device));
    input.splat_colors = torch::zeros({num_splats, 3}, torch::dtype(torch::kFloat32).device(device));
    input.splat_opacities = torch::zeros({num_splats, 1}, torch::dtype(torch::kFloat32).device(device));
    input.splat_covariances = torch::zeros({num_splats, 3, 3}, torch::dtype(torch::kFloat32).device(device));
    
    for (int i = 0; i < num_splats; ++i) {
        const auto& splat = splats[i];
        
        // Position
        input.splat_positions[i][0] = splat.position.x();
        input.splat_positions[i][1] = splat.position.y();
        input.splat_positions[i][2] = splat.position.z();
        
        // Color
        input.splat_colors[i][0] = splat.color.x();
        input.splat_colors[i][1] = splat.color.y();
        input.splat_colors[i][2] = splat.color.z();
        
        // Opacity
        input.splat_opacities[i][0] = splat.opacity;
        
        // Covariance
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                input.splat_covariances[i][row][col] = splat.covariance(row, col);
            }
        }
    }
    
    input.camera_pose = camera_pose.to(device);
    input.camera_intrinsics = camera_intrinsics.to(device);
    
    return input;
}

} // namespace rendering
} // namespace gaussian_splatting