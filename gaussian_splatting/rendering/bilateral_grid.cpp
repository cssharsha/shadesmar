#include "bilateral_grid.hpp"
#include <logging/logging.hpp>

namespace gaussian_splatting {
namespace rendering {

BilateralGrid::BilateralGrid(const BilateralGridConfig& config, torch::Device device)
    : config_(config), device_(device) {
    
    LOG(INFO) << "Initializing BilateralGrid: " << config_.grid_width << "x" 
              << config_.grid_height << "x" << config_.grid_depth;
    
    // Initialize grid coefficients
    auto opts = torch::TensorOptions().device(device_).dtype(torch::kFloat32);
    grid_coeffs_ = torch::zeros({config_.grid_depth, config_.grid_height, config_.grid_width, 12}, opts);
    
    // Initialize identity transforms
    for (int d = 0; d < config_.grid_depth; ++d) {
        for (int h = 0; h < config_.grid_height; ++h) {
            for (int w = 0; w < config_.grid_width; ++w) {
                // Identity matrix (3x3)
                grid_coeffs_[d][h][w][0] = 1.0f; // r->r
                grid_coeffs_[d][h][w][4] = 1.0f; // g->g
                grid_coeffs_[d][h][w][8] = 1.0f; // b->b
                // Bias terms remain zero
            }
        }
    }
    
    grid_coeffs_.requires_grad_(true);
}

torch::Tensor BilateralGrid::forward(const torch::Tensor& input_image, 
                                    const torch::Tensor& guide_image) {
    if (input_image.dim() != 4 || guide_image.dim() != 4) {
        LOG(ERROR) << "Input tensors must be 4D (batch, channels, height, width)";
        return torch::Tensor();
    }
    
    // Compute bilateral coordinates
    auto coords = computeBilateralCoordinates(guide_image);
    
    // Interpolate grid coefficients
    auto interpolated_coeffs = trilinearInterpolate(grid_coeffs_, coords);
    
    // Apply per-pixel affine transformation
    auto output = applyAffineTransform(input_image, interpolated_coeffs);
    
    return output;
}

torch::Tensor BilateralGrid::backward(const torch::Tensor& grad_output,
                                     const torch::Tensor& input_image,
                                     const torch::Tensor& guide_image) {
    // Compute bilateral coordinates
    auto coords = computeBilateralCoordinates(guide_image);
    
    // Interpolate grid coefficients
    auto interpolated_coeffs = trilinearInterpolate(grid_coeffs_, coords);
    
    // Compute gradients with respect to input
    auto grad_input = applyAffineTransform(grad_output, interpolated_coeffs);
    
    return grad_input;
}

void BilateralGrid::initializeGrid(const torch::Tensor& reference_image) {
    LOG(INFO) << "Initializing bilateral grid with reference image";
    
    // Reset to identity transforms with small random perturbations
    auto opts = torch::TensorOptions().device(device_).dtype(torch::kFloat32);
    grid_coeffs_ = torch::zeros({config_.grid_depth, config_.grid_height, config_.grid_width, 12}, opts);
    
    for (int d = 0; d < config_.grid_depth; ++d) {
        for (int h = 0; h < config_.grid_height; ++h) {
            for (int w = 0; w < config_.grid_width; ++w) {
                // Identity matrix with small noise
                grid_coeffs_[d][h][w][0] = 1.0f + torch::randn({1}).item<float>() * 0.01f;
                grid_coeffs_[d][h][w][4] = 1.0f + torch::randn({1}).item<float>() * 0.01f;
                grid_coeffs_[d][h][w][8] = 1.0f + torch::randn({1}).item<float>() * 0.01f;
            }
        }
    }
    
    grid_coeffs_.requires_grad_(true);
}

void BilateralGrid::setGridCoefficients(const torch::Tensor& coeffs) {
    if (coeffs.sizes() != grid_coeffs_.sizes()) {
        LOG(ERROR) << "Grid coefficient size mismatch";
        return;
    }
    
    grid_coeffs_ = coeffs.clone();
    grid_coeffs_.requires_grad_(true);
}

torch::Tensor BilateralGrid::computeRegularizationLoss() const {
    // Total variation regularization
    auto tv_loss = torch::zeros({1}, torch::TensorOptions().device(device_));
    
    // Spatial regularization (adjacent grid cells should be similar)
    if (config_.grid_width > 1) {
        auto diff_w = grid_coeffs_.slice(2, 1, config_.grid_width) - 
                      grid_coeffs_.slice(2, 0, config_.grid_width - 1);
        tv_loss += diff_w.pow(2).sum();
    }
    
    if (config_.grid_height > 1) {
        auto diff_h = grid_coeffs_.slice(1, 1, config_.grid_height) - 
                      grid_coeffs_.slice(1, 0, config_.grid_height - 1);
        tv_loss += diff_h.pow(2).sum();
    }
    
    if (config_.grid_depth > 1) {
        auto diff_d = grid_coeffs_.slice(0, 1, config_.grid_depth) - 
                      grid_coeffs_.slice(0, 0, config_.grid_depth - 1);
        tv_loss += diff_d.pow(2).sum();
    }
    
    return tv_loss * config_.regularization_weight;
}

torch::Tensor BilateralGrid::computeBilateralCoordinates(const torch::Tensor& guide_image) const {
    auto coords_pair = computeCoordinates(guide_image);
    auto spatial_coords = coords_pair.first;
    auto luma_coords = coords_pair.second;
    
    // Combine spatial and luma coordinates
    auto bilateral_coords = torch::cat({spatial_coords, luma_coords.unsqueeze(-1)}, -1);
    
    return bilateral_coords;
}

std::pair<torch::Tensor, torch::Tensor> BilateralGrid::computeCoordinates(const torch::Tensor& guide_image) const {
    int batch_size = guide_image.size(0);
    int height = guide_image.size(2);
    int width = guide_image.size(3);
    
    auto opts = torch::TensorOptions().device(device_).dtype(torch::kFloat32);
    
    // Spatial coordinates
    auto y_coords = torch::linspace(0, config_.grid_height - 1, height, opts);
    auto x_coords = torch::linspace(0, config_.grid_width - 1, width, opts);
    
    auto meshgrid = torch::meshgrid({y_coords, x_coords}, "ij");
    auto y_grid = meshgrid[0].unsqueeze(0).repeat({batch_size, 1, 1});
    auto x_grid = meshgrid[1].unsqueeze(0).repeat({batch_size, 1, 1});
    
    auto spatial_coords = torch::stack({y_grid, x_grid}, -1);
    
    // Luma coordinates (RGB to luma conversion)
    auto luma = 0.299f * guide_image.select(1, 0) + 
                0.587f * guide_image.select(1, 1) + 
                0.114f * guide_image.select(1, 2);
    
    // Normalize luma to grid depth
    auto luma_coords = luma * (config_.grid_depth - 1);
    
    return {spatial_coords, luma_coords};
}

torch::Tensor BilateralGrid::trilinearInterpolate(const torch::Tensor& grid, 
                                                 const torch::Tensor& coordinates) const {
    // Extract coordinates
    auto y_coords = coordinates.select(-1, 0);
    auto x_coords = coordinates.select(-1, 1);
    auto z_coords = coordinates.select(-1, 2);
    
    // Clamp coordinates to grid bounds
    y_coords = torch::clamp(y_coords, 0, config_.grid_height - 1);
    x_coords = torch::clamp(x_coords, 0, config_.grid_width - 1);
    z_coords = torch::clamp(z_coords, 0, config_.grid_depth - 1);
    
    // Floor coordinates for interpolation
    auto y0 = y_coords.floor().to(torch::kLong);
    auto x0 = x_coords.floor().to(torch::kLong);
    auto z0 = z_coords.floor().to(torch::kLong);
    
    auto y1 = torch::clamp(y0 + 1, 0, config_.grid_height - 1);
    auto x1 = torch::clamp(x0 + 1, 0, config_.grid_width - 1);
    auto z1 = torch::clamp(z0 + 1, 0, config_.grid_depth - 1);
    
    // Interpolation weights
    auto wy = y_coords - y0.to(torch::kFloat32);
    auto wx = x_coords - x0.to(torch::kFloat32);
    auto wz = z_coords - z0.to(torch::kFloat32);
    
    // Perform trilinear interpolation
    auto c000 = grid.index({z0, y0, x0});
    auto c001 = grid.index({z0, y0, x1});
    auto c010 = grid.index({z0, y1, x0});
    auto c011 = grid.index({z0, y1, x1});
    auto c100 = grid.index({z1, y0, x0});
    auto c101 = grid.index({z1, y0, x1});
    auto c110 = grid.index({z1, y1, x0});
    auto c111 = grid.index({z1, y1, x1});
    
    // Interpolate along x
    auto c00 = c000 * (1 - wx.unsqueeze(-1)) + c001 * wx.unsqueeze(-1);
    auto c01 = c010 * (1 - wx.unsqueeze(-1)) + c011 * wx.unsqueeze(-1);
    auto c10 = c100 * (1 - wx.unsqueeze(-1)) + c101 * wx.unsqueeze(-1);
    auto c11 = c110 * (1 - wx.unsqueeze(-1)) + c111 * wx.unsqueeze(-1);
    
    // Interpolate along y
    auto c0 = c00 * (1 - wy.unsqueeze(-1)) + c01 * wy.unsqueeze(-1);
    auto c1 = c10 * (1 - wy.unsqueeze(-1)) + c11 * wy.unsqueeze(-1);
    
    // Interpolate along z
    auto result = c0 * (1 - wz.unsqueeze(-1)) + c1 * wz.unsqueeze(-1);
    
    return result;
}

torch::Tensor BilateralGrid::applyAffineTransform(const torch::Tensor& input,
                                                 const torch::Tensor& coefficients) const {
    int batch_size = input.size(0);
    int channels = input.size(1);
    int height = input.size(2);
    int width = input.size(3);
    
    // Reshape input for matrix operations
    auto input_flat = input.permute({0, 2, 3, 1}).reshape({batch_size * height * width, channels});
    
    // Extract affine matrix and bias
    auto affine_matrix = coefficients.slice(-1, 0, 9).reshape({batch_size, height, width, 3, 3});
    auto bias = coefficients.slice(-1, 9, 12).reshape({batch_size, height, width, 3});
    
    // Flatten for batch operations
    affine_matrix = affine_matrix.reshape({batch_size * height * width, 3, 3});
    bias = bias.reshape({batch_size * height * width, 3});
    
    // Apply affine transformation: y = Ax + b
    auto output_flat = torch::bmm(affine_matrix, input_flat.unsqueeze(-1)).squeeze(-1) + bias;
    
    // Reshape back to image format
    auto output = output_flat.reshape({batch_size, height, width, channels}).permute({0, 3, 1, 2});
    
    return output;
}

} // namespace rendering
} // namespace gaussian_splatting