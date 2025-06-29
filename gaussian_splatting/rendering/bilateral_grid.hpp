#pragma once

#include <torch/torch.h>
#include "../training/training_config.hpp"

namespace gaussian_splatting {
namespace rendering {

struct BilateralGridConfig {
    int grid_width = 64;
    int grid_height = 48;
    int grid_depth = 8;
    float sigma_spatial = 16.0f;
    float sigma_luma = 8.0f;
    float regularization_weight = 0.01f;
};

class BilateralGrid {
public:
    explicit BilateralGrid(const BilateralGridConfig& config, torch::Device device);
    
    // Forward pass: apply bilateral grid to input image
    torch::Tensor forward(const torch::Tensor& input_image, 
                         const torch::Tensor& guide_image);
    
    // Backward pass with regularization
    torch::Tensor backward(const torch::Tensor& grad_output,
                          const torch::Tensor& input_image,
                          const torch::Tensor& guide_image);
    
    // Initialize grid coefficients
    void initializeGrid(const torch::Tensor& reference_image);
    
    // Get grid coefficients for optimization
    torch::Tensor getGridCoefficients() const { return grid_coeffs_; }
    
    // Set grid coefficients (for optimization)
    void setGridCoefficients(const torch::Tensor& coeffs);
    
    // Apply regularization loss
    torch::Tensor computeRegularizationLoss() const;
    
private:
    BilateralGridConfig config_;
    torch::Device device_;
    
    // Grid coefficients [grid_depth, grid_height, grid_width, 12]
    // 12 coefficients: 3x3 affine transform (9) + bias (3)
    torch::Tensor grid_coeffs_;
    
    // Compute bilateral coordinates
    torch::Tensor computeBilateralCoordinates(const torch::Tensor& guide_image) const;
    
    // Trilinear interpolation in bilateral space
    torch::Tensor trilinearInterpolate(const torch::Tensor& grid, 
                                      const torch::Tensor& coordinates) const;
    
    // Apply affine transformation per pixel
    torch::Tensor applyAffineTransform(const torch::Tensor& input,
                                      const torch::Tensor& coefficients) const;
    
    // Compute spatial and luma coordinates
    std::pair<torch::Tensor, torch::Tensor> computeCoordinates(const torch::Tensor& guide_image) const;
};

} // namespace rendering
} // namespace gaussian_splatting