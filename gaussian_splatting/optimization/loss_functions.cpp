#include "loss_functions.hpp"
#include <logging/logging.hpp>

namespace gaussian_splatting {
namespace optimization {

torch::Tensor LossFunctions::computeL1Loss(const torch::Tensor& rendered, const torch::Tensor& ground_truth) {
    return torch::l1_loss(rendered, ground_truth);
}

torch::Tensor LossFunctions::computeD_SSIMLoss(const torch::Tensor& rendered, const torch::Tensor& ground_truth) {
    auto ssim_value = computeSSIM(rendered, ground_truth);
    return (1.0f - ssim_value) / 2.0f; // D-SSIM: (1 - SSIM) / 2
}

torch::Tensor LossFunctions::computeCombinedLoss(const torch::Tensor& rendered, 
                                               const torch::Tensor& ground_truth, 
                                               float lambda) {
    auto l1_loss = computeL1Loss(rendered, ground_truth);
    auto d_ssim_loss = computeD_SSIMLoss(rendered, ground_truth);
    
    return (1.0f - lambda) * l1_loss + lambda * d_ssim_loss;
}

torch::Tensor LossFunctions::computeSSIM(const torch::Tensor& img1, const torch::Tensor& img2) {
    // Simple SSIM implementation
    // Expected input: [B, C, H, W] or [C, H, W]
    
    auto mu1 = torch::mean(img1);
    auto mu2 = torch::mean(img2);
    
    auto mu1_sq = mu1 * mu1;
    auto mu2_sq = mu2 * mu2;
    auto mu1_mu2 = mu1 * mu2;
    
    auto sigma1_sq = torch::mean((img1 - mu1) * (img1 - mu1));
    auto sigma2_sq = torch::mean((img2 - mu2) * (img2 - mu2));
    auto sigma12 = torch::mean((img1 - mu1) * (img2 - mu2));
    
    float c1 = 0.01f * 0.01f;
    float c2 = 0.03f * 0.03f;
    
    auto numerator = (2 * mu1_mu2 + c1) * (2 * sigma12 + c2);
    auto denominator = (mu1_sq + mu2_sq + c1) * (sigma1_sq + sigma2_sq + c2);
    
    return numerator / denominator;
}

torch::Tensor LossFunctions::createGaussianWindow(int window_size, int channels, torch::Device device) {
    // Create 1D Gaussian kernel
    auto coords = torch::arange(window_size, torch::kFloat32).to(device) - window_size / 2.0f;
    auto g = torch::exp(-(coords * coords) / (2.0f * 1.5f * 1.5f));
    g = g / g.sum();
    
    // Create 2D kernel
    auto kernel_2d = g.unsqueeze(0) * g.unsqueeze(1);
    
    // Expand for channels
    return kernel_2d.unsqueeze(0).unsqueeze(0).expand({channels, 1, window_size, window_size});
}

} // namespace optimization
} // namespace gaussian_splatting