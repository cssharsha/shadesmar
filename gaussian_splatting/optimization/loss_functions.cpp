#include "loss_functions.hpp"
#include <iterator>
#include <logging/logging.hpp>
#include "gaussian_splatting/optimization/ssim/fused_ssim.hpp"
#include "gaussian_splatting/utils/image_utils.hpp"

namespace gaussian_splatting {
namespace optimization {

LossFunctions::Config LossFunctions::config;

torch::Tensor LossFunctions::computePhotometricLoss(const torch::Tensor& rendered_image,
                                                    const torch::Tensor& ground_truth_image,
                                                    int keyframe_id, int iteration) {
    std::cout << "Rendered image: " << rendered_image.sizes() << std::endl;
    std::cout << "Ground truth image: " << ground_truth_image.sizes() << std::endl;
    // std::cout << "Mean xyz of ground truth image: " << ground_truth_image.mean(1)[0] <<
    // std::endl; std::cout << "Mean xyz of rendered image: " << rendered_image.mean(1)[0] <<
    // std::endl;

    std::string append = (iteration >= 0) ? std::to_string(iteration) + "_" : "";
    append += (keyframe_id >= 0) ? std::to_string(keyframe_id) + "_" : "";
    {
        torch::NoGradGuard no_grad;
        utils::writeImageToDirectory(utils::tensorToMat(rendered_image.to(torch::kCPU), false),
                                     "/data/south-building/debug/",
                                     append + "loss_rendered_image.png");
        utils::writeImageToDirectory(utils::tensorToMat(ground_truth_image.to(torch::kCPU), false),
                                     "/data/south-building/debug/",
                                     append + "loss_ground_truth_image.png");
    }

    auto l1_loss = torch::l1_loss(rendered_image, ground_truth_image);
    torch::Tensor ssim_loss_tensor;
    auto ssim_loss = fused_ssim(rendered_image, ground_truth_image, true, ssim_loss_tensor);

    {
        torch::NoGradGuard no_grad;
        std::cout << "L1 loss shape: " << l1_loss.sizes() << ", value: " << l1_loss << std::endl;
        std::cout << "SSIM loss shape: " << ssim_loss_tensor.sizes() << ", value: " << ssim_loss
                  << std::endl;

        utils::writeImageToDirectory(utils::tensorToMat(ssim_loss_tensor.to(torch::kCPU)[0], false),
                                     "/data/south-building/debug/", append + "ssim_loss.png");
    }

    auto loss = (1.f - config.lambda_dssim) * l1_loss + config.lambda_dssim * ssim_loss;
    return loss;
}

torch::Tensor LossFunctions::computeScaleRegularizationLoss(const torch::Tensor& scale) {
    if (config.scale_regularization_weight > 0.f) {
        auto l1_scale = scale.mean();
        return config.scale_regularization_weight * l1_scale;
    }

    return torch::zeros({1}, torch::kFloat32).requires_grad_();
}

torch::Tensor LossFunctions::computeOpacityRegularizationLoss(const torch::Tensor& opacities) {
    if (config.opacity_regularization_weight > 0.f) {
        auto l1_opacity = opacities.mean();
        return config.opacity_regularization_weight * l1_opacity;
    }

    return torch::zeros({1}, torch::kFloat32).requires_grad_();
}

torch::Tensor LossFunctions::computeL1Loss(const torch::Tensor& rendered,
                                           const torch::Tensor& ground_truth) {
    return torch::l1_loss(rendered, ground_truth);
}

torch::Tensor LossFunctions::computeD_SSIMLoss(const torch::Tensor& rendered,
                                               const torch::Tensor& ground_truth) {
    auto ssim_value = computeSSIM(rendered, ground_truth);
    return (1.0f - ssim_value) / 2.0f;  // D-SSIM: (1 - SSIM) / 2
}

torch::Tensor LossFunctions::computeCombinedLoss(const torch::Tensor& rendered,
                                                 const torch::Tensor& ground_truth, float lambda) {
    auto l1_loss = computeL1Loss(rendered, ground_truth);
    std::cout << "l1_loss: " << l1_loss.item<float>() << std::endl;
    auto d_ssim_loss = computeD_SSIMLoss(rendered, ground_truth);
    std::cout << "D_SSIM loss: " << d_ssim_loss.item<float>() << std::endl;

    return (1.0f - lambda) * l1_loss + lambda * d_ssim_loss;
}

torch::Tensor LossFunctions::computeSSIM(const torch::Tensor& img1, const torch::Tensor& img2) {
    // Simple SSIM implementation
    // Expected input: [B, C, H, W] or [C, H, W]
    std::cout << "Image 1: " << img1.sizes() << ", Image2: " << img2.sizes() << std::endl;

    auto mu1 = torch::mean(img1);
    auto mu2 = torch::mean(img2);
    std::cout << "Image 1 mu: " << mu1.item<float>() << " image2 mu: " << mu2.item<float>()
              << std::endl;

    auto mu1_sq = mu1 * mu1;
    auto mu2_sq = mu2 * mu2;
    auto mu1_mu2 = mu1 * mu2;

    auto sigma1_sq = torch::mean((img1 - mu1) * (img1 - mu1));
    auto sigma2_sq = torch::mean((img2 - mu2) * (img2 - mu2));
    auto sigma12 = torch::mean((img1 - mu1) * (img2 - mu2));
    std::cout << "Image 1 sigma: " << sigma1_sq.item<float>()
              << " image2 sigma: " << sigma2_sq.item<float>()
              << "sigma12: " << sigma12.item<float>() << std::endl;

    float c1 = 0.01f * 0.01f;
    float c2 = 0.03f * 0.03f;

    auto numerator = (2 * mu1_mu2 + c1) * (2 * sigma12 + c2);
    auto denominator = (mu1_sq + mu2_sq + c1) * (sigma1_sq + sigma2_sq + c2);

    std::cout << "Total loss: " << numerator.item<float>() << "/" << denominator.item<float>()
              << std::endl;

    return numerator / denominator;
}

torch::Tensor LossFunctions::createGaussianWindow(int window_size, int channels,
                                                  torch::Device device) {
    // Create 1D Gaussian kernel
    auto coords = torch::arange(window_size, torch::kFloat32).to(device) - window_size / 2.0f;
    auto g = torch::exp(-(coords * coords) / (2.0f * 1.5f * 1.5f));
    g = g / g.sum();

    // Create 2D kernel
    auto kernel_2d = g.unsqueeze(0) * g.unsqueeze(1);

    // Expand for channels
    return kernel_2d.unsqueeze(0).unsqueeze(0).expand({channels, 1, window_size, window_size});
}

}  // namespace optimization
}  // namespace gaussian_splatting
