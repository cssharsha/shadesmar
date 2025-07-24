#pragma once

#include <torch/torch.h>

namespace gaussian_splatting {
namespace optimization {

class LossFunctions {
public:
    static struct Config {
        int window_size = 11;
        float lambda_dssim = 0.2f;
        float scale_regularization_weight = 0.1f;
        float opacity_regularization_weight = 0.1f;
    } config;

    static torch::Tensor computePhotometricLoss(const torch::Tensor& rendered_image,
                                                const torch::Tensor& ground_truth_image,
                                                int keyframe_id = -1, int iteration = -1);
    static torch::Tensor computeScaleRegularizationLoss(const torch::Tensor& scale);
    static torch::Tensor computeOpacityRegularizationLoss(const torch::Tensor& opacities);
    // L1 loss between rendered and ground truth images
    static torch::Tensor computeL1Loss(const torch::Tensor& rendered,
                                       const torch::Tensor& ground_truth);

    // D-SSIM loss computation
    static torch::Tensor computeD_SSIMLoss(const torch::Tensor& rendered,
                                           const torch::Tensor& ground_truth);

    // Combined loss: (1-lambda) * L1 + lambda * D-SSIM
    static torch::Tensor computeCombinedLoss(const torch::Tensor& rendered,
                                             const torch::Tensor& ground_truth,
                                             float lambda = 0.2f);

private:
    // SSIM computation helper
    static torch::Tensor computeSSIM(const torch::Tensor& img1, const torch::Tensor& img2);

    // Gaussian window for SSIM
    static torch::Tensor createGaussianWindow(int window_size, int channels, torch::Device device);
};

}  // namespace optimization
}  // namespace gaussian_splatting
