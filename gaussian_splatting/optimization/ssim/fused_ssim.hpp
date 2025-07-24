#pragma once

#include <torch/torch.h>
#include <tuple>
#include "gaussian_splatting/optimization/ssim/ssim.h"

namespace gaussian_splatting {
namespace optimization {
// Knicked these constants from the original implementation
constexpr double C1 = 0.01 * 0.01;
constexpr double C2 = 0.03 * 0.03;

class FusedSSIM : public torch::autograd::Function<FusedSSIM> {
public:
    static torch::Tensor forward(torch::autograd::AutogradContext* ctx, torch::Tensor img1,
                                 torch::Tensor img2, bool train);
    static std::vector<torch::Tensor> backward(torch::autograd::AutogradContext* ctx,
                                               std::vector<torch::Tensor> grad_out);
};

inline torch::Tensor fused_ssim(torch::Tensor img1, torch::Tensor img2, bool train,
                                torch::Tensor& result) {
    img1 = img1.contiguous();
    result = FusedSSIM::apply(img1, img2, train);
    return result.mean();
}

}  // namespace optimization
}  // namespace gaussian_splatting
