#include <torch/torch.h>
#include <tuple>

#include "gaussian_splatting/optimization/ssim/fused_ssim.hpp"

namespace gaussian_splatting {
namespace optimization {

torch::Tensor FusedSSIM::forward(torch::autograd::AutogradContext* ctx, torch::Tensor img1,
                                 torch::Tensor img2, bool train) {
    // Unsqueeze to add a batch dim. The fusedsssim expects 4D tensors [N,C,H,W]
    assert(img1.dim() == 4 && img2.dim() == 4);

    auto out = fusedssim(C1, C2, img1, img2, train);
    auto ssim_map = std::get<0>(out);
    auto dm_dm1 = std::get<1>(out);
    auto dm_ds1sq = std::get<2>(out);
    auto dm_ds12 = std::get<3>(out);

    // add padding
    int64_t h = ssim_map.size(2);
    int64_t w = ssim_map.size(3);
    if (h > 10 && w > 10) {
        ssim_map =
            ssim_map.index({torch::indexing::Slice(), torch::indexing::Slice(),
                            torch::indexing::Slice(5, h - 5), torch::indexing::Slice(5, w - 5)});
    }

    ctx->save_for_backward({img1.detach(), img2, dm_dm1, dm_ds1sq, dm_ds12});
    return ssim_map;
}

std::vector<torch::Tensor> FusedSSIM::backward(torch::autograd::AutogradContext* ctx,
                                               std::vector<torch::Tensor> grad_out) {
    std::cout << "Fused ssim backward" << std::endl;
    auto vars = ctx->get_saved_variables();
    auto img1 = vars[0];
    auto img2 = vars[1];
    auto dm1 = vars[2];
    auto ds1sq = vars[3];
    auto ds12 = vars[4];

    std::cout << "Reread all the saved variabled: " << vars.size() << "img1: " << img1.sizes()
              << "img2: " << img2.sizes() << "dm1: " << dm1.sizes() << "ds1sq: " << ds1sq.sizes()
              << "ds12: " << ds12.sizes() << "grad_out: " << grad_out.size() << std::endl;

    auto dL_dmap = grad_out[0];
    std::cout << "dL_dmap: " << dL_dmap.sizes() << std::endl;
    using torch::indexing::Slice;
    auto full = torch::zeros_like(img1);
    // Convert negative indices to positive ones
    int64_t h = full.size(2);  // height
    int64_t w = full.size(3);  // width
    if (h > 10 && w > 10) {    // Ensure we have enough pixels to crop
        full.index_put_({torch::indexing::Slice(), torch::indexing::Slice(),
                         torch::indexing::Slice(5, h - 5), torch::indexing::Slice(5, w - 5)},
                        dL_dmap);
    }
    dL_dmap = full;

    auto grad_img1 = fusedssim_backward(C1, C2, img1, img2, dL_dmap, dm1, ds1sq, ds12);

    return {grad_img1, torch::Tensor(), torch::Tensor(), torch::Tensor()};
}

}  // namespace optimization
}  // namespace gaussian_splatting
