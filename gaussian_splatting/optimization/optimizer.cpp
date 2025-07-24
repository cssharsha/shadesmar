#include "gaussian_splatting/optimization/optimizer.hpp"
#include <vector>
#include "gaussian_splatting/training/gaussian_tensors.hpp"

namespace gaussian_splatting {
namespace optimization {

Optimizer::Config Optimizer::config;

void Optimizer::initialize(GaussianTensors& gaussians) {
    // Rather than having one optimizer for each of the parameters as in gsplat examples,
    // create an optimization param group that is passed to one single
    // handle of the optimizer
    // (https://discuss.pytorch.org/t/per-parameter-options-in-libtorch-c/98817)
    std::vector<torch::optim::OptimizerParamGroup> param_groups;

    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {gaussians.get_positions()},
        std::make_unique<torch::optim::AdamOptions>(config.positions_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {gaussians.get_scales()}, std::make_unique<torch::optim::AdamOptions>(config.scales_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {gaussians.get_rotations()},
        std::make_unique<torch::optim::AdamOptions>(config.rotations_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {gaussians.get_opacities()},
        std::make_unique<torch::optim::AdamOptions>(config.opacities_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {gaussians.get_sh_0()},
        std::make_unique<torch::optim::AdamOptions>(config.sh_coefficients_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {gaussians.get_sh_N()},
        std::make_unique<torch::optim::AdamOptions>(config.sh_coefficients_lr / 20.f)));

    for (auto& g : param_groups)
        static_cast<torch::optim::AdamOptions&>(g.options()).eps(1e-15);
    optimizer_ = std::make_unique<torch::optim::Adam>(param_groups,
                                                      torch::optim::AdamOptions(0.).eps(1e-15));
}

void Optimizer::step() {
    optimizer_->step();
    optimizer_->zero_grad(true);
}

}  // namespace optimization
}  // namespace gaussian_splatting
