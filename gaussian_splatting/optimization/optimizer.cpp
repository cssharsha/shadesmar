#include "gaussian_splatting/optimization/optimizer.hpp"
#include <iomanip>
#include <logging/logging.hpp>
#include <vector>
#include "gaussian_splatting/common/tensor_config.hpp"
#include "gaussian_splatting/training/gaussian_tensors.hpp"

namespace gaussian_splatting {
namespace optimization {

void Optimizer::initialize(GaussianTensors& gaussians, int iterations) {
    // Rather than having one optimizer for each of the parameters as in gsplat examples,
    // create an optimization param group that is passed to one single
    // handle of the optimizer
    // (https://discuss.pytorch.org/t/per-parameter-options-in-libtorch-c/98817)
    std::vector<torch::optim::OptimizerParamGroup> param_groups;

    // Get references to the actual tensors
    auto& positions = gaussians.get_positions();
    auto& scales = gaussians.get_scales();
    auto& rotations = gaussians.get_rotations();
    auto& opacities = gaussians.get_opacities();
    auto& sh_0 = gaussians.get_sh_0();
    auto& sh_N = gaussians.get_sh_N();

    LOG(INFO) << "=== SREE DEBUG: Optimizer Init ===";
    LOG(INFO) << "Positions tensor address: " << positions.data_ptr();
    LOG(INFO) << "Opacities tensor address: " << opacities.data_ptr();
    LOG(INFO) << "Scales tensor address: " << scales.data_ptr();

    // Scale positions learning rate by scene_scale to adapt to scene size
    // This matches gsplat reference: means_lr * scene_scale (simple_trainer.py:259)
    float scene_scale = gaussians.get_scene_scale();
    float adjusted_positions_lr = config.positions_lr * scene_scale;
    LOG(INFO) << "Scene scale: " << scene_scale;
    LOG(INFO) << "Adjusted positions LR: " << adjusted_positions_lr
              << " (base=" << config.positions_lr << " * scene_scale=" << scene_scale << ")";

    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {positions}, std::make_unique<torch::optim::AdamOptions>(adjusted_positions_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {scales}, std::make_unique<torch::optim::AdamOptions>(config.scales_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {rotations}, std::make_unique<torch::optim::AdamOptions>(config.rotations_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {opacities}, std::make_unique<torch::optim::AdamOptions>(config.opacities_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {sh_0}, std::make_unique<torch::optim::AdamOptions>(config.sh_coefficients_lr)));
    param_groups.emplace_back(torch::optim::OptimizerParamGroup(
        {sh_N}, std::make_unique<torch::optim::AdamOptions>(config.sh_coefficients_lr / 20.f)));

    for (auto& g : param_groups)
        static_cast<torch::optim::AdamOptions&>(g.options()).eps(1e-15);

    // Makebe use something else. We don't need to use Adam here.
    optimizer_ = std::make_unique<torch::optim::Adam>(param_groups,
                                                      torch::optim::AdamOptions(0.f).eps(1e-15));

    LOG(INFO) << "After optimizer creation:";
    LOG(INFO) << "Optimizer param[0] address: "
              << optimizer_->param_groups()[0].params()[0].data_ptr();
    LOG(INFO) << "Optimizer param[3] address: "
              << optimizer_->param_groups()[3].params()[0].data_ptr();

    for (size_t i = 0; i < optimizer_->param_groups().size(); ++i) {
        auto& group = optimizer_->param_groups()[i];
        auto& opts = static_cast<torch::optim::AdamOptions&>(group.options());
        LOG(INFO) << "Why Mr Anderson why: Param group " << i << " LR: " << opts.lr();
    }
    LOG(INFO) << "=======================================";

    // Initialize scheduler
    const double gamma = std::pow(0.01, 1.0 / iterations);
    scheduler_ = std::make_unique<Scheduler>(optimizer_.get(), gamma, 0);
}

void Optimizer::step() {
    // Debug: Log optimizer state before step
    LOG(INFO) << "=== Calling Optimizer at all? ===";
    LOG(INFO) << "Number of param groups: " << optimizer_->param_groups().size();

    // Log tensor addresses that the optimizer sees
    LOG(INFO) << "=== Tensor Addresses IN OPTIMIZER ===";
    if (optimizer_->param_groups().size() >= 4) {
        LOG(INFO) << "Optimizer param[0] (positions) address: "
                  << optimizer_->param_groups()[0].params()[0].data_ptr()
                  << (optimizer_->param_groups()[0].params()[0].grad().defined()
                          ? ", grad address: " +
                                std::to_string(reinterpret_cast<uintptr_t>(
                                    optimizer_->param_groups()[0].params()[0].grad().data_ptr()))
                          : ", grad: UNDEFINED");

        LOG(INFO) << "Optimizer param[1] (scales) address: "
                  << optimizer_->param_groups()[1].params()[0].data_ptr()
                  << (optimizer_->param_groups()[1].params()[0].grad().defined()
                          ? ", grad address: " +
                                std::to_string(reinterpret_cast<uintptr_t>(
                                    optimizer_->param_groups()[1].params()[0].grad().data_ptr()))
                          : ", grad: UNDEFINED");

        LOG(INFO) << "Optimizer param[3] (opacities) address: "
                  << optimizer_->param_groups()[3].params()[0].data_ptr()
                  << (optimizer_->param_groups()[3].params()[0].grad().defined()
                          ? ", grad address: " +
                                std::to_string(reinterpret_cast<uintptr_t>(
                                    optimizer_->param_groups()[3].params()[0].grad().data_ptr()))
                          : ", grad: UNDEFINED");
    }
    LOG(INFO) << "======================================";

    for (size_t i = 0; i < optimizer_->param_groups().size(); ++i) {
        const auto& group = optimizer_->param_groups()[i];
        const auto& params = group.params();

        if (!params.empty()) {
            const auto& param = params[0];
            const auto& opts = static_cast<const torch::optim::AdamOptions&>(group.options());
            double lr = opts.lr();

            std::ostringstream oss;
            oss << "Param group " << i << ": size=" << param.sizes()
                << ", requires_grad=" << param.requires_grad()
                << ", has_grad=" << (param.grad().defined() ? "yes" : "no") << ", lr=" << lr;

            if (param.grad().defined()) {
                // Use double precision to check if float rounding is the issue
                double grad_norm_double = param.grad().norm().item<double>();
                float grad_norm_float = common::itemAs(param.grad().norm());
                double grad_max = param.grad().abs().max().item<double>();
                double grad_min_nonzero =
                    param.grad().abs().masked_select(param.grad().abs() > 0).numel() > 0
                        ? param.grad()
                              .abs()
                              .masked_select(param.grad().abs() > 0)
                              .min()
                              .item<double>()
                        : 0.0;

                oss << ", grad_norm(double)=" << std::scientific << std::setprecision(15)
                    << grad_norm_double << ", grad_norm(float)=" << grad_norm_float
                    << ", grad_max=" << grad_max << ", grad_min_nonzero=" << grad_min_nonzero;
            }
            LOG(INFO) << oss.str();
        }
    }

    // Capture parameter values before step for all param groups
    std::vector<torch::Tensor> params_before;
    for (size_t i = 0; i < optimizer_->param_groups().size(); ++i) {
        params_before.push_back(optimizer_->param_groups()[i].params()[0].clone().detach());
    }

    LOG(INFO) << "=== Pre-Step Analysis ===";
    auto& positions_param = optimizer_->param_groups()[0].params()[0];
    LOG(INFO) << "Positions before has grad: " << positions_param.grad().defined();
    LOG(INFO) << "Positions grad norm before step: "
              << common::itemAs(positions_param.grad().norm());
    LOG(INFO) << "Positions requires_grad: " << positions_param.requires_grad();
    LOG(INFO) << "Positions is_leaf: " << positions_param.is_leaf();

    // Show expected update magnitude for each param group
    LOG(INFO) << "=== Expected Update Magnitude (lr * grad_norm) ===";
    std::vector<std::string> param_names = {"positions", "scales", "rotations",
                                            "opacities", "sh_0",   "sh_N"};
    for (size_t i = 0; i < optimizer_->param_groups().size(); ++i) {
        const auto& group = optimizer_->param_groups()[i];
        const auto& param = group.params()[0];
        const auto& opts = static_cast<const torch::optim::AdamOptions&>(group.options());
        double lr = opts.lr();

        if (param.grad().defined()) {
            double grad_norm = param.grad().norm().item<double>();
            double expected_update = lr * grad_norm;
            LOG(INFO) << param_names[i] << ": lr=" << lr << " * grad_norm=" << grad_norm
                      << " = expected_update=" << expected_update;
        }
    }
    LOG(INFO) << "=============================================";

    LOG(INFO) << "About to call optimizer_->step()...";

    // Check optimizer state
    std::string param_key = c10::guts::to_string(positions_param.unsafeGetTensorImpl());
    auto state_it = optimizer_->state().find(param_key);
    if (state_it != optimizer_->state().end()) {
        LOG(INFO) << "Adam state exists for positions";
    } else {
        LOG(INFO)
            << "WARNING: No Adam state for positions! Optimizer will create it on first step.";
    }
    LOG(INFO) << "Optimizer state:" << optimizer_->state().size();

    for (const auto& state : optimizer_->state()) {
        LOG(INFO) << "State: " << state.first;
    }

    optimizer_->step();
    LOG(INFO) << "Optimizer step completed";

    // Check if all parameters actually changed
    LOG(INFO) << "=== Parameter Changes ===";
    LOG(INFO) << "Format: name - max_change, mean_change, [lr, expected=lr*grad_norm]";
    for (size_t i = 0; i < optimizer_->param_groups().size() && i < params_before.size(); ++i) {
        const auto& group = optimizer_->param_groups()[i];
        const auto& param_after = group.params()[0];
        const auto& param_before = params_before[i];
        const auto& opts = static_cast<const torch::optim::AdamOptions&>(group.options());

        double max_diff = common::itemAs((param_after - param_before).abs().max());
        double mean_diff = common::itemAs((param_after - param_before).abs().mean());

        // Calculate expected update for comparison (gradients are already zeroed at this point)
        double lr = opts.lr();

        LOG(INFO) << param_names[i] << " - max: " << std::scientific << std::setprecision(10)
                  << max_diff << ", mean: " << mean_diff << " [lr=" << lr << "]";
    }
    LOG(INFO) << "=========================";

    optimizer_->zero_grad(true);
    LOG(INFO) << "Gradients zeroed";
    scheduler_->step();
}

}  // namespace optimization
}  // namespace gaussian_splatting
