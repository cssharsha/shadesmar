#pragma once

#include <torch/torch.h>
#include "gaussian_splatting/training/gaussian_tensors.hpp"

namespace gaussian_splatting {
namespace optimization {

class Optimizer {
public:
    static struct Config {
        int max_iterations = 100;
        float learning_rate = 0.01f;
        float positions_lr = 0.01f;
        float rotations_lr = 0.01f;
        float scales_lr = 0.01f;
        float opacities_lr = 0.01f;
        float sh_coefficients_lr = 0.01f;
    } config;

    Optimizer() = default;
    void initialize(GaussianTensors& gaussians);
    torch::optim::Optimizer* getOptimizer() {
        return optimizer_.get();
    }

    void step();

    template <typename ParamUpdateFunction, typename StateUpdateFunction>
    void updateParamAndState(ParamUpdateFunction param_fn, StateUpdateFunction state_fn,
                             GaussianTensors* gaussians) {
        torch::NoGradGuard no_grad;
        const size_t param_size = optimizer_->param_groups().size();

        std::vector<torch::Tensor*> params;
        params.push_back(&gaussians->get_positions());
        params.push_back(&gaussians->get_scales());
        params.push_back(&gaussians->get_rotations());
        params.push_back(&gaussians->get_opacities());
        params.push_back(&gaussians->get_sh_0());
        params.push_back(&gaussians->get_sh_N());

        if (params.size() != param_size) {
            throw std::runtime_error("Expected " + std::to_string(param_size) +
                                     " parameters, got " + std::to_string(params.size()));
        }

        std::vector<torch::Tensor> new_params;
        new_params.resize(param_size);

        // Collect old parameter keys and states
        std::vector<std::string> old_param_keys;
        std::vector<std::unique_ptr<torch::optim::OptimizerParamState>> saved_states;
        saved_states.resize(param_size);
        for (size_t i = 0; i < param_size; ++i) {
            auto param = params[i];
            auto new_param = param_fn(i, *param);
            std::cout << "New param: " << new_param.sizes() << std::endl;
            new_params[i] = new_param;
            std::cout << "New params: " << new_params[i].sizes() << std::endl;

            auto& old_param = optimizer_->param_groups()[i].params()[0];
            std::cout << "Done old param fetch" << std::endl;
            std::cout << "Old param: " << old_param.sizes() << std::endl;
            std::string old_param_key = c10::guts::to_string(old_param.unsafeGetTensorImpl());
            old_param_keys.push_back(old_param_key);

            // Check if state exists
            auto state_it = optimizer_->state().find(old_param_key);
            if (state_it != optimizer_->state().end()) {
                // Clone the state before modifying
                auto* adam_state =
                    dynamic_cast<torch::optim::AdamParamState*>(state_it->second.get());
                auto new_state = state_fn(*adam_state, new_param);
                saved_states[i] = std::move(new_state);
            } else {
                saved_states[i] = nullptr;
            }
        }

        for (const auto& key : old_param_keys) {
            optimizer_->state().erase(key);
        }

        for (size_t i = 0; i < param_size; ++i) {
            optimizer_->param_groups()[i].params()[0] = new_params[i];

            if (saved_states[i]) {
                std::string new_param_key =
                    c10::guts::to_string(new_params[i].unsafeGetTensorImpl());
                optimizer_->state()[new_param_key] = std::move(saved_states[i]);
            }
        }

        *params[0] = new_params[0];  // positions
        *params[1] = new_params[1];  // rotations
        *params[2] = new_params[2];  // scales
        *params[3] = new_params[3];  // opacities
        *params[4] = new_params[4];  // sh_0
        *params[5] = new_params[5];  // sh_N

        std::cout << "Updated params: " << params[0]->sizes() << params[1]->sizes()
                  << params[2]->sizes() << params[3]->sizes() << params[4]->sizes() << std::endl;
        std::cout << "Gaussian tensors: " << gaussians->get_positions().sizes() << " "
                  << gaussians->get_rotations().sizes() << " " << gaussians->get_scales().sizes()
                  << " " << gaussians->get_opacities().sizes() << " "
                  << gaussians->get_sh_0().sizes() << " " << gaussians->get_sh_N().sizes()
                  << std::endl;
    }

private:
    // There is no SelectiveAdam directly available in libtorch :(
    std::unique_ptr<torch::optim::Adam> optimizer_;
    // std::map<std::string, std::unique_ptr<torch::optim::Adam>> optimizers_;
};

}  // namespace optimization
}  // namespace gaussian_splatting
