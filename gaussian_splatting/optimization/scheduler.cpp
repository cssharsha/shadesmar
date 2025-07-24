#include "gaussian_splatting/optimization/scheduler.hpp"
#include <logging/logging.hpp>

namespace gaussian_splatting {
namespace optimization {

void Scheduler::step() {
    if (param_group_index_ >= 0) {
        auto& group = optimizer_->param_groups()[param_group_index_];

        auto* adam_options = dynamic_cast<torch::optim::AdamOptions*>(&group.options());
        double current_lr = adam_options->lr();
        adam_options->lr(current_lr * gamma_);
    } else {
        // Update all param groups
        for (auto& group : optimizer_->param_groups()) {
            auto* adam_options = dynamic_cast<torch::optim::AdamOptions*>(&group.options());
            double current_lr = adam_options->lr();
            adam_options->lr(current_lr * gamma_);
        }
    }
}
}  // namespace optimization
}  // namespace gaussian_splatting
