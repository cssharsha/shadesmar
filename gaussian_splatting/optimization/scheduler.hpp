#pragma once

#include <torch/torch.h>
#include <memory>

namespace gaussian_splatting {
namespace optimization {

class Scheduler {
public:
    Scheduler(torch::optim::Optimizer* optimizer, double gamma, int param_group_index = -1)
        : optimizer_(optimizer), gamma_(gamma), param_group_index_(param_group_index) {}

    void step();

private:
    // torch::optim::Adam is supposed to be inherited from torch::optim::Optimizer
    torch::optim::Optimizer* optimizer_;
    double gamma_;
    int param_group_index_;
};

}  // namespace optimization
}  // namespace gaussian_splatting
