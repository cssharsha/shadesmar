#pragma once

#include <torch/torch.h>
#include <memory>
#include "gaussian_splatting/optimization/config.hpp"
#include "gaussian_splatting/optimization/optimizer.hpp"
#include "gaussian_splatting/optimization/strategy/default.hpp"
#include "gaussian_splatting/rendering/rasterizer.hpp"
#include "gaussian_splatting/training/gaussian_tensors.hpp"

namespace gaussian_splatting {
namespace optimization {

template <typename UpdatePolicy>
class Strategy {
public:
    Strategy(GaussianTensors* gaussians, std::unique_ptr<Optimizer> optimizer)
        : gaussians_(gaussians), update_policy_(std::move(optimizer)) {}

    Strategy(GaussianTensors* gaussians, UpdatePolicy update_policy)
        : gaussians_(gaussians), update_policy_(std::move(update_policy)) {}

    void postBackward(rendering::RasterizationOutput& r_output, int iter);
    void step(int iter);

    GaussianTensors* getGaussians() {
        return gaussians_;
    }

private:
    GaussianTensors* gaussians_;
    UpdatePolicy update_policy_;

    strategy::Config config;
};

using DefaultStrategy = Strategy<strategy::Default>;

}  // namespace optimization
}  // namespace gaussian_splatting
