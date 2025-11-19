#include <logging/logging.hpp>

#include "gaussian_splatting/common/tensor_config.hpp"
#include "gaussian_splatting/optimization/optimizer.hpp"
#include "gaussian_splatting/optimization/strategy/default.hpp"
#include "gaussian_splatting/optimization/strategy/strategy.hpp"
#include "gaussian_splatting/utils/torch_utils.hpp"

namespace gaussian_splatting {
namespace optimization {

template <typename UpdatePolicy>
void Strategy<UpdatePolicy>::postBackward(rendering::RasterizationOutput& r_output, int iter) {
    update_policy_(gaussians_, r_output, iter);
}

template <typename UpdatePolicy>
void Strategy<UpdatePolicy>::step(int iter) {
    update_policy_.step(iter);
}

template class Strategy<strategy::Default>;

}  // namespace optimization
}  // namespace gaussian_splatting
