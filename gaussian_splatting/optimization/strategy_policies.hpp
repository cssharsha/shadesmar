#pragma once

#include <torch/torch.h>

namespace gaussian_splatting {
namespace optimization {

// Default grow policy implementation
// Uses the original densification algorithm: duplicate small splats with high gradients,
// split large splats with high gradients
class DefaultGrowPolicy {
public:
    template<typename StrategyType>
    void operator()(StrategyType* strategy, int iter);
};

// Default prune policy implementation
// Removes splats with low opacity
class DefaultPrunePolicy {
public:
    template<typename StrategyType>
    void operator()(StrategyType* strategy, int iter);
};

}  // namespace optimization
}  // namespace gaussian_splatting
