#pragma once

#include <torch/torch.h>
#include <vector>
#include "../utils/batch_gpu_manager.hpp"
#include "../training/training_config.hpp"

namespace gaussian_splatting {
namespace optimization {

struct DensificationStats {
    int gaussians_added = 0;
    int gaussians_removed = 0;
    int total_gaussians = 0;
    float avg_opacity = 0.0f;
    float avg_gradient_norm = 0.0f;
};

class DensificationController {
public:
    explicit DensificationController(const training::TrainingConfig& config);
    
    // Check if densification should be performed
    bool shouldDensify(int iteration) const;
    
    // Perform densification/pruning
    bool densifyAndPrune(utils::GPUBatchData& gpu_data, 
                        const torch::Tensor& position_gradients,
                        DensificationStats& stats);
    
    // Reset tracking for new batch
    void resetForNewBatch();
    
    // Accumulate gradients for densification decisions
    void accumulateGradients(const torch::Tensor& position_gradients);
    
private:
    training::TrainingConfig config_;
    
    // Gradient accumulation for densification
    torch::Tensor accumulated_gradients_;
    torch::Tensor gradient_counts_;
    int accumulation_steps_ = 0;
    
    // Densification operations
    std::vector<int> identifyGaussiansToRemove(const utils::GPUBatchData& gpu_data) const;
    std::vector<int> identifyGaussiansToSplit(const torch::Tensor& avg_gradients,
                                            const torch::Tensor& scales) const;
    std::vector<int> identifyGaussiansToClone(const torch::Tensor& avg_gradients,
                                            const torch::Tensor& scales) const;
    
    // Apply densification operations
    bool removeGaussians(utils::GPUBatchData& gpu_data, 
                        const std::vector<int>& indices_to_remove);
    bool splitGaussians(utils::GPUBatchData& gpu_data,
                       const std::vector<int>& indices_to_split);
    bool cloneGaussians(utils::GPUBatchData& gpu_data,
                       const std::vector<int>& indices_to_clone);
    
    // Utility functions
    torch::Tensor computeAverageGradients() const;
    void updateGaussianCounts(utils::GPUBatchData& gpu_data);
};

} // namespace optimization
} // namespace gaussian_splatting