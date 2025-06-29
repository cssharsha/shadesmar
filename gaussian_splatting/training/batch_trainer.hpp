#pragma once

#include <torch/torch.h>
#include <memory>
#include "training_config.hpp"
#include "../utils/batch_gpu_manager.hpp"
#include "../optimization/parameter_transforms.hpp"
#include "../optimization/loss_functions.hpp"
#include "core/types/gaussian_splat.hpp"
#include "core/storage/map_store.hpp"

namespace gaussian_splatting {
namespace training {

struct TrainingResults {
    float total_loss = 0.0f;
    float l1_loss = 0.0f;
    float d_ssim_loss = 0.0f;
    int iterations_completed = 0;
    bool success = false;
};

class BatchTrainer {
public:
    explicit BatchTrainer(const TrainingConfig& config, 
                         std::shared_ptr<core::storage::MapStore> map_store);
    ~BatchTrainer();
    
    // Train single batch completely
    bool trainBatch(uint32_t batch_id, TrainingResults& results);
    
    // Load batch and setup for training
    bool setupBatchForTraining(const core::types::GaussianSplatBatch& batch);
    
    // Training step
    bool performTrainingStep(int iteration);
    
    // Multi-view consistency setup
    void setupMultiViewRendering(const std::vector<uint64_t>& keyframe_ids);
    
    // Cleanup
    void clearCurrentBatch();
    
    // Status
    bool isTraining() const { return is_training_; }
    uint32_t getCurrentBatchId() const { return current_batch_id_; }
    
private:
    TrainingConfig config_;
    std::shared_ptr<core::storage::MapStore> map_store_;
    
    // Components
    std::unique_ptr<utils::BatchGPUManager> gpu_manager_;
    std::unique_ptr<optimization::ParameterTransforms> param_transforms_;
    std::unique_ptr<optimization::LossFunctions> loss_functions_;
    
    // Current training state
    utils::GPUBatchData current_gpu_data_;
    std::vector<uint64_t> current_keyframe_ids_;
    uint32_t current_batch_id_ = 0;
    bool is_training_ = false;
    
    // Optimization state
    std::unique_ptr<torch::optim::Adam> optimizer_;
    std::vector<torch::Tensor> optimizable_params_;
    
    // Setup optimization parameters
    void setupOptimizer();
    
    // Render step
    torch::Tensor renderBatch(const std::vector<torch::Tensor>& camera_params);
    
    // Extract ground truth images for comparison
    std::vector<torch::Tensor> extractGroundTruthImages(const std::vector<uint64_t>& keyframe_ids);
    
    // Apply parameter transforms
    void applyParameterTransforms();
    
    // Initialize optimizable parameters
    void initializeOptimizableParameters();
};

} // namespace training
} // namespace gaussian_splatting