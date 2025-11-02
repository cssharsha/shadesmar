#pragma once

#include <torch/torch.h>
#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <queue>
#include <stf/transform_tree.hpp>
#include <thread>
#include "../optimization/loss_functions.hpp"
#include "../optimization/parameter_transforms.hpp"
#include "../utils/batch_gpu_manager.hpp"
#include "core/storage/map_store.hpp"
#include "core/types/gaussian_splat.hpp"
#include "gaussian_splatting/optimization/strategy.hpp"
#include "gaussian_splatting/rendering/rasterizer.hpp"
#include "gaussian_splatting/training/gaussian_tensors.hpp"
#include "gaussian_splatting/training/keyframe_batch.hpp"
#include "gaussian_splatting/training/keyframe_tensor.hpp"
#include "gaussian_splatting/training/training_config.hpp"
#include "gaussian_splatting/visualization/training_visualizer.hpp"

namespace gaussian_splatting {
namespace training {

struct TrainingResults {
    float total_loss = 0.0f;
    float l1_loss = 0.0f;
    float d_ssim_loss = 0.0f;
    int iterations_completed = 0;
    bool success = false;
    torch::Tensor rendered_image;
};

class BatchTrainer {
public:
    explicit BatchTrainer(const TrainingConfig& config,
                          std::shared_ptr<core::storage::MapStore> map_store,
                          std::shared_ptr<stf::TransformTree> tf_tree);
    ~BatchTrainer();

    // Train single batch completely
    bool trainBatch(uint32_t batch_id, const training::KeyframeBatch& keyframe_batch,
                    TrainingResults& results);
    bool trainBatch(const core::types::GaussianSplatBatch& batch,
                    const training::KeyframeBatch& keyframe_batch, TrainingResults& results,
                    std::shared_ptr<visualization::RerunTrainingVisualizer> viz = nullptr);
    bool trainKeyframe(const core::storage::KeyFramePtr& keyframe, TrainingResults& result,
                       int iteration = 0,
                       std::shared_ptr<visualization::RerunTrainingVisualizer> viz = nullptr);

    // Load batch and setup for training
    bool setupBatchForTraining(const core::types::GaussianSplatBatch& batch,
                               const training::KeyframeBatch& keyframe_batch);

    void copySplatsToBatch(std::vector<core::types::GaussianSplat>& splats);

    // Training step
    bool performTrainingStep(int iteration,
                             std::shared_ptr<visualization::RerunTrainingVisualizer> viz = nullptr);

    // Cleanup
    void clearCurrentBatch();

    // Status
    bool isTraining() const {
        return is_training_;
    }
    uint32_t getCurrentBatchId() const {
        return current_batch_id_;
    }

    void enqueueKeyframeForTraining(core::storage::KeyFramePtr& keyframe) {
        std::lock_guard<std::mutex> lock(keyframe_train_queue_mutex_);
        keyframe_train_queue_.push(keyframe);
        keyframe_train_queue_cv_.notify_one();
    }
    void setBatchForTraining(const core::types::GaussianSplatBatch& batch) {
        current_gaussian_tensors_.fromSplats(batch.splats);
    }
    void setupTraining(const core::types::GaussianSplatBatch& batch);

private:
    TrainingConfig config_;
    std::shared_ptr<core::storage::MapStore> map_store_;
    std::shared_ptr<stf::TransformTree> tf_tree_;

    std::thread training_thread_;

    // Components
    std::unique_ptr<utils::BatchGPUManager> gpu_manager_;
    std::unique_ptr<optimization::ParameterTransforms> param_transforms_;
    std::unique_ptr<optimization::LossFunctions> loss_functions_;
    std::unique_ptr<rendering::DifferentiableRasterizer> rasterizer_;

    // Current training state
    GaussianTensors current_gaussian_tensors_;
    training::KeyframeBatch current_keyframe_batch_;
    training::KeyframeTensor current_keyframe_tensor_;
    uint32_t current_batch_id_ = 0;
    bool is_training_ = false;

    // Optimization state
    std::unique_ptr<torch::optim::Adam> optimizer_;
    std::vector<torch::Tensor> optimizable_params_;
    std::unique_ptr<optimization::Strategy> strategy_;

    // Setup optimization parameters
    void setupOptimizer();

    // Render step
    rendering::RasterizationOutput renderBatch();
    rendering::RasterizationOutput renderKeyframe();

    void trainingThreadLoop();
    std::queue<core::storage::KeyFramePtr> keyframe_train_queue_;
    std::map<uint32_t, int> keyframe_train_count_;
    std::mutex keyframe_train_queue_mutex_;
    std::condition_variable keyframe_train_queue_cv_;
    std::atomic<bool> shutdown_requested_ = false;

    // Apply parameter transforms
    // void applyParameterTransforms();
};

}  // namespace training
}  // namespace gaussian_splatting
