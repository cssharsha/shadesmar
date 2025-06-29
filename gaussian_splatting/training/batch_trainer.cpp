#include "batch_trainer.hpp"
#include <logging/logging.hpp>

namespace gaussian_splatting {
namespace training {

BatchTrainer::BatchTrainer(const TrainingConfig& config, 
                          std::shared_ptr<core::storage::MapStore> map_store)
    : config_(config), map_store_(map_store) {
    
    LOG(INFO) << "Initializing BatchTrainer";
    
    // Initialize components
    gpu_manager_ = std::make_unique<utils::BatchGPUManager>(config_);
    param_transforms_ = std::make_unique<optimization::ParameterTransforms>();
    loss_functions_ = std::make_unique<optimization::LossFunctions>();
    
    if (!gpu_manager_->isDeviceAvailable()) {
        LOG(WARNING) << "GPU device not available, using CPU";
    }
}

BatchTrainer::~BatchTrainer() {
    clearCurrentBatch();
}

bool BatchTrainer::trainBatch(uint32_t batch_id, TrainingResults& results) {
    LOG(INFO) << "Training batch " << batch_id;
    
    // Load batch from map store
    auto batch = map_store_->getGaussianSplatBatch(batch_id);
    if (!batch) {
        LOG(ERROR) << "Failed to load batch " << batch_id;
        return false;
    }
    
    // Setup batch for training
    if (!setupBatchForTraining(*batch)) {
        LOG(ERROR) << "Failed to setup batch " << batch_id << " for training";
        return false;
    }
    
    // Training loop
    is_training_ = true;
    results.success = false;
    
    for (int iter = 0; iter < config_.max_iterations_per_batch; ++iter) {
        if (!performTrainingStep(iter)) {
            LOG(ERROR) << "Training step " << iter << " failed for batch " << batch_id;
            break;
        }
        
        // Log progress periodically
        if (iter % 100 == 0) {
            LOG(INFO) << "Batch " << batch_id << " iteration " << iter 
                      << " loss: " << results.total_loss;
        }
        
        results.iterations_completed = iter + 1;
    }
    
    is_training_ = false;
    results.success = true;
    
    LOG(INFO) << "Completed training batch " << batch_id 
              << " after " << results.iterations_completed << " iterations";
    
    return true;
}

bool BatchTrainer::setupBatchForTraining(const core::types::GaussianSplatBatch& batch) {
    LOG(INFO) << "Setting up batch " << batch.batch_id << " for training";
    
    // Clear previous batch
    clearCurrentBatch();
    
    // Load batch to GPU
    if (!gpu_manager_->loadBatchToGPU(batch, current_gpu_data_)) {
        LOG(ERROR) << "Failed to load batch to GPU";
        return false;
    }
    
    current_batch_id_ = batch.batch_id;
    
    // Extract keyframe IDs for multi-view setup
    current_keyframe_ids_.clear();
    for (uint64_t kf_id = batch.start_keyframe_id; kf_id <= batch.end_keyframe_id; ++kf_id) {
        current_keyframe_ids_.push_back(kf_id);
    }
    
    // Setup multi-view rendering
    setupMultiViewRendering(current_keyframe_ids_);
    
    // Initialize optimizable parameters
    initializeOptimizableParameters();
    
    // Setup optimizer
    setupOptimizer();
    
    LOG(INFO) << "Batch setup complete: " << current_gpu_data_.num_gaussians 
              << " gaussians, " << current_keyframe_ids_.size() << " keyframes";
    
    return true;
}

bool BatchTrainer::performTrainingStep(int iteration) {
    if (!is_training_ || !current_gpu_data_.isValid()) {
        LOG(ERROR) << "Invalid training state for step " << iteration;
        return false;
    }
    
    // Apply parameter transforms
    applyParameterTransforms();
    
    // TODO: Implement actual rendering step
    // For now, create dummy rendered output
    auto rendered = torch::zeros({static_cast<int64_t>(current_keyframe_ids_.size()), 3, config_.initial_height, config_.initial_width},
                                torch::TensorOptions().device(gpu_manager_->getDevice()).dtype(torch::kFloat32));
    
    // Extract ground truth images
    auto ground_truth = extractGroundTruthImages(current_keyframe_ids_);
    
    if (ground_truth.empty()) {
        LOG(WARNING) << "No ground truth images available for iteration " << iteration;
        return true; // Continue training
    }
    
    // Compute loss
    torch::Tensor total_loss = torch::zeros({1}, torch::TensorOptions().device(gpu_manager_->getDevice()));
    
    for (size_t i = 0; i < std::min(rendered.size(0), static_cast<int64_t>(ground_truth.size())); ++i) {
        auto view_rendered = rendered[i];
        auto view_gt = ground_truth[i];
        
        auto view_loss = loss_functions_->computeCombinedLoss(view_rendered, view_gt, config_.d_ssim_lambda);
        total_loss += view_loss;
    }
    
    // Backward pass
    if (optimizer_) {
        optimizer_->zero_grad();
        total_loss.backward();
        optimizer_->step();
    }
    
    return true;
}

void BatchTrainer::setupMultiViewRendering(const std::vector<uint64_t>& keyframe_ids) {
    LOG(INFO) << "Setting up multi-view rendering for " << keyframe_ids.size() << " keyframes";
    
    // TODO: Extract camera poses from transform tree
    // For now, just store the keyframe IDs
    current_keyframe_ids_ = keyframe_ids;
}

void BatchTrainer::clearCurrentBatch() {
    if (current_gpu_data_.isValid()) {
        gpu_manager_->clearBatchFromGPU(current_gpu_data_);
    }
    
    current_keyframe_ids_.clear();
    current_batch_id_ = 0;
    optimizable_params_.clear();
    
    // Reset optimizer
    optimizer_.reset();
}

void BatchTrainer::setupOptimizer() {
    if (optimizable_params_.empty()) {
        LOG(WARNING) << "No optimizable parameters to setup optimizer";
        return;
    }
    
    // Create Adam optimizer with parameters
    auto adam_options = torch::optim::AdamOptions(config_.learning_rate);
    optimizer_ = std::make_unique<torch::optim::Adam>(optimizable_params_, adam_options);
    
    LOG(INFO) << "Setup optimizer with " << optimizable_params_.size() << " parameter groups";
}

torch::Tensor BatchTrainer::renderBatch(const std::vector<torch::Tensor>& camera_params) {
    // TODO: Implement actual rasterization
    // For now, return dummy output
    return torch::zeros({static_cast<int64_t>(camera_params.size()), 3, config_.initial_height, config_.initial_width},
                       torch::TensorOptions().device(gpu_manager_->getDevice()).dtype(torch::kFloat32));
}

std::vector<torch::Tensor> BatchTrainer::extractGroundTruthImages(const std::vector<uint64_t>& keyframe_ids) {
    std::vector<torch::Tensor> gt_images;
    
    for (uint64_t kf_id : keyframe_ids) {
        auto keyframe = map_store_->getKeyFrame(kf_id);
        if (!keyframe || !keyframe->hasColorImage()) {
            LOG(WARNING) << "Keyframe " << kf_id << " has no color image";
            continue;
        }
        
        // TODO: Convert cv::Mat to torch::Tensor
        // For now, create dummy ground truth
        auto gt_tensor = torch::rand({3, config_.initial_height, config_.initial_width},
                                   torch::TensorOptions().device(gpu_manager_->getDevice()).dtype(torch::kFloat32));
        gt_images.push_back(gt_tensor);
    }
    
    return gt_images;
}

void BatchTrainer::applyParameterTransforms() {
    if (!current_gpu_data_.isValid()) {
        return;
    }
    
    // Apply sigmoid to opacity
    current_gpu_data_.opacities = param_transforms_->applySigmoidOpacity(current_gpu_data_.opacities);
    
    // Apply exponential to scales
    current_gpu_data_.scales = param_transforms_->applyExponentialScaling(current_gpu_data_.scales);
    
    // Normalize rotations
    current_gpu_data_.rotations = param_transforms_->normalizeRotations(current_gpu_data_.rotations);
}

void BatchTrainer::initializeOptimizableParameters() {
    if (!current_gpu_data_.isValid()) {
        LOG(ERROR) << "Cannot initialize parameters: invalid GPU data";
        return;
    }
    
    optimizable_params_.clear();
    
    // Make parameters require gradients
    current_gpu_data_.positions.requires_grad_(true);
    current_gpu_data_.rotations.requires_grad_(true);
    current_gpu_data_.scales.requires_grad_(true);
    current_gpu_data_.opacities.requires_grad_(true);
    current_gpu_data_.sh_coeffs.requires_grad_(true);
    
    // Add to optimizable parameters
    optimizable_params_ = {
        current_gpu_data_.positions,
        current_gpu_data_.rotations,
        current_gpu_data_.scales,
        current_gpu_data_.opacities,
        current_gpu_data_.sh_coeffs
    };
    
    LOG(INFO) << "Initialized " << optimizable_params_.size() << " parameter groups for optimization";
}

} // namespace training
} // namespace gaussian_splatting