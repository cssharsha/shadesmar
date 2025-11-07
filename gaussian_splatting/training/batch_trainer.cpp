#include "batch_trainer.hpp"
#include <cuda_runtime.h>  // For cudaMemGetInfo
#include <torch/csrc/autograd/function.h>
#include <torch/csrc/autograd/functions/accumulate_grad.h>  // For identifying leaves
#include <cmath>
#include <core/storage/map_store.hpp>
#include <fstream>
#include <logging/logging.hpp>
#include <memory>  // For std::shared_ptr
#include <sstream>
#include <stf/transform_tree.hpp>
#include "core/types/gaussian_splat.hpp"
#include "gaussian_splatting/common/tensor_config.hpp"
#include "gaussian_splatting/optimization/loss_functions.hpp"
#include "gaussian_splatting/optimization/optimizer.hpp"
#include "gaussian_splatting/optimization/scheduler.hpp"
#include "gaussian_splatting/rendering/rasterizer.hpp"
#include "gaussian_splatting/utils/image_utils.hpp"
#include "gaussian_splatting/visualization/training_visualizer.hpp"

namespace gaussian_splatting {
namespace training {

BatchTrainer::BatchTrainer(const TrainingConfig& config,
                           std::shared_ptr<core::storage::MapStore> map_store,
                           std::shared_ptr<stf::TransformTree> tf_tree)
    : config_(config),
      map_store_(map_store),
      tf_tree_(tf_tree),
      current_keyframe_tensor_(map_store, tf_tree, config_.base_link_, config_.camera_frame_) {
    LOG(INFO) << "Initializing BatchTrainer";

    // Initialize components
    gpu_manager_ = std::make_unique<utils::BatchGPUManager>(config_);
    LOG(INFO) << "INited gpu manager";
    param_transforms_ = std::make_unique<optimization::ParameterTransforms>();
    LOG(INFO) << "Inited param transforms";
    loss_functions_ = std::make_unique<optimization::LossFunctions>();
    LOG(INFO) << "Inteed loss func";

    // Configure loss functions with debug output path
    optimization::LossFunctions::config.debug_output_path = config_.debug_output_path;
    LOG(INFO) << "Set loss functions debug output path to: " << config_.debug_output_path;

    rasterizer_ = std::make_unique<rendering::DifferentiableRasterizer>();
    LOG(INFO) << "Inted rasterizer";

    if (!gpu_manager_->isDeviceAvailable()) {
        LOG(WARNING) << "GPU device not available, using CPU";
    } else {
        LOG(INFO) << "GPU device is available";
    }
    training_thread_ = std::thread(&BatchTrainer::trainingThreadLoop, this);
}

BatchTrainer::~BatchTrainer() {
    // Signal shutdown and wake up training thread
    shutdown_requested_ = true;
    keyframe_train_queue_cv_.notify_all();

    // Wait for training thread to finish if it's joinable
    if (training_thread_.joinable()) {
        training_thread_.join();
    }

    clearCurrentBatch();
}

void BatchTrainer::setupTraining(const core::types::GaussianSplatBatch& batch) {
    clearCurrentBatch();
    current_gaussian_tensors_.fromSplats(batch.splats);

    // Move tensors to GPU BEFORE setting requires_grad and initializing optimizer
    // This ensures the optimizer holds references to GPU tensors, not CPU tensors
    // IMPORTANT: Must move to device BEFORE setting requires_grad to keep tensors as leaves
    if (torch::cuda::is_available()) {
        current_gaussian_tensors_.to(torch::kCUDA);
    }

    // Set requires_grad AFTER moving to GPU so tensors remain as leaf nodes
    current_gaussian_tensors_.setRequiresGrad(true);

    auto optimizer = std::make_unique<optimization::Optimizer>();
    optimizer->initialize(current_gaussian_tensors_);
    // Set the proper gamma
    auto scheduler = std::make_unique<optimization::Scheduler>(optimizer->getOptimizer(), 0.5);
    strategy_ = std::make_unique<optimization::DefaultStrategy>(std::move(optimizer), std::move(scheduler),
                                                                &current_gaussian_tensors_);
}

void BatchTrainer::copySplatsToBatch(std::vector<core::types::GaussianSplat>& splats) {
    LOG(INFO) << "Copying splats to batch";
    LOG(INFO) << "Copying splats back to batch";
    splats.resize(current_gaussian_tensors_.get_positions().size(0));
    splats = current_gaussian_tensors_.toSplats();
}

void BatchTrainer::trainingThreadLoop() {
    int global_iteration = 0;
    for (uint32_t epoch = 0; epoch < config_.max_iterations_per_batch && !shutdown_requested_;
         ++epoch) {
        LOG(INFO) << "Training epoch " << epoch;

        while (!shutdown_requested_) {
            core::storage::KeyFramePtr keyframe;

            // Im releasing the lock inside his block and recapturing it
            // multiple times.
            {
                std::unique_lock<std::mutex> lock(keyframe_train_queue_mutex_);
                keyframe_train_queue_cv_.wait(
                    lock, [this] { return !keyframe_train_queue_.empty() || shutdown_requested_; });

                if (shutdown_requested_) {
                    return;
                }

                keyframe = keyframe_train_queue_.front();
                keyframe_train_queue_.pop();

                lock.unlock();

                LOG(INFO) << "Training keyframe " << keyframe->id << " at global iteration "
                          << global_iteration;
                TrainingResults results;
                if (!trainKeyframe(keyframe, results, global_iteration)) {
                    LOG(ERROR) << "Failed to train keyframe " << keyframe->id;
                    std::lock_guard<std::mutex> requeue_lock(keyframe_train_queue_mutex_);
                    keyframe_train_queue_.push(keyframe);
                } else {
                    keyframe_train_count_[keyframe->id]++;
                    global_iteration++;
                }
            }
        }
        LOG(INFO) << "Completed epoch " << epoch;
    }

    LOG(INFO) << "Training thread completed all epochs";
}

bool BatchTrainer::trainBatch(uint32_t batch_id, const training::KeyframeBatch& keyframe_batch,
                              TrainingResults& results) {
    // Load batch from map store
    auto batch = map_store_->getGaussianSplatBatch(batch_id);
    if (!batch) {
        LOG(ERROR) << "Failed to load batch " << batch_id;
        return false;
    }
    return trainBatch(*batch, keyframe_batch, results);
}

bool BatchTrainer::trainBatch(const core::types::GaussianSplatBatch& batch,
                              const training::KeyframeBatch& keyframe_batch,
                              TrainingResults& results,
                              std::shared_ptr<visualization::RerunTrainingVisualizer> viz) {
    LOG(INFO) << "Training batch " << batch.batch_id;

    // Setup batch for training
    if (!setupBatchForTraining(batch, keyframe_batch)) {
        LOG(ERROR) << "Failed to setup batch " << batch.batch_id << " for training";
        return false;
    }

    // Visualize the initial state of the keyframe batch before training
    // if (viz) {
    //     std::cout << "Visualize keyframe batch vizzz" << std::endl;
    //     viz->visualizeKeyframeBatch(keyframe_batch, "initial_state");
    // }

    // Training loop
    is_training_ = true;
    results.success = false;

    // if (viz) {
    //     viz->visualizeCurrentSplats(current_gaussian_tensors_.toSplats(), 0.5);
    // }

    for (int iter = 0; iter < config_.max_iterations_per_batch; ++iter) {
        if (!performTrainingStep(iter, viz)) {
            LOG(ERROR) << "Training step " << iter << " failed for batch " << batch.batch_id;
            break;
        }

        // Log progress periodically
        if (iter % 100 == 0) {
            LOG(INFO) << "Batch " << batch.batch_id << " iteration " << iter
                      << " loss: " << results.total_loss;
        }

        results.iterations_completed = iter + 1;
    }

    is_training_ = false;
    results.success = true;

    LOG(INFO) << "Completed training batch " << batch.batch_id << " after "
              << results.iterations_completed << " iterations";

    return true;
}

bool BatchTrainer::setupBatchForTraining(const core::types::GaussianSplatBatch& batch,
                                         const training::KeyframeBatch& keyframe_batch) {
    LOG(INFO) << "Setting up batch " << batch.batch_id << " for training";

    // Clear previous batch
    clearCurrentBatch();

    // Load batch to GPU
    if (!gpu_manager_->loadBatchToTensors(batch, current_gaussian_tensors_)) {
        LOG(ERROR) << "Failed to load batch to GPU";
        return false;
    }

    current_batch_id_ = batch.batch_id;
    current_keyframe_batch_ = keyframe_batch;

    if (!gpu_manager_->loadToGPU(current_keyframe_batch_) ||
        !gpu_manager_->loadToGPU(current_gaussian_tensors_)) {
        LOG(ERROR) << "Failed to load batch to GPU";
        return false;
    }

    // Initialize optimizable parameters
    // initializeOptimizableParameters();

    // Setup optimizer
    setupOptimizer();

    LOG(INFO) << "Batch setup complete: " << current_gaussian_tensors_.get_positions().size(0)
              << " gaussians, " << current_keyframe_batch_.batch_size << " keyframes";

    return true;
}

// Helper to create a unique ID for a node pointer for DOT file
std::string get_node_id_dot(const torch::autograd::Node* node_ptr) {
    std::stringstream ss;
    ss << "node" << reinterpret_cast<uintptr_t>(node_ptr);
    return ss.str();
}

// Recursively traverses the graph and writes nodes and edges to the DOT file
void print_graph_recursive_dot(std::ostream& out,
                               const std::shared_ptr<torch::autograd::Node>& node,
                               std::set<torch::autograd::Node*>& visited) {
    if (!node || visited.count(node.get())) {
        return;
    }
    visited.insert(node.get());

    std::string node_id = get_node_id_dot(node.get());
    std::string node_label = node->name();

    // For AccumulateGrad nodes (leaves), add tensor details to the label
    if (auto accumulate_grad_node =
            std::dynamic_pointer_cast<torch::autograd::AccumulateGrad>(node)) {
        const auto& variable = accumulate_grad_node->variable;
        std::stringstream details;
        details << "\nShape: " << variable.sizes();
        details << "\nDevice: " << variable.device();
        details << "\nRequires Grad: " << (variable.requires_grad() ? "true" : "false");
        node_label += details.str();
        out << "  " << node_id << " [label=\"" << node_label
            << "\", shape=box, style=filled, fillcolor=lightblue];\n";
    } else {
        // For other nodes, just use the function name
        out << "  " << node_id << " [label=\"" << node_label << "\"];\n";
    }

    // Traverse to the next nodes in the graph
    for (const auto& edge : node->next_edges()) {
        if (auto next_node = edge.function) {
            std::string next_node_id = get_node_id_dot(next_node.get());
            out << "  " << node_id << " -> " << next_node_id << ";\n";
            print_graph_recursive_dot(out, next_node, visited);
        }
    }
}

// Main function to write the autograd graph of a tensor to a DOT file
void write_graph_to_disk(const torch::Tensor& tensor, const std::string& filename) {
    if (!tensor.grad_fn()) {
        LOG(WARNING) << "Tensor does not have a grad_fn. Cannot write graph.";
        return;
    }

    std::ofstream out_file(filename);
    if (!out_file.is_open()) {
        LOG(ERROR) << "Failed to open file for writing graph: " << filename;
        return;
    }

    out_file << "digraph AutogradGraph {\n";
    out_file << "  rankdir=TB; // Top-to-bottom layout\n";
    out_file << "  node [shape=ellipse, style=filled, fillcolor=lightgrey];\n";

    std::set<torch::autograd::Node*> visited;
    print_graph_recursive_dot(out_file, tensor.grad_fn(), visited);

    out_file << "}\n";
    LOG(INFO) << "Saved autograd graph to " << filename;
}

bool BatchTrainer::trainKeyframe(const core::storage::KeyFramePtr& keyframe,
                                 TrainingResults& result, int iteration,
                                 std::shared_ptr<visualization::RerunTrainingVisualizer> viz) {
    LOG(INFO) << "Training keyframe " << keyframe->id << " at iteration " << iteration;

    if (!torch::cuda::is_available()) {
        LOG(ERROR) << "CUDA not available";
        return false;
    }

    if (iteration % 10 == 0) {
        size_t free_mem, total_mem;
        cudaMemGetInfo(&free_mem, &total_mem);
        LOG(INFO) << "GPU Memory before iteration " << iteration << ": "
                  << (total_mem - free_mem) / (1024.0 * 1024.0) << " MB used, "
                  << free_mem / (1024.0 * 1024.0) << " MB free";
        LOG(INFO) << "Gaussian splat count: " << current_gaussian_tensors_.get_positions().size(0);
    }

    current_keyframe_tensor_.loadFromKeyframe(keyframe);
    LOG(INFO) << "Loaded keyframe tensor";

    current_keyframe_tensor_.to(torch::kCUDA);

    if (!current_gaussian_tensors_.get_positions().is_cuda()) {
        LOG(WARNING) << "Gaussian tensors not on GPU, moving now";
        current_gaussian_tensors_.to(torch::kCUDA);
    }

    LOG(INFO) << "Current device: " << current_keyframe_tensor_.getDevice().type();
    current_gaussian_tensors_.check_stuff = 10;

    LOG(INFO) << "pose on gpu: " << current_keyframe_tensor_.getCameraPose().is_cuda();
    LOG(INFO) << "intrinsics on gpu: " << current_keyframe_tensor_.getCameraIntrinsic().is_cuda();

    // Debug: Check Gaussian splat properties before rendering
    {
        torch::NoGradGuard no_grad;
        auto positions = current_gaussian_tensors_.get_positions();
        auto opacities = current_gaussian_tensors_.get_opacities();
        auto scales = current_gaussian_tensors_.get_scales();

        LOG(INFO) << "=== Some more debug info (Iteration " << iteration << ") ===";
        LOG(INFO) << "Number of splats: " << positions.size(0);
        LOG(INFO) << "Positions - min: " << common::itemAs(positions.min())
                  << ", max: " << common::itemAs(positions.max())
                  << ", mean: " << common::itemAs(positions.mean());
        LOG(INFO) << "Opacities (logit) - min: " << common::itemAs(opacities.min())
                  << ", max: " << common::itemAs(opacities.max())
                  << ", mean: " << common::itemAs(opacities.mean());

        auto opacity_probs = torch::sigmoid(opacities);
        LOG(INFO) << "Opacities (sigmoid) - min: " << common::itemAs(opacity_probs.min())
                  << ", max: " << common::itemAs(opacity_probs.max())
                  << ", mean: " << common::itemAs(opacity_probs.mean());

        LOG(INFO) << "Scales (log) - min: " << common::itemAs(scales.min())
                  << ", max: " << common::itemAs(scales.max())
                  << ", mean: " << common::itemAs(scales.mean());

        auto actual_scales = torch::exp(scales);
        LOG(INFO) << "Scales (exp) - min: " << common::itemAs(actual_scales.min())
                  << ", max: " << common::itemAs(actual_scales.max())
                  << ", mean: " << common::itemAs(actual_scales.mean());
        LOG(INFO) << "=================================";
    }

    auto render_result = renderKeyframe();
    if (!render_result.success) {
        LOG(ERROR) << "Failed to render keyframe " << keyframe->id;
        return false;
    }
    result.rendered_image = render_result.rendered_image;

    // Set retain_grad to true. This is what the gsplat example
    // does which is set in the preBackward step, so setting it here.
    render_result.means2d.retain_grad();

    auto ground_truth = current_keyframe_tensor_.getImage();
    ground_truth = ground_truth.unsqueeze(0);

    // Put the channel dimension to last
    ground_truth = ground_truth.permute({0, 2, 3, 1});

    auto photometric_loss = optimization::LossFunctions::computePhotometricLoss(
        render_result.rendered_image, ground_truth, keyframe->id);
    LOG(INFO) << "Photometric loss: " << common::itemAs(photometric_loss);

    // Log photometric loss to Rerun for time series plot (RED)
    if (viz) {
        viz->logLoss("training/loss/photometric", common::itemAs(photometric_loss), iteration,
                     255, 0, 0);  // Red
    }

    LOG(INFO) << "Doint photometric loss";
    photometric_loss.backward();

    LOG(INFO) << "Doing scale loss";
    auto scale_loss = optimization::LossFunctions::computeScaleRegularizationLoss(
        current_gaussian_tensors_.get_scales());
    LOG(INFO) << "Scale loss: " << common::itemAs(scale_loss);
    LOG(INFO) << "current_gaussian_tensors_.get_scales(): "
              << current_gaussian_tensors_.get_scales().is_cuda();

    // Log scale loss to Rerun for time series plot (GREEN)
    if (viz) {
        viz->logLoss("training/loss/scale", common::itemAs(scale_loss), iteration,
                     0, 255, 0);  // Green
    }

    scale_loss.backward();

    auto opacity_loss = optimization::LossFunctions::computeOpacityRegularizationLoss(
        current_gaussian_tensors_.get_opacities());
    LOG(INFO) << "Opacity loss: " << common::itemAs(opacity_loss);

    // Log opacity loss to Rerun for time series plot (BLUE)
    if (viz) {
        viz->logLoss("training/loss/opacity", common::itemAs(opacity_loss), iteration,
                     0, 0, 255);  // Blue
    }

    opacity_loss.backward();

    // Log total loss to Rerun for time series plot (MAGENTA)
    if (viz) {
        float total_loss =
            common::itemAs(photometric_loss) + common::itemAs(scale_loss) + common::itemAs(opacity_loss);
        viz->logLoss("training/loss/total", total_loss, iteration,
                     255, 0, 255);  // Magenta
    }

    // Debug: Check gradient magnitudes after backward pass
    {
        torch::NoGradGuard no_grad;
        LOG(INFO) << "=== Gradient Debug Info (Iteration " << iteration << ") ===";

        auto& positions = current_gaussian_tensors_.get_positions();
        auto& opacities = current_gaussian_tensors_.get_opacities();
        auto& scales = current_gaussian_tensors_.get_scales();
        auto& rotations = current_gaussian_tensors_.get_rotations();
        auto& sh_0 = current_gaussian_tensors_.get_sh_0();
        auto& sh_N = current_gaussian_tensors_.get_sh_N();

        if (positions.grad().defined() && positions.grad().numel() > 0) {
            auto pos_grad_norm = common::itemAs(positions.grad().norm());
            auto pos_grad_mean = common::itemAs(positions.grad().abs().mean());
            LOG(INFO) << "Positions grad - norm: " << pos_grad_norm
                      << ", mean abs: " << pos_grad_mean;
        } else {
            LOG(WARNING) << "Positions grad is NOT defined or empty!";
        }

        if (opacities.grad().defined() && opacities.grad().numel() > 0) {
            auto opacity_grad_norm = common::itemAs(opacities.grad().norm());
            auto opacity_grad_mean = common::itemAs(opacities.grad().abs().mean());
            LOG(INFO) << "Opacities grad - norm: " << opacity_grad_norm
                      << ", mean abs: " << opacity_grad_mean;
        } else {
            LOG(WARNING) << "Opacities grad is NOT defined or empty!";
        }

        if (scales.grad().defined() && scales.grad().numel() > 0) {
            auto scale_grad_norm = common::itemAs(scales.grad().norm());
            auto scale_grad_mean = common::itemAs(scales.grad().abs().mean());
            LOG(INFO) << "Scales grad - norm: " << scale_grad_norm
                      << ", mean abs: " << scale_grad_mean;
        } else {
            LOG(WARNING) << "Scales grad is NOT defined or empty!";
        }

        if (rotations.grad().defined() && rotations.grad().numel() > 0) {
            auto rot_grad_norm = common::itemAs(rotations.grad().norm());
            auto rot_grad_mean = common::itemAs(rotations.grad().abs().mean());
            LOG(INFO) << "Rotations grad - norm: " << rot_grad_norm
                      << ", mean abs: " << rot_grad_mean;
        } else {
            LOG(WARNING) << "Rotations grad is NOT defined or empty!";
        }

        if (sh_0.grad().defined() && sh_0.grad().numel() > 0) {
            auto sh0_grad_norm = common::itemAs(sh_0.grad().norm());
            auto sh0_grad_mean = common::itemAs(sh_0.grad().abs().mean());
            LOG(INFO) << "SH_0 grad - norm: " << sh0_grad_norm << ", mean abs: " << sh0_grad_mean;
        } else {
            LOG(WARNING) << "SH_0 grad is NOT defined or empty!";
        }

        if (sh_N.grad().defined() && sh_N.grad().numel() > 0) {
            auto shN_grad_norm = common::itemAs(sh_N.grad().norm());
            auto shN_grad_mean = common::itemAs(sh_N.grad().abs().mean());
            LOG(INFO) << "SH_N grad - norm: " << shN_grad_norm << ", mean abs: " << shN_grad_mean;
        } else {
            LOG(WARNING) << "SH_N grad is NOT defined or empty!";
        }

        LOG(INFO) << "=================================";
    }

    // Log tensor addresses BEFORE postBackward to check if they change
    LOG(INFO) << "=== Tensor Addresses BEFORE postBackward ===";
    LOG(INFO) << "Positions: " << current_gaussian_tensors_.get_positions().data_ptr();
    LOG(INFO) << "Scales: " << current_gaussian_tensors_.get_scales().data_ptr();
    LOG(INFO) << "Opacities: " << current_gaussian_tensors_.get_opacities().data_ptr();
    LOG(INFO) << "Positions grad: " << current_gaussian_tensors_.get_positions().grad().data_ptr();
    LOG(INFO) << "Scales grad: " << current_gaussian_tensors_.get_scales().grad().data_ptr();
    LOG(INFO) << "============================================";

    {
        torch::NoGradGuard no_grad;

        LOG(INFO) << "Before pose backward: " << current_gaussian_tensors_.check_stuff;

        // Store values before optimizer step
        auto positions_before = current_gaussian_tensors_.get_positions().clone();
        auto opacities_before = current_gaussian_tensors_.get_opacities().clone();
        auto scales_before = current_gaussian_tensors_.get_scales().clone();

        // IMPORTANT: Match gsplat order: backward() -> optimizer.step() -> step_post_backward()
        // Optimizer step MUST happen BEFORE densification, otherwise densification
        // replaces tensors and destroys gradients (causing crash at iteration 50+)

        // Lock mutex to prevent concurrent cloning while modifying tensors
        {
            std::lock_guard<std::mutex> lock(tensors_mutex_);
            strategy_->step(iteration);

            LOG(INFO) << "=== After optimizer step ===";

            // Densification happens AFTER optimizer step (when it's safe to replace tensors)
            // This may grow/prune splats, changing tensor sizes
            strategy_->postBackward(render_result, iteration);
        }

        LOG(INFO) << "=== After densification (postBackward) ===";
        LOG(INFO) << "Positions: " << current_gaussian_tensors_.get_positions().data_ptr();
        LOG(INFO) << "Scales: " << current_gaussian_tensors_.get_scales().data_ptr();
        LOG(INFO) << "Opacities: " << current_gaussian_tensors_.get_opacities().data_ptr();
        LOG(INFO) << "============================================";

        // Check if parameters actually changed
        // Note: Skip this check if densification changed tensor sizes
        auto positions_after = current_gaussian_tensors_.get_positions();
        auto opacities_after = current_gaussian_tensors_.get_opacities();
        auto scales_after = current_gaussian_tensors_.get_scales();

        LOG(INFO) << "=== Parameter Change Check ===";
        if (positions_after.size(0) == positions_before.size(0)) {
            // Sizes match - densification didn't happen, so we can compare
            float pos_diff = common::itemAs((positions_after - positions_before).abs().max());
            float opacity_diff = common::itemAs((opacities_after - opacities_before).abs().max());
            float scale_diff = common::itemAs((scales_after - scales_before).abs().max());

            LOG(INFO) << "Max position change: " << pos_diff;
            LOG(INFO) << "Max opacity change: " << opacity_diff;
            LOG(INFO) << "Max scale change: " << scale_diff;
        } else {
            // Sizes changed - densification happened
            LOG(INFO) << "Splat count changed: " << positions_before.size(0)
                      << " -> " << positions_after.size(0);
            LOG(INFO) << "Skipping parameter diff (densification occurred)";
        }
        LOG(INFO) << "Positions tensor address: " << positions_after.data_ptr();
        LOG(INFO) << "Opacities tensor address: " << opacities_after.data_ptr();
        LOG(INFO) << "Scales tensor address: " << scales_after.data_ptr();
        LOG(INFO) << "==============================";
    }

    // Synchronize CUDA to ensure all operations complete
    if (torch::cuda::is_available()) {
        torch::cuda::synchronize();
        // Note: LibTorch C++ API doesn't expose empty_cache() like Python
        // Memory will be managed automatically by PyTorch's caching allocator
    }

    return true;
}

bool BatchTrainer::performTrainingStep(
    int iteration, std::shared_ptr<visualization::RerunTrainingVisualizer> viz) {
    if (!is_training_ || !current_gaussian_tensors_.isValid()) {
        LOG(ERROR) << "Invalid training state for step " << iteration;
        return false;
    }

    // Apply parameter transforms
    // applyParameterTransforms();

    // Render the batch
    auto rendered = renderBatch();
    LOG(INFO) << "Rendered batch " << rendered.rendered_image.sizes();

    // Extract ground truth images
    auto ground_truth = current_keyframe_batch_.images;

    if (!ground_truth.defined() || ground_truth.size(0) == 0) {
        LOG(WARNING) << "No ground truth images available for iteration " << iteration;
        return true;  // Continue training
    }
    LOG(INFO) << "is it here? Ground truth images: " << ground_truth.sizes();
    for (size_t i = 0; i < rendered.rendered_image.size(0); ++i) {  // Limit to first 3 images
        try {
            std::string output_dir = "/data/robot/log/batch_" + std::to_string(current_batch_id_) +
                                     "/iter_" + std::to_string(iteration);
            // Convert tensors to cv::Mat
            auto rendered_mat = utils::tensorToMat(rendered.rendered_image[i], true);  // CHW format
            auto gt_mat = utils::tensorToMat(ground_truth[i], true);                   // CHW format

            // Combine images horizontally (rendered | ground_truth)
            auto combined_mat = utils::combineImagesHorizontally(rendered_mat, gt_mat);

            // Create filename
            std::string filename = "batch_" + std::to_string(current_batch_id_) + "_view_" +
                                   std::to_string(i) + ".png";

            // Write to output directory
            utils::writeImageToDirectory(combined_mat, output_dir, filename);

        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to output comparison image for view " << i << ": " << e.what();
        }
    }

    // TODO: There is a pre backward stuff that is called which might be useful?
    // Compute loss
    torch::Tensor total_loss =
        torch::zeros({1}, torch::TensorOptions().device(gpu_manager_->getDevice()));

    for (size_t i = 0; i < rendered.rendered_image.size(0); ++i) {
        auto view_rendered = rendered.rendered_image[i];
        auto view_gt = ground_truth[i];

        auto view_loss =
            loss_functions_->computeCombinedLoss(view_rendered, view_gt, config_.d_ssim_lambda);
        if (std::isnan(common::itemAs(view_loss)) || std::isinf(common::itemAs(view_loss))) {
            continue;
        }
        total_loss += view_loss;
        LOG(INFO) << "View loss combined: " << common::itemAs(view_loss)
                  << " total loss utn: " << common::itemAs(total_loss);
    }
    LOG(INFO) << "=====================";
    LOG(INFO) << "Total loss combined: " << total_loss;
    // --- Write the Computation Graph to Disk ---
    if (total_loss.grad_fn()) {
        std::string output_dir = "/data/robot/log/batch_" + std::to_string(current_batch_id_) +
                                 "/iter_" + std::to_string(iteration);
        std::string graph_filename = output_dir + "/autograd_graph.dot";
        write_graph_to_disk(total_loss, graph_filename);
    } else {
        LOG(WARNING) << "Total loss tensor does not have a grad_fn. Cannot write graph.";
    }
    current_gaussian_tensors_.printGradInfo();
    LOG(INFO) << "=====================";

    // Output comparison images every 100 iterations
    if (iteration % 99 == 0) {
        for (size_t i = 0; i < rendered.rendered_image.size(0); ++i) {  // Limit to first 3 images
            try {
                std::string output_dir = "/data/robot/log/batch_" +
                                         std::to_string(current_batch_id_) + "/iter_" +
                                         std::to_string(iteration);
                // Convert tensors to cv::Mat
                auto rendered_mat =
                    utils::tensorToMat(rendered.rendered_image[i], true);  // CHW format
                auto gt_mat = utils::tensorToMat(ground_truth[i], true);   // CHW format

                // Combine images horizontally (rendered | ground_truth)
                auto combined_mat = utils::combineImagesHorizontally(rendered_mat, gt_mat);

                // Create filename
                std::string filename = "batch_" + std::to_string(current_batch_id_) + "_view_" +
                                       std::to_string(i) + ".png";

                // Write to output directory
                utils::writeImageToDirectory(combined_mat, output_dir, filename);

            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to output comparison image for view " << i << ": "
                             << e.what();
            }
        }
    }

    // TODO: There is supposed to be one optimizer for
    // each of the different parameters which might be actually required
    // for different learning rate?
    if (optimizer_) {
        optimizer_->zero_grad();
        total_loss.backward();
        optimizer_->step();
    }

    // Visualize the splats after the optimizer step
    if (viz) {
        auto all_splats = current_gaussian_tensors_.toSplats();
        std::vector<core::types::GaussianSplat> filtered_splats;

        // Define the 10x10x10m bounding box centered at the origin
        constexpr double min_bound = -5.0;
        constexpr double max_bound = 5.0;

        for (const auto& splat : all_splats) {
            const auto& pos = splat.position;
            if (pos.x() >= min_bound && pos.x() <= max_bound && pos.y() >= min_bound &&
                pos.y() <= max_bound && pos.z() >= min_bound && pos.z() <= max_bound) {
                filtered_splats.push_back(splat);
            }
        }
        viz->visualizeCurrentSplats(filtered_splats, iteration);
    }

    // TODO: Big one!!! Add a post_backward step where you actually
    // update (densify/prune) the splats. This is where the Strategy
    // kicks in.

    return true;
}

void BatchTrainer::clearCurrentBatch() {
    if (current_gaussian_tensors_.isValid()) {
        gpu_manager_->clearBatchFromGPU(current_gaussian_tensors_);
    }

    current_keyframe_batch_.clear();
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

rendering::RasterizationOutput BatchTrainer::renderKeyframe() {
    if (!current_gaussian_tensors_.isValid() || !current_keyframe_tensor_.isValid()) {
        LOG(ERROR) << "Cannot render batch: invalid GPU data or keyframe batch "
                   << current_gaussian_tensors_.isValid() << " "
                   << current_keyframe_tensor_.isValid();
        return rendering::RasterizationOutput();
    }

    auto rasterisation_output =
        rasterizer_->rasterize(current_gaussian_tensors_, current_keyframe_tensor_);
    return rasterisation_output;
}

rendering::RasterizationOutput BatchTrainer::renderBatch() {
    if (!current_gaussian_tensors_.isValid() || !current_keyframe_batch_.isValid()) {
        LOG(ERROR) << "Cannot render batch: invalid GPU data or keyframe batch "
                   << current_gaussian_tensors_.isValid() << " "
                   << current_keyframe_batch_.isValid();
        return rendering::RasterizationOutput();
    }

    // utils::GaussianTensors gaussians(current_gpu_data_);
    torch::Tensor rendered_images;

    // auto stacked_camera_poses = torch::stack(current_keyframe_batch_.camera_poses, 0);
    // auto stacked_camera_intrinsics = torch::Stack(current_keyframe_batch_.camera_intrinsics, 0);
    auto output = rasterizer_->rasterize(
        current_gaussian_tensors_, current_keyframe_batch_.camera_poses,
        current_keyframe_batch_.camera_intrinsics, current_keyframe_batch_.image_width,
        current_keyframe_batch_.image_height);
    LOG(INFO) << "Finished rasterizing";

    // for (int i = 0; i < current_keyframe_batch_.batch_size; ++i) {
    //     auto camera_pose = current_keyframe_batch_.camera_poses[i];
    //     auto camera_intrinsics = current_keyframe_batch_.camera_intrinsics[i];
    //
    //     auto output = rasterizer_->rasterize(current_gaussian_tensors_, camera_pose,
    //                                          camera_intrinsics,
    //                                          current_keyframe_batch_.image_width,
    //                                          current_keyframe_batch_.image_height);
    //
    //     rendered_images.push_back(output.rendered_image);
    // }

    if (!output.rendered_image.size(0)) {
        std::cerr << "Should not have been no images" << std::endl;
        output.success = false;
    } else {
        output.success = true;
    }

    return output;
}

// void BatchTrainer::applyParameterTransforms() {
//     if (!current_gaussian_tensors_.isValid()) {
//         return;
//     }
//
//     // Apply sigmoid to opacity
//     current_gaussian_tensors_.get_opacities() =
//         param_transforms_->applySigmoidOpacity(current_gaussian_tensors_.get_opacities());
//
//     // Apply exponential to scales
//     current_gaussian_tensors_.get_scales() =
//         param_transforms_->applyExponentialScaling(current_gaussian_tensors_.get_scales());
//
//     // Normalize rotations
//     current_gaussian_tensors_.get_rotations() =
//         param_transforms_->normalizeRotations(current_gaussian_tensors_.get_rotations());
// }

}  // namespace training
}  // namespace gaussian_splatting
