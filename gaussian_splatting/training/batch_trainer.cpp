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
      current_keyframe_tensor_(map_store, tf_tree) {
    LOG(INFO) << "Initializing BatchTrainer";

    // Initialize components
    gpu_manager_ = std::make_unique<utils::BatchGPUManager>(config_);
    LOG(INFO) << "INited gpu manager";
    param_transforms_ = std::make_unique<optimization::ParameterTransforms>();
    LOG(INFO) << "Inited param transforms";
    loss_functions_ = std::make_unique<optimization::LossFunctions>();
    LOG(INFO) << "Inteed loss func";
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
    strategy_ = std::make_unique<optimization::Strategy>(std::move(optimizer), std::move(scheduler),
                                                         &current_gaussian_tensors_);
}

void BatchTrainer::copySplatsToBatch(std::vector<core::types::GaussianSplat>& splats) {
    std::cout << "Copying splats to batch" << std::endl;
    std::cout << "Copying splats back to batch" << std::endl;
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
                                 TrainingResults& result, int iteration) {
    LOG(INFO) << "Training keyframe " << keyframe->id << " at iteration " << iteration;

    // Load gaussian tensor and keyframe tensor to GPU
    if (!torch::cuda::is_available()) {
        LOG(ERROR) << "CUDA not available";
        return false;
    }

    // Log GPU memory before training
    if (iteration % 10 == 0) {
        size_t free_mem, total_mem;
        cudaMemGetInfo(&free_mem, &total_mem);
        LOG(INFO) << "GPU Memory before iteration " << iteration << ": "
                  << (total_mem - free_mem) / (1024.0 * 1024.0) << " MB used, "
                  << free_mem / (1024.0 * 1024.0) << " MB free";
        LOG(INFO) << "Gaussian splat count: " << current_gaussian_tensors_.get_positions().size(0);
    }

    // Load keyframe data (already on CPU from loadFromKeyframe)
    current_keyframe_tensor_.loadFromKeyframe(keyframe);
    LOG(INFO) << "Loaded keyframe tensor";

    // Move only the keyframe tensor to GPU (gaussian tensors already on GPU from setupTraining)
    current_keyframe_tensor_.to(torch::kCUDA);

    // Ensure gaussian tensors are on GPU (should be no-op if already there)
    if (!current_gaussian_tensors_.get_positions().is_cuda()) {
        LOG(WARNING) << "Gaussian tensors not on GPU, moving now";
        current_gaussian_tensors_.to(torch::kCUDA);
    }

    LOG(INFO) << "Current device: " << current_keyframe_tensor_.getDevice().type();
    current_gaussian_tensors_.check_stuff = 10;

    LOG(INFO) << "pose on gpu: " << current_keyframe_tensor_.getCameraPose().is_cuda();
    LOG(INFO) << "intrinsics on gpu: " << current_keyframe_tensor_.getCameraIntrinsic().is_cuda();

    auto render_result = renderKeyframe();
    if (!render_result.success) {
        LOG(ERROR) << "Failed to render keyframe " << keyframe->id;
        return false;
    }
    result.rendered_image = render_result.alpha_channel;

    // Set retain_grad to true. This is what the gsplat example
    // does, so setting it here. The autograd understanding is missing
    // (my knowledge of autograd is limited)
    render_result.means2d.retain_grad();

    // The rendered image is "batched" but the ground truth image is not
    auto ground_truth = current_keyframe_tensor_.getImage();
    ground_truth = ground_truth.unsqueeze(0);

    // Put the channel dimension to last
    ground_truth = ground_truth.permute({0, 2, 3, 1});

    auto photometric_loss = optimization::LossFunctions::computePhotometricLoss(
        render_result.rendered_image, ground_truth, keyframe->id);

    std::cout << "Doint photometric loss" << std::endl;
    photometric_loss.backward();

    std::cout << "Doing scale loss" << std::endl;
    auto scale_loss = optimization::LossFunctions::computeScaleRegularizationLoss(
        current_gaussian_tensors_.get_scales());
    std::cout << "current_gaussian_tensors_.get_scales(): "
              << current_gaussian_tensors_.get_scales().is_cuda() << std::endl;
    scale_loss.backward();

    auto opacity_loss = optimization::LossFunctions::computeOpacityRegularizationLoss(
        current_gaussian_tensors_.get_opacities());
    opacity_loss.backward();

    {
        torch::NoGradGuard no_grad;

        std::cout << "Before pose backward: " << current_gaussian_tensors_.check_stuff << std::endl;
        // Pass the actual iteration to strategy
        strategy_->postBackward(render_result, iteration);
        strategy_->step(iteration);
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
        if (std::isnan(view_loss.item<float>()) || std::isinf(view_loss.item<float>())) {
            continue;
        }
        total_loss += view_loss;
        std::cout << "View loss combined: " << view_loss.item<float>()
                  << " total loss utn: " << total_loss.item<float>() << std::endl;
    }
    std::cout << "=====================\n";
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
    std::cout << "=====================\n";

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
