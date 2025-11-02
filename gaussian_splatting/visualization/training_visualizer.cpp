#include "training_visualizer.hpp"
#include <algorithm>
#include <logging/logging.hpp>
#include <numeric>
#include <opencv2/opencv.hpp>

namespace gaussian_splatting {
namespace visualization {

RerunTrainingVisualizer::RerunTrainingVisualizer(const std::string& recording_id,
                                                 const std::string& host, uint16_t port)
    : recording_id_(recording_id),
      host_(host),
      port_(port),
      initialized_(false),
      current_epoch_(0),
      current_batch_(0),
      total_batches_processed_(0),
      log_frequency_(10),
      visualize_batches_(true),
      visualize_splats_(true),
      current_timestamp_(0.0) {
    LOG(INFO) << "RerunTrainingVisualizer created with recording ID: " << recording_id_;
}

RerunTrainingVisualizer::~RerunTrainingVisualizer() {
    shutdown();
}

bool RerunTrainingVisualizer::initialize() {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (initialized_) {
        LOG(WARNING) << "Training visualizer already initialized";
        return true;
    }

    try {
        // Create Rerun visualizer with shared recording ID to combine with rosbag visualization
        std::string shared_recording_id = "shadesmar_combined";
        rerun_viz_ = std::make_shared<viz::RerunVisualizer>("gs_training", shared_recording_id,
                                                            host_, port_);

        if (!rerun_viz_->initialize(false)) {
            LOG(ERROR) << "Failed to initialize Rerun visualizer";
            return false;
        }

        // Setup training visualization layout
        createLossCurveTimeSeries();

        // Log initial training state
        logText("training/status", "Training visualizer initialized");

        training_start_time_ = std::chrono::steady_clock::now();
        initialized_ = true;

        LOG(INFO) << "Training visualizer initialized successfully";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to initialize training visualizer: " << e.what();
        return false;
    }
}

void RerunTrainingVisualizer::shutdown() {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_) {
        return;
    }

    // Don't use LOG() or logText() during shutdown - glog may be shutting down
    std::cerr << "Training visualizer shutting down..." << std::endl;

    if (rerun_viz_) {
        rerun_viz_->disconnect();
        rerun_viz_.reset();
    }

    initialized_ = false;
    std::cerr << "Training visualizer shut down" << std::endl;
}

void RerunTrainingVisualizer::logTrainingMetrics(const TrainingMetrics& metrics) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_) {
        return;
    }

    // Update internal state
    current_epoch_ = metrics.epoch;
    current_batch_ = metrics.batch_in_epoch;
    total_batches_processed_ = metrics.total_batches_processed;

    // Log scalar metrics
    logScalar("training/metrics/loss/total", metrics.total_loss, metrics.total_batches_processed);
    logScalar("training/metrics/loss/l1", metrics.l1_loss, metrics.total_batches_processed);
    logScalar("training/metrics/loss/ssim", metrics.ssim_loss, metrics.total_batches_processed);
    logScalar("training/metrics/learning_rate", metrics.learning_rate,
              metrics.total_batches_processed);
    logScalar("training/metrics/timing/batch_time_ms", metrics.batch_processing_time_ms,
              metrics.total_batches_processed);
    logScalar("training/metrics/splats/count", static_cast<double>(metrics.num_splats),
              metrics.total_batches_processed);
    logScalar("training/metrics/batch/keyframes",
              static_cast<double>(metrics.num_keyframes_in_batch), metrics.total_batches_processed);

    // Update loss history
    loss_history_.addLoss(metrics.total_loss, metrics.l1_loss, metrics.ssim_loss,
                          metrics.total_batches_processed);

    // Log training progress text
    std::string progress_text = "Epoch: " + std::to_string(metrics.epoch) +
                                ", Batch: " + std::to_string(metrics.batch_in_epoch) +
                                ", Loss: " + std::to_string(metrics.total_loss) +
                                ", Splats: " + std::to_string(metrics.num_splats);

    logText("training/progress", progress_text);

    LOG(INFO) << "Logged training metrics for epoch " << metrics.epoch << ", batch "
              << metrics.batch_in_epoch << ", loss: " << metrics.total_loss;
}

void RerunTrainingVisualizer::logEpochStart(uint32_t epoch, size_t total_batches) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_) {
        return;
    }

    std::string epoch_text = "Starting epoch " + std::to_string(epoch) + " with " +
                             std::to_string(total_batches) + " batches";

    logText("training/epochs/current", epoch_text);
    logScalar("training/epochs/number", static_cast<double>(epoch), total_batches_processed_);

    LOG(INFO) << "Logged epoch start: " << epoch_text;
}

void RerunTrainingVisualizer::logEpochEnd(uint32_t epoch, double avg_loss, double epoch_time_s) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_) {
        return;
    }

    std::string epoch_text = "Completed epoch " + std::to_string(epoch) +
                             ", avg loss: " + std::to_string(avg_loss) +
                             ", time: " + std::to_string(epoch_time_s) + "s";

    logText("training/epochs/completed", epoch_text);
    logScalar("training/epochs/avg_loss", avg_loss, epoch);
    logScalar("training/epochs/time_seconds", epoch_time_s, epoch);

    LOG(INFO) << "Logged epoch end: " << epoch_text;
}

void RerunTrainingVisualizer::updateLossCurves(double total_loss, double l1_loss, double ssim_loss,
                                               uint32_t batch_idx) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_) {
        return;
    }

    // Log individual loss components
    logScalar("training/loss_curves/total", total_loss, batch_idx);
    logScalar("training/loss_curves/l1", l1_loss, batch_idx);
    logScalar("training/loss_curves/ssim", ssim_loss, batch_idx);

    // Update loss history
    loss_history_.addLoss(total_loss, l1_loss, ssim_loss, batch_idx);
}

void RerunTrainingVisualizer::visualizeLossHistory() {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_ || loss_history_.total_losses.empty()) {
        return;
    }

    // Create loss history visualization
    std::string history_text =
        "Loss history (last " + std::to_string(loss_history_.total_losses.size()) + " batches):\n";

    // Get recent losses for summary
    size_t recent_count = std::min(static_cast<size_t>(10), loss_history_.total_losses.size());
    double recent_avg = 0.0;

    for (size_t i = loss_history_.total_losses.size() - recent_count;
         i < loss_history_.total_losses.size(); ++i) {
        recent_avg += loss_history_.total_losses[i];
    }
    recent_avg /= recent_count;

    history_text += "Recent avg loss: " + std::to_string(recent_avg);

    logText("training/loss_curves/summary", history_text);
}

void RerunTrainingVisualizer::visualizeKeyframeBatch(const training::KeyframeBatch& batch,
                                                     const std::string& stage) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_ || !visualize_batches_) {
        LOG(INFO) << "Viz not initialized?";
        return;
    }

    std::string base_path =
        "training/batches/" + stage + "/batch_" + std::to_string(batch.batch_id);

    // Visualize batch metadata
    std::string batch_info = "Batch " + std::to_string(batch.batch_id) + " (" + stage +
                             "): " + std::to_string(batch.batch_size) + " keyframes, " +
                             std::to_string(batch.image_width) + "x" +
                             std::to_string(batch.image_height);

    logText(base_path + "/info", batch_info);

    LOG(INFO) << "Visualizing batch " << batch.batch_id << " at " << base_path << " ("
              << batch.batch_size << " keyframes, " << batch.image_width << "x"
              << batch.image_height << ") " << batch.camera_poses.size(0) << " camera poses";
    // Visualize camera poses if available
    if (batch.camera_poses.defined() && batch.camera_poses.size(0) > 0) {
        visualizeBatchCameraPoses(batch, base_path);
    }

    // Visualize sample images from batch
    if (batch.images.defined() && batch.images.size(0) > 0) {
        visualizeBatchImages(batch, base_path);
    }

    LOG(INFO) << "Visualized keyframe batch " << batch.batch_id << " for stage: " << stage;
}
void RerunTrainingVisualizer::visualizeKeyframe(const core::types::Pose& pose,
                                                const core::types::CameraInfo& cam_info,
                                                const std::string& entity) {
    LOG(INFO) << "Logging camera pose: " << pose.position.transpose() << " at " << entity;
    try {
        auto now = std::chrono::system_clock::now();
        double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
        rerun_viz_->addPose(pose, entity, 0.);
        rerun_viz_->addCamera(cam_info, entity, 0.);
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to log camera pose: " << e.what();
    }
}

void RerunTrainingVisualizer::visualizeCurrentSplats(
    const core::types::GaussianSplatBatch& splat_batch, uint32_t iteration) {
    LOG(INFO) << "trying to viz current splats";
    std::lock_guard<std::mutex> lock(viz_mutex_);
    LOG(INFO) << "Vizing current splats: " << splat_batch.splats.size();

    if (!initialized_ || !visualize_splats_ || splat_batch.splats.empty()) {
        return;
    }

    // Only visualize every N iterations to avoid overwhelming the viewer
    if (iteration % log_frequency_ != 0) {
        return;
    }

    // std::string entity_path = "training/splats/current";
    std::string entity_path = "/splats";
    try {
        // Convert splats to point cloud for visualization
        logPoints3D(entity_path, splat_batch.splats, iteration);

        // Log splat statistics
        std::string splat_info = "Iteration " + std::to_string(iteration) + ": " +
                                 std::to_string(splat_batch.splats.size()) + " splats";

        logText("training/splats/info", splat_info);

        LOG(INFO) << "Visualized " << splat_batch.splats.size() << " splats at iteration "
                  << iteration;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to visualize splats: " << e.what();
    }
}
void RerunTrainingVisualizer::visualizeCurrentSplats(
    const std::vector<core::types::GaussianSplat>& splats, uint32_t iteration) {
    LOG(INFO) << "trying to viz current splats";
    std::lock_guard<std::mutex> lock(viz_mutex_);
    LOG(INFO) << "Vizing current splats: " << splats.size();

    if (!initialized_ || !visualize_splats_ || splats.empty()) {
        return;
    }

    // Only visualize every N iterations to avoid overwhelming the viewer
    if (iteration % log_frequency_ != 0) {
        return;
    }

    std::string entity_path = "/splats";
    for (const auto& splat : splats) {
        std::cout << "Viz Splat: " << splat.id << " with position: " << splat.position.transpose()
                  << std::endl;
    }

    try {
        // Convert splats to point cloud for visualization
        logPoints3D(entity_path, splats, iteration);

        // Log splat statistics
        std::string splat_info = "Iteration " + std::to_string(iteration) + ": " +
                                 std::to_string(splats.size()) + " splats";

        // logText("training/splats/info", splat_info);

        LOG(INFO) << "Visualized " << splats.size() << " splats at iteration " << iteration;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to visualize splats: " << e.what();
    }
}

void RerunTrainingVisualizer::visualizeBBox(const utils::BoundingBox& bbox, std::string name) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_) {
        return;
    }
    if (!rerun_viz_) {
        return;
    }

    auto center = bbox.center();
    Eigen::Vector3f half_size = bbox.size() / 2.0f;
    try {
        // Use RerunVisualizer's Gaussian splat method to render as proper ellipsoids with colors
        auto now = std::chrono::system_clock::now();
        double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
        rerun_viz_->addBoundingBox(center, half_size, name);

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to log bounding box " << name << ": " << e.what();
    }
}

void RerunTrainingVisualizer::visualizeTrainingState(const std::string& state,
                                                     const std::string& details) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_) {
        return;
    }

    std::string state_text = "State: " + state;
    if (!details.empty()) {
        state_text += "\nDetails: " + details;
    }

    logText("training/state", state_text);

    LOG(INFO) << "Training state: " << state << " - " << details;
}

void RerunTrainingVisualizer::visualizeLearningRate(double learning_rate, uint32_t iteration) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_) {
        return;
    }

    logScalar("training/optimization/learning_rate", learning_rate, iteration);
}

void RerunTrainingVisualizer::visualizeGradientNorms(const std::vector<double>& grad_norms,
                                                     uint32_t iteration) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_ || grad_norms.empty()) {
        return;
    }

    // Log gradient statistics
    double mean_grad =
        std::accumulate(grad_norms.begin(), grad_norms.end(), 0.0) / grad_norms.size();
    double max_grad = *std::max_element(grad_norms.begin(), grad_norms.end());

    logScalar("training/optimization/grad_norm_mean", mean_grad, iteration);
    logScalar("training/optimization/grad_norm_max", max_grad, iteration);
}

void RerunTrainingVisualizer::setCurrentTimestamp(double timestamp) {
    current_timestamp_ = timestamp;
}

// Private helper methods

void RerunTrainingVisualizer::logScalar(const std::string& entity_path, double value,
                                        uint32_t step) {
    if (!rerun_viz_) {
        return;
    }

    try {
        // Use Rerun's scalar logging - this would need the actual Rerun API
        // For now, we'll use text logging as a placeholder
        std::string scalar_text = std::to_string(value) + " (step " + std::to_string(step) + ")";
        logText(entity_path, scalar_text);

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to log scalar " << entity_path << ": " << e.what();
    }
}

void RerunTrainingVisualizer::logText(const std::string& entity_path, const std::string& text) {
    if (!rerun_viz_) {
        return;
    }

    try {
        // Use Rerun's text logging capabilities
        // This is a placeholder - actual implementation would use rerun_viz_ methods
        LOG(INFO) << "[" << entity_path << "] " << text;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to log text " << entity_path << ": " << e.what();
    }
}

void RerunTrainingVisualizer::logImage(const std::string& entity_path, const cv::Mat& image,
                                       uint32_t step) {
    LOG(INFO) << "Logging image " << entity_path << " at step " << step
              << " image size: " << image.empty();
    if (!rerun_viz_ || image.empty()) {
        return;
    }
    rerun_viz_->addImage(image, entity_path, 0.);

    // try {
    //     // Convert cv::Mat to format suitable for Rerun
    //     cv::Mat display_image;
    //     LOG(INFO) << "Image channels: " << image.channels();
    //     if (image.channels() == 1) {
    //         cv::cvtColor(image, display_image, cv::COLOR_GRAY2RGB);
    //     } else {
    //         display_image = image.clone();
    //     }
    //
    //     // Use RerunVisualizer's addImage method
    //     auto now = std::chrono::system_clock::now();
    //     double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
    //     LOG(INFO) << "Display image: " << display_image.rows << "x" << display_image.cols;
    //     rerun_viz_->addImage(display_image, entity_path, 0.);
    //
    // } catch (const std::exception& e) {
    //     LOG(ERROR) << "Failed to log image " << entity_path << ": " << e.what();
    // }
}

void RerunTrainingVisualizer::logPoints3D(const std::string& entity_path,
                                          const std::vector<core::types::GaussianSplat>& splats,
                                          uint32_t iteration) {
    if (!rerun_viz_ || splats.empty()) {
        return;
    }

    try {
        // Use RerunVisualizer's Gaussian splat method to render as proper ellipsoids with colors
        auto now = std::chrono::system_clock::now();
        double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
        rerun_viz_->addGaussianSplats(splats, entity_path, 0.);

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to log Gaussian splats " << entity_path << ": " << e.what();
    }
}

void RerunTrainingVisualizer::visualizeBatchImages(const training::KeyframeBatch& batch,
                                                   const std::string& base_path) {
    // Visualize a subset of images from the batch to avoid overwhelming the viewer
    size_t max_images = std::min(static_cast<size_t>(4), batch.batch_size);

    for (size_t i = 0; i < max_images; ++i) {
        if (i < batch.keyframes.size() && batch.keyframes[i] &&
            batch.keyframes[i]->hasColorImage()) {
            const auto& image = batch.keyframes[i]->getColorImage();
            cv::Mat cv_image = image.toCvMat();

            if (!cv_image.empty()) {
                std::string image_path = base_path + "/images/keyframe_" + std::to_string(i);
                logImage(image_path, cv_image, batch.batch_id);
            }
        }
    }
}

void RerunTrainingVisualizer::visualizeBatchCameraPoses(const training::KeyframeBatch& batch,
                                                        const std::string& base_path) {
    // Extract camera poses and visualize them
    LOG(INFO) << "Visualizing camera poses " << batch.keyframes.size()
              << " camera poses: " << batch.camera_poses.size(0);
    for (size_t i = 0; i < batch.keyframes.size() && i < 10; ++i) {  // Limit to first 10 poses
        if (batch.keyframes[i]) {
            const auto& pose = batch.keyframes[i]->pose;
            LOG(INFO) << "Visualizing camera pose " << batch.keyframes[i]->id
                      << ":: Pose: " << pose.position.transpose()
                      << " with timestamp: " << pose.timestamp
                      << " inverse: " << pose.inverse().position.transpose();
            // std::string pose_path = base_path + "/camera_" + std::to_string(i);
            std::string pose_path = "/camera_" + std::to_string(i);

            try {
                rerun_viz_->addPose(pose.inverse(), pose_path, 0.);
                rerun_viz_->addCamera(batch.keyframes[i]->getCameraInfo(), pose_path, 0.);
                LOG(INFO) << "Successfully visualized camera pose " << i << " at " << pose_path;
            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to visualize camera pose " << i << ": " << e.what();
            }
        } else {
            LOG(WARNING) << "Failed to visualize camera pose " << i << ": Keyframe not available";
        }
    }
}

void RerunTrainingVisualizer::createLossCurveTimeSeries() {
    // Initialize loss curve visualization layout
    logText("training/loss_curves/layout",
            "Loss curves initialized for Gaussian Splatting training");
    logText("training/metrics/layout", "Training metrics initialized");
    logText("training/epochs/layout", "Epoch tracking initialized");
}

void RerunTrainingVisualizer::logLoss(const std::string& entity_path, double value, int iteration) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_ || !rerun_viz_) {
        return;
    }

    try {
        rerun_viz_->logLoss(entity_path, value, iteration);
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to log loss to " << entity_path << ": " << e.what();
    }
}

void RerunTrainingVisualizer::logLoss(const std::string& entity_path, double value, int iteration,
                                      uint8_t r, uint8_t g, uint8_t b) {
    std::lock_guard<std::mutex> lock(viz_mutex_);

    if (!initialized_ || !rerun_viz_) {
        return;
    }

    try {
        rerun_viz_->logLoss(entity_path, value, iteration, r, g, b);
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to log loss to " << entity_path << ": " << e.what();
    }
}

}  // namespace visualization
}  // namespace gaussian_splatting
