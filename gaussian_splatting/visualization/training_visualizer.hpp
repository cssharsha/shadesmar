#pragma once

#include <chrono>
#include <core/types/image.hpp>
#include <core/types/pose.hpp>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "core/types/gaussian_splat.hpp"
#include "gaussian_splatting/training/keyframe_batch.hpp"
#include "viz/rerun_viz.hpp"

namespace gaussian_splatting {
namespace visualization {

struct TrainingMetrics {
    uint32_t epoch;
    uint32_t batch_in_epoch;
    uint32_t total_batches_processed;
    double total_loss;
    double l1_loss;
    double ssim_loss;
    double learning_rate;
    double batch_processing_time_ms;
    std::chrono::steady_clock::time_point timestamp;
    size_t num_splats;
    size_t num_keyframes_in_batch;
};

struct LossHistory {
    std::deque<double> total_losses;
    std::deque<double> l1_losses;
    std::deque<double> ssim_losses;
    std::deque<uint32_t> batch_indices;
    std::deque<std::chrono::steady_clock::time_point> timestamps;

    static constexpr size_t MAX_HISTORY_SIZE = 1000;

    void addLoss(double total_loss, double l1_loss, double ssim_loss, uint32_t batch_idx) {
        total_losses.push_back(total_loss);
        l1_losses.push_back(l1_loss);
        ssim_losses.push_back(ssim_loss);
        batch_indices.push_back(batch_idx);
        timestamps.push_back(std::chrono::steady_clock::now());

        // Maintain maximum history size
        if (total_losses.size() > MAX_HISTORY_SIZE) {
            total_losses.pop_front();
            l1_losses.pop_front();
            ssim_losses.pop_front();
            batch_indices.pop_front();
            timestamps.pop_front();
        }
    }

    void clear() {
        total_losses.clear();
        l1_losses.clear();
        ssim_losses.clear();
        batch_indices.clear();
        timestamps.clear();
    }
};

class RerunTrainingVisualizer {
public:
    explicit RerunTrainingVisualizer(const std::string& recording_id = "gaussian_splat_training",
                                     const std::string& host = "localhost", uint16_t port = 9999);

    ~RerunTrainingVisualizer();

    // Initialization
    bool initialize();
    void shutdown();
    bool isInitialized() const {
        return initialized_;
    }

    // Training progress tracking
    void logTrainingMetrics(const TrainingMetrics& metrics);
    void logEpochStart(uint32_t epoch, size_t total_batches);
    void logEpochEnd(uint32_t epoch, double avg_loss, double epoch_time_s);

    // Loss visualization
    void updateLossCurves(double total_loss, double l1_loss, double ssim_loss, uint32_t batch_idx);
    void visualizeLossHistory();

    // Batch and keyframe visualization
    void visualizeKeyframeBatch(const training::KeyframeBatch& batch,
                                const std::string& stage = "training");
    void visualizeKeyframe(const core::types::Pose& pose, const core::types::CameraInfo& cam_info,
                           const std::string& entity);
    void visualizeCurrentSplats(const core::types::GaussianSplatBatch& splat_batch,
                                uint32_t iteration);
    void visualizeCurrentSplats(const std::vector<core::types::GaussianSplat>& splats,
                                uint32_t iteration);

    // Training state visualization
    void visualizeTrainingState(const std::string& state, const std::string& details = "");
    void visualizeLearningRate(double learning_rate, uint32_t iteration);
    void visualizeGradientNorms(const std::vector<double>& grad_norms, uint32_t iteration);

    // Configuration
    void setVisualizationFrequency(uint32_t log_every_n_batches) {
        log_frequency_ = log_every_n_batches;
    }
    void enableBatchVisualization(bool enabled) {
        visualize_batches_ = enabled;
    }
    void enableSplatVisualization(bool enabled) {
        visualize_splats_ = enabled;
    }

    // Real-time updates
    void setCurrentTimestamp(double timestamp);
    void logImage(const std::string& entity_path, const cv::Mat& image, uint32_t step);

private:
    // Rerun visualization helpers
    void logScalar(const std::string& entity_path, double value, uint32_t step);
    void logText(const std::string& entity_path, const std::string& text);
    void logPoints3D(const std::string& entity_path,
                     const std::vector<core::types::GaussianSplat>& splats, uint32_t iteration = 0);

    // Batch visualization helpers
    void visualizeBatchImages(const training::KeyframeBatch& batch, const std::string& base_path);
    void visualizeBatchCameraPoses(const training::KeyframeBatch& batch,
                                   const std::string& base_path);

    // Loss curve helpers
    void createLossCurveTimeSeries();

    std::shared_ptr<viz::RerunVisualizer> rerun_viz_;
    std::string recording_id_;
    std::string host_;
    uint16_t port_;

    bool initialized_;
    std::mutex viz_mutex_;

    // Training state
    LossHistory loss_history_;
    uint32_t current_epoch_;
    uint32_t current_batch_;
    uint32_t total_batches_processed_;

    // Visualization settings
    uint32_t log_frequency_;
    bool visualize_batches_;
    bool visualize_splats_;

    // Timing
    std::chrono::steady_clock::time_point training_start_time_;
    double current_timestamp_;
};

}  // namespace visualization
}  // namespace gaussian_splatting
