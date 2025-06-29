#pragma once

#include <torch/torch.h>
#include <Eigen/Dense>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "core/storage/map_store.hpp"
#include "core/storage/shared_memory_wrapper.hpp"
#include "core/types/gaussian_splat.hpp"
#include "core/types/keyframe.hpp"
#include "core/types/keypoint.hpp"
#include "stf/transform_tree.hpp"

#include "optimization/densification_controller.hpp"
#include "rendering/bilateral_grid.hpp"
#include "rendering/rasterizer.hpp"
#include "rendering/spherical_harmonics.hpp"
#include "training/batch_trainer.hpp"
#include "training/keyframe_batch.hpp"
#include "training/training_config.hpp"
#include "visualization/training_visualizer.hpp"

namespace gaussian_splatting {

void initializeLogging();

// Splat initialization modes
enum class SplatInitMode {
    SFM_KEYPOINTS,  // Initialize from existing SfM keypoints
    RANDOM_POINTS   // Initialize random points within scene bounds
};

// Training progress callback (non-blocking)
using TrainingProgressCallback = std::function<void(int iteration, float loss, int splat_count)>;

// Template configuration for different backends
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
struct TrainingProcessorConfig {
    // Basic configuration
    std::string map_base_path;
    std::string status_file_path;

    // Training parameters
    SplatInitMode init_mode = SplatInitMode::SFM_KEYPOINTS;
    int keyframes_per_batch = 10;
    int max_training_iterations = 1000;
    int visualization_frequency = 10;

    // Visualization settings - use shared recording ID
    bool enable_training_visualization = true;
    std::string training_viz_recording_id = "shadesmar_combined";  // Shared with main viz
    std::string training_viz_application_id = "gs_training";
    std::string training_viz_host = "localhost";
    uint16_t training_viz_port = 9999;

    // Scene bounds for random initialization
    Eigen::Vector3f scene_min = Eigen::Vector3f(-10.0f, -10.0f, -10.0f);
    Eigen::Vector3f scene_max = Eigen::Vector3f(10.0f, 10.0f, 10.0f);
    int initial_random_splats = 10000;

    // Device configuration
    torch::Device device = torch::kCPU;
    bool enable_cuda = true;

    // Training schedule
    bool train_batch_to_convergence = true;
    float convergence_threshold = 1e-4f;

    // Thread control
    int keyframe_batch_threshold = 10;  // Minimum keyframes needed to start training

    TrainingProcessorConfig() = default;
    TrainingProcessorConfig(const std::string& map_path) : map_base_path(map_path) {
        status_file_path = map_path + "_training_gs_status";

        // Auto-detect CUDA
        if (enable_cuda && torch::cuda::is_available()) {
            device = torch::kCUDA;
        }
    }
};

// Training statistics
struct TrainingStats {
    std::atomic<int> total_iterations{0};
    std::atomic<int> current_batch_id{0};
    std::atomic<float> current_loss{0.0f};
    std::atomic<int> current_splat_count{0};
    std::atomic<bool> is_training{false};
    std::atomic<uint64_t> last_processed_keyframe_id{0};
    std::atomic<bool> splats_initialized{false};
    std::atomic<int> initialized_splat_count{0};

    void reset() {
        total_iterations = 0;
        current_batch_id = 0;
        current_loss = 0.0f;
        current_splat_count = 0;
        is_training = false;
        last_processed_keyframe_id = 0;
        splats_initialized = false;
        initialized_splat_count = 0;
    }
};

// Thread communication structures
struct ThreadNotification {
    std::atomic<bool> splats_ready{false};
    std::atomic<bool> training_complete{false};
    std::atomic<bool> should_stop{false};
    std::atomic<uint64_t> latest_splat_batch_id{0};
    std::condition_variable splats_ready_cv;
    std::condition_variable training_complete_cv;
    std::mutex notification_mutex;
};

template <typename BilateralGridT = rendering::BilateralGrid,
          typename DensityControllerT = optimization::DensificationController,
          typename TrainingConfigT = training::TrainingConfig,
          typename RasterizationT = rendering::DifferentiableRasterizer>
class TrainingGaussianSplatProcessor {
public:
    using ConfigType = TrainingProcessorConfig<BilateralGridT, DensityControllerT, TrainingConfigT,
                                               RasterizationT>;

    explicit TrainingGaussianSplatProcessor(const ConfigType& config);
    ~TrainingGaussianSplatProcessor();

    // Main processing control
    bool initialize();
    bool start();
    void stop();
    bool isRunning() const {
        return main_thread_running_.load();
    }

    // Configuration access
    void setConfig(const ConfigType& config) {
        config_ = config;
    }
    const ConfigType& getConfig() const {
        return config_;
    }

    // Statistics
    const TrainingStats& getStats() const {
        return stats_;
    }
    void resetStats() {
        stats_.reset();
    }

    // Callbacks
    void setTrainingProgressCallback(TrainingProgressCallback callback);
    void clearTrainingProgressCallback();

private:
    ConfigType config_;
    TrainingStats stats_;
    TrainingConfigT training_config_;

    // Template-parameterized components (note: some are now handled by BatchTrainer)
    std::unique_ptr<BilateralGridT> bilateral_grid_;
    std::unique_ptr<DensityControllerT> density_controller_;
    std::unique_ptr<training::BatchTrainer> batch_trainer_;  // This handles the actual training

    // Thread management
    std::unique_ptr<std::thread> main_thread_;           // Map sync + splat generation
    std::unique_ptr<std::thread> training_thread_;       // Training execution
    std::unique_ptr<std::thread> visualization_thread_;  // Visualization

    std::atomic<bool> main_thread_running_{false};
    std::atomic<bool> training_thread_running_{false};
    std::atomic<bool> visualization_thread_running_{false};

    ThreadNotification thread_notification_;

    // Storage and data access
    std::shared_ptr<core::storage::MapStore> map_store_;
    std::shared_ptr<stf::TransformTree> transform_tree_;
    std::unique_ptr<core::storage::SharedMemoryWrapper> shared_memory_;

    uint32_t last_synced_keyframes_id_{0};
    // Training state
    uint32_t current_batch_id_{0};
    uint32_t next_splat_id_{1};
    uint64_t last_processed_keyframe_id_{0};
    
    // Batch queue for training
    std::queue<uint32_t> training_batch_queue_;
    mutable std::mutex batch_queue_mutex_;
    
    // Visualization queues (never remove elements, only append)
    std::queue<uint32_t> initialized_batches_queue_;
    std::queue<uint32_t> trained_batches_queue_;
    mutable std::mutex visualization_queue_mutex_;
    size_t last_initialized_queue_size_{0};
    size_t last_trained_queue_size_{0};

    // Callbacks
    TrainingProgressCallback training_progress_callback_;

    // Training visualization
    std::unique_ptr<visualization::RerunTrainingVisualizer> training_visualizer_;

    // Thread main functions
    void mainThreadLoop();           // Map sync + splat generation
    void trainingThreadLoop();       // Training execution
    void visualizationThreadLoop();  // Visualization

    // Main thread methods (Map sync + splat generation)
    bool checkForNewKeyframes();
    bool syncMapStore();
    bool generateSplatsFromKeypoints();
    std::vector<core::types::Keypoint> getVisibleKeypoints(
        const std::vector<uint64_t>& keyframe_ids);
    bool initializeFromSfMKeypoints(const std::vector<core::types::Keypoint>& keypoints,
                                    const std::vector<uint64_t>& keyframe_ids);
    bool writeSplatBatchToDisk(const core::types::GaussianSplatBatch& batch);

    // Training thread methods
    bool loadSplatBatchFromDisk(uint64_t batch_id, core::types::GaussianSplatBatch& batch);
    bool loadKeyframeBatchFromDisk(const std::vector<uint64_t>& keyframe_ids,
                                   training::KeyframeBatch& batch);
    bool executeTrainingOnBatch(const core::types::GaussianSplatBatch& splat_batch,
                                const training::KeyframeBatch& keyframe_batch);
    bool writeTrainedSplatsToDisk(const core::types::GaussianSplatBatch& batch);

    // Visualization thread methods
    bool initializeTrainingVisualization();
    void shutdownTrainingVisualization();
    bool visualizeInitializedSplats();
    bool visualizeTrainedSplats();
    bool visualizeInitializedSplatBatch(uint32_t batch_id);
    bool visualizeTrainedSplatBatch(uint32_t batch_id);
    void handleTrainingStatsCallback(int iteration, float total_loss, float l1_loss,
                                     float d_ssim_loss, int splat_count);

    // Helper methods for SfM keypoint initialization
    Eigen::Vector3f extractColorFromKeyframes(
        const core::types::Keypoint& keypoint,
        const std::unordered_map<uint64_t, core::types::KeyFrame::Ptr>& keyframe_map);
    Eigen::Matrix3d computeKeypointCovariance(
        const core::types::Keypoint& keypoint,
        const std::unordered_map<uint64_t, core::types::KeyFrame::Ptr>& keyframe_map);
    float computeInitialOpacity(const core::types::Keypoint& keypoint);
    float computeInitialConfidence(const core::types::Keypoint& keypoint);

    // Scene bounds estimation
    std::pair<Eigen::Vector3f, Eigen::Vector3f> estimateSceneBounds();

    // Image and camera processing
    bool extractImageTensor(const core::types::KeyFrame::Ptr& keyframe, torch::Tensor& image_tensor,
                            core::types::CameraInfo& camera_info);
    bool extractCameraPoses(const std::vector<uint64_t>& keyframe_ids,
                            std::vector<Eigen::Isometry3d>& camera_poses,
                            const std::string& target_frame = "world");
    torch::Tensor convertCameraIntrinsicsToTensor(const core::types::CameraInfo& camera_info);
    torch::Tensor convertCameraPoseToTensor(const Eigen::Isometry3d& pose);

    // Note: Rendering and training logic is handled by the existing BatchTrainer class

    // Batch queue management methods
    void enqueueBatchForTraining(uint32_t batch_id);
    bool dequeueBatchForTraining(uint32_t& batch_id);
    bool hasPendingBatches() const;
    size_t getPendingBatchCount() const;

    // Visualization queue management methods
    void enqueueInitializedBatch(uint32_t batch_id);
    void enqueueTrainedBatch(uint32_t batch_id);
    bool hasNewInitializedBatches();
    bool hasNewTrainedBatches();
    std::vector<uint32_t> getAllInitializedBatches() const;
    std::vector<uint32_t> getAllTrainedBatches() const;

    // Utility methods
    bool waitForMapData();
    double getCurrentTimestamp() const;
    void logTrainingStats() const;
    void notifyThreads();
};

// Convenience type aliases for common configurations
using StandardTrainingProcessor =
    TrainingGaussianSplatProcessor<rendering::BilateralGrid, optimization::DensificationController,
                                   training::TrainingConfig, rendering::DifferentiableRasterizer>;

using CPUTrainingProcessor =
    TrainingGaussianSplatProcessor<rendering::BilateralGrid, optimization::DensificationController,
                                   training::TrainingConfig, rendering::DifferentiableRasterizer>;

}  // namespace gaussian_splatting
