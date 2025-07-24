#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

#include <torch/torch.h>
#include <Eigen/Dense>

#include "core/storage/map_store.hpp"
#include "core/storage/shared_memory_wrapper.hpp"
#include "core/types/gaussian_splat.hpp"
#include "core/types/keyframe.hpp"
#include "core/types/keypoint.hpp"
#include "gaussian_splatting/optimization/densification_controller.hpp"
#include "gaussian_splatting/rendering/bilateral_grid.hpp"
#include "gaussian_splatting/training/batch_trainer.hpp"
#include "gaussian_splatting/visualization/training_visualizer.hpp"
#include "logging/logging.hpp"
#include "utils/stf/transform_tree.hpp"

namespace gaussian_splatting {

// Forward declarations for template parameters
namespace rendering {
class BilateralGrid;
class DifferentiableRasterizer;
}  // namespace rendering

namespace optimization {
class DensificationController;
}  // namespace optimization

namespace training {
struct TrainingConfig;
struct KeyframeBatch;
struct TrainingResults;
}  // namespace training

// Configuration for sliding window behavior
struct SlidingWindowConfig {
    size_t max_window_size = 10;              // Maximum keyframes in sliding window
    size_t min_window_size = 3;               // Minimum keyframes needed to start training
    double keyframe_overlap_threshold = 0.7;  // Overlap threshold for keyframe selection
    bool enable_window_optimization = true;   // Enable optimization across entire window
};

// Configuration for convergence detection
struct ConvergenceConfig {
    double covariance_threshold = 0.001;       // Splat covariance convergence threshold
    int max_iterations_per_splat = 1000;       // Maximum training iterations per splat
    double loss_convergence_threshold = 1e-6;  // Loss convergence threshold
    int convergence_check_interval = 10;       // Check convergence every N iterations
    double min_opacity_threshold = 0.05;       // Minimum opacity for active splats
    bool enable_early_stopping = true;         // Enable early stopping based on convergence
};

// Configuration for random splat initialization
struct SplatInitializationConfig {
    int initial_splat_count = 10000;         // Initial number of splats to generate
    double scene_bounds_padding = 2.0;       // Padding around camera trajectory bounds
    double initial_covariance_scale = 0.1;   // Initial covariance scaling factor
    double initial_opacity_range_min = 0.1;  // Minimum initial opacity
    double initial_opacity_range_max = 0.9;  // Maximum initial opacity
    bool adaptive_density = true;            // Enable adaptive splat density
    double density_scale_factor = 1.0;       // Global density scaling factor
};

// Sliding window data structure for managing keyframes
struct SlidingWindow {
    std::deque<core::types::KeyFrame::Ptr> keyframes;
    std::deque<uint64_t> keyframe_ids;
    size_t max_size;
    mutable std::mutex window_mutex;

    explicit SlidingWindow(size_t max_window_size) : max_size(max_window_size) {}

    void addKeyframe(core::types::KeyFrame::Ptr keyframe);
    void removeOldest();
    bool isFull() const;
    bool isEmpty() const;
    size_t size() const;
    std::vector<core::types::KeyFrame::Ptr> getKeyframes() const;
    std::vector<uint64_t> getKeyframeIds() const;
    void clear();
};

// Thread notification structure for inter-thread communication
struct ThreadNotification {
    std::atomic<bool> should_stop{false};
    std::atomic<bool> new_keyframe_available{false};
    std::atomic<bool> training_complete{false};
    std::atomic<bool> splats_converged{false};

    std::mutex notification_mutex;
    std::condition_variable new_keyframe_cv;
    std::condition_variable training_complete_cv;
    std::condition_variable convergence_cv;

    std::atomic<uint64_t> latest_keyframe_id{0};
    std::atomic<uint32_t> current_training_iteration{0};
    std::atomic<double> current_loss{std::numeric_limits<double>::max()};
};

// Training statistics for monitoring and visualization
struct TrainingStatistics {
    std::atomic<uint64_t> total_keyframes_processed{0};
    std::atomic<uint64_t> total_training_iterations{0};
    std::atomic<double> current_loss{0.0};
    std::atomic<double> best_loss{std::numeric_limits<double>::max()};
    std::atomic<int> current_splat_count{0};
    std::atomic<int> converged_splats{0};
    std::atomic<bool> is_training{false};
    std::atomic<bool> is_converged{false};
    std::chrono::steady_clock::time_point start_time;
    std::chrono::steady_clock::time_point last_update_time;

    void reset();
    double getElapsedTimeSeconds() const;
    double getTrainingRate() const;  // Iterations per second
};

// Main configuration structure
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
struct StreamingGSProcessorConfig {
    // Device configuration
    torch::Device device = torch::kCPU;

    // File paths
    std::string map_base_path;
    std::string training_viz_recording_id = "streaming_gs_training";
    std::string training_viz_host = "127.0.0.1";
    int training_viz_port = 9876;

    // Sliding window configuration
    SlidingWindowConfig window_config;

    // Convergence configuration
    ConvergenceConfig convergence_config;

    // Splat initialization configuration
    SplatInitializationConfig init_config;

    // Training configuration
    int max_training_iterations = 1000;
    double learning_rate = 0.01;
    int training_frequency = 1;  // Train every N keyframes

    // Scene bounds (fallback if trajectory estimation fails)
    Eigen::Vector3f scene_min{-10.0f, -10.0f, -10.0f};
    Eigen::Vector3f scene_max{10.0f, 10.0f, 10.0f};

    // Polling intervals
    int main_loop_interval_ms = 500;
    int training_loop_interval_ms = 100;
    int visualization_loop_interval_ms = 1000;
};

// Forward declaration for callback types
using TrainingProgressCallback = std::function<void(int, float, int)>;

// Main StreamingGaussianSplatProcessor class
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
class StreamingGaussianSplatProcessor {
public:
    using ConfigType = StreamingGSProcessorConfig<BilateralGridT, DensityControllerT,
                                                  TrainingConfigT, RasterizationT>;

    explicit StreamingGaussianSplatProcessor(const ConfigType& config);
    ~StreamingGaussianSplatProcessor();

    // Core lifecycle methods
    bool initialize();
    bool start();
    void stop();

    // Configuration and callback methods
    void setTrainingProgressCallback(TrainingProgressCallback callback);
    void clearTrainingProgressCallback();

    // Status and statistics methods
    bool isRunning() const {
        return main_thread_running_.load();
    }
    bool isTraining() const {
        return stats_.is_training.load();
    }
    bool isConverged() const {
        return stats_.is_converged.load();
    }
    const TrainingStatistics& getStatistics() const {
        return stats_;
    }

private:
    // Configuration
    ConfigType config_;
    TrainingConfigT training_config_;

    // Core components
    std::shared_ptr<core::storage::MapStore> map_store_;
    std::shared_ptr<stf::TransformTree> transform_tree_;
    std::unique_ptr<core::storage::SharedMemoryWrapper> shared_memory_;

    // Template-parameterized components
    std::unique_ptr<BilateralGridT> bilateral_grid_;
    std::unique_ptr<DensityControllerT> density_controller_;
    std::unique_ptr<training::BatchTrainer> batch_trainer_;
    std::unique_ptr<visualization::RerunTrainingVisualizer> training_visualizer_;

    // Sliding window management
    std::unique_ptr<SlidingWindow> sliding_window_;

    // Thread management
    std::unique_ptr<std::thread> main_thread_;
    std::unique_ptr<std::thread> training_thread_;
    std::unique_ptr<std::thread> visualization_thread_;

    std::atomic<bool> main_thread_running_{false};
    std::atomic<bool> training_thread_running_{false};
    std::atomic<bool> visualization_thread_running_{false};

    // Thread communication
    ThreadNotification thread_notification_;

    // Statistics and monitoring
    TrainingStatistics stats_;

    // Splat management
    std::atomic<uint64_t> next_splat_id_{0};
    std::atomic<uint64_t> batch_id_{0};
    std::atomic<uint64_t> last_processed_keyframe_id_{0};

    // Callback
    TrainingProgressCallback training_progress_callback_;

    // Random number generation
    std::mt19937 random_generator_;
    std::uniform_real_distribution<double> uniform_dist_;

    // Main thread methods
    void mainThreadLoop();
    bool checkForNewKeyframes();
    bool syncMapStoreAndTransformTree();
    bool updateSlidingWindow();
    bool handleNewKeyframe(core::types::KeyFrame::Ptr keyframe);

    // Training thread methods
    void trainingThreadLoop();
    bool trainOnCurrentWindow();
    bool executeIncrementalTraining(const training::KeyframeBatch& keyframe_batch);
    bool checkConvergence();

    // Visualization thread methods
    void visualizationThreadLoop();
    bool initializeVisualization();
    void shutdownVisualization();
    bool visualizeCurrentSplats();
    bool visualizeSlidingWindow();
    void handleTrainingStatsCallback(int iteration, float total_loss, float l1_loss,
                                     float d_ssim_loss, int splat_count);

    // Utility methods
    bool waitForMapData();
    double getCurrentTimestamp() const;
    void logTrainingStats() const;
    void notifyThreads();
};

// Type aliases for common configurations
using DefaultStreamingGSProcessor =
    StreamingGaussianSplatProcessor<rendering::BilateralGrid, optimization::DensificationController,
                                    training::TrainingConfig, rendering::DifferentiableRasterizer>;

}  // namespace gaussian_splatting
