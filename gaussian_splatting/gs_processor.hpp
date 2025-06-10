#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "core/storage/map_store.hpp"
#include "core/types/gaussian_splat.hpp"
#include "core/types/keyframe.hpp"
#include "core/types/keypoint.hpp"
#include "stf/transform_tree.hpp"

namespace gaussian_splatting {

struct ProcessorConfig {
    std::string map_base_path;                    // Base path for map data files
    std::string status_file_path;                 // Path for process status files
    double polling_interval_ms = 100.0;          // How often to check for new data
    size_t batch_size = 10;                      // Number of keyframes per splat batch
    size_t max_splat_batches_in_memory = 50;     // Memory management limit
    bool enable_gpu_acceleration = true;         // Use GPU for splat generation
    double min_keypoint_confidence = 0.5;        // Minimum confidence for splat generation
    
    ProcessorConfig() = default;
    ProcessorConfig(const std::string& map_path) : map_base_path(map_path) {
        status_file_path = map_path + "_gs_status";
    }
};

struct ProcessorStats {
    std::atomic<uint64_t> total_keyframes_processed{0};
    std::atomic<uint32_t> total_splat_batches_generated{0};
    std::atomic<uint64_t> total_splats_generated{0};
    std::atomic<uint64_t> last_processed_keyframe_id{0};
    std::atomic<uint32_t> last_generated_batch_id{0};
    std::atomic<bool> is_processing{false};
    std::atomic<double> last_processing_time_ms{0.0};
    
    void reset() {
        total_keyframes_processed = 0;
        total_splat_batches_generated = 0;
        total_splats_generated = 0;
        last_processed_keyframe_id = 0;
        last_generated_batch_id = 0;
        is_processing = false;
        last_processing_time_ms = 0.0;
    }
};

class GaussianSplatProcessor {
public:
    explicit GaussianSplatProcessor(const ProcessorConfig& config);
    ~GaussianSplatProcessor();

    // Main processing control
    bool initialize();
    bool start();
    void stop();
    bool isRunning() const { return processing_thread_running_.load(); }

    // Configuration
    void setConfig(const ProcessorConfig& config) { config_ = config; }
    const ProcessorConfig& getConfig() const { return config_; }

    // Statistics
    const ProcessorStats& getStats() const { return stats_; }
    void resetStats() { stats_.reset(); }

    // Status file management
    bool readVSLAMStatus(core::proto::ProcessStatus& status) const;
    bool writeGSStatus(const core::proto::ProcessStatus& status) const;

private:
    ProcessorConfig config_;
    ProcessorStats stats_;

    // Threading infrastructure
    std::unique_ptr<std::thread> processing_thread_;
    std::atomic<bool> processing_thread_running_{false};
    std::atomic<bool> should_stop_processing_{false};

    // Storage and data access
    std::unique_ptr<core::storage::MapStore> map_store_;
    std::shared_ptr<stf::TransformTree> transform_tree_;

    // Batch management
    uint32_t next_batch_id_{1};
    std::vector<core::types::GaussianSplatBatch> splat_batches_in_memory_;

    // Main processing methods
    void processingLoop();
    bool checkForNewMapData();
    bool processNewKeyframes(uint64_t start_keyframe_id, uint64_t end_keyframe_id);
    
    // Splat generation pipeline
    bool generateSplatBatch(const std::vector<core::types::KeyFrame::Ptr>& keyframes,
                           const std::vector<core::types::Keypoint>& map_points,
                           core::types::GaussianSplatBatch& output_batch);
    
    bool generateSplatsFromKeypoints(const std::vector<core::types::Keypoint>& keypoints,
                                   std::vector<core::types::GaussianSplat>& output_splats);
    
    bool generateSplatFromKeypoint(const core::types::Keypoint& keypoint,
                                  core::types::GaussianSplat& output_splat);

    // Covariance estimation methods
    bool estimateCovarianceFromObservations(const core::types::Keypoint& keypoint,
                                          Eigen::Matrix3d& covariance);
    
    bool extractColorFromObservations(const core::types::Keypoint& keypoint,
                                    Eigen::Vector3f& color);

    // Memory management
    void manageSplatBatchMemory();
    bool saveSplatBatchToDisk(const core::types::GaussianSplatBatch& batch);

    // Status and health monitoring  
    bool updateProcessStatus();
    bool isVSLAMProcessHealthy() const;
    
    // Utility methods
    double getCurrentTimestamp() const;
    void logProcessingStats() const;
};

}  // namespace gaussian_splatting