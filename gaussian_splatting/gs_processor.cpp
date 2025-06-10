#include "gs_processor.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <logging/logging.hpp>

namespace gaussian_splatting {

GaussianSplatProcessor::GaussianSplatProcessor(const ProcessorConfig& config)
    : config_(config) {
    LOG(INFO) << "GaussianSplatProcessor initialized with map path: " << config_.map_base_path;
}

GaussianSplatProcessor::~GaussianSplatProcessor() {
    stop();
    LOG(INFO) << "GaussianSplatProcessor destroyed";
}

bool GaussianSplatProcessor::initialize() {
    LOG(INFO) << "Initializing Gaussian splat processor...";
    
    try {
        // Initialize MapStore for reading VSLAM data
        map_store_ = std::make_unique<core::storage::MapStore>(config_.map_base_path);
        if (!map_store_) {
            LOG(ERROR) << "Failed to create MapStore for path: " << config_.map_base_path;
            return false;
        }

        // Load transform tree from map data
        transform_tree_ = map_store_->getTransformTree();
        if (!transform_tree_) {
            LOG(WARNING) << "No transform tree found in map data, will create empty one";
            transform_tree_ = std::make_shared<stf::TransformTree>();
        }

        // Reset statistics
        stats_.reset();
        
        LOG(INFO) << "Gaussian splat processor initialized successfully";
        return true;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to initialize Gaussian splat processor: " << e.what();
        return false;
    }
}

bool GaussianSplatProcessor::start() {
    if (processing_thread_running_.load()) {
        LOG(WARNING) << "Gaussian splat processor already running";
        return true;
    }

    if (!map_store_) {
        LOG(ERROR) << "Cannot start processor: not initialized";
        return false;
    }

    should_stop_processing_ = false;
    processing_thread_running_ = true;
    
    processing_thread_ = std::make_unique<std::thread>(&GaussianSplatProcessor::processingLoop, this);
    
    LOG(INFO) << "Gaussian splat processor started";
    return true;
}

void GaussianSplatProcessor::stop() {
    if (!processing_thread_running_.load()) {
        return;
    }

    LOG(INFO) << "Stopping Gaussian splat processor...";
    
    should_stop_processing_ = true;
    processing_thread_running_ = false;

    if (processing_thread_ && processing_thread_->joinable()) {
        processing_thread_->join();
    }
    
    processing_thread_.reset();
    
    LOG(INFO) << "Gaussian splat processor stopped";
}

void GaussianSplatProcessor::processingLoop() {
    LOG(INFO) << "Gaussian splat processing loop started";
    
    auto last_status_update = std::chrono::steady_clock::now();
    const auto status_update_interval = std::chrono::seconds(5);
    
    while (processing_thread_running_.load() && !should_stop_processing_.load()) {
        auto loop_start = std::chrono::high_resolution_clock::now();
        
        try {
            // Check for new map data from VSLAM process
            if (checkForNewMapData()) {
                stats_.is_processing = true;
                
                // TODO: Implement actual splat generation from new keyframes
                LOG(INFO) << "Processing new map data (placeholder)";
                
                // Update processing statistics
                auto loop_end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
                stats_.last_processing_time_ms = duration.count();
                
                stats_.is_processing = false;
            }
            
            // Periodic status updates
            auto now = std::chrono::steady_clock::now();
            if (now - last_status_update >= status_update_interval) {
                updateProcessStatus();
                logProcessingStats();
                last_status_update = now;
            }
            
            // Memory management
            manageSplatBatchMemory();
            
        } catch (const std::exception& e) {
            LOG(ERROR) << "Error in Gaussian splat processing loop: " << e.what();
            stats_.is_processing = false;
        }
        
        // Sleep to control polling frequency
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<int>(config_.polling_interval_ms)));
    }
    
    LOG(INFO) << "Gaussian splat processing loop ended";
}

bool GaussianSplatProcessor::checkForNewMapData() {
    // Read VSLAM process status
    core::proto::ProcessStatus vslam_status;
    if (!readVSLAMStatus(vslam_status)) {
        return false;
    }
    
    // Check if VSLAM process is healthy
    if (!isVSLAMProcessHealthy()) {
        return false;
    }
    
    // Check if there are new keyframes to process
    uint64_t last_vslam_keyframe = vslam_status.last_processed_keyframe_id();
    uint64_t last_gs_keyframe = stats_.last_processed_keyframe_id.load();
    
    if (last_vslam_keyframe > last_gs_keyframe) {
        LOG(INFO) << "New keyframes available: " << last_gs_keyframe + 1 
                  << " to " << last_vslam_keyframe;
        return processNewKeyframes(last_gs_keyframe + 1, last_vslam_keyframe);
    }
    
    return false;
}

bool GaussianSplatProcessor::processNewKeyframes(uint64_t start_keyframe_id, uint64_t end_keyframe_id) {
    // TODO: Load keyframes and map points from MapStore
    // TODO: Generate splat batches from triangulated keypoints
    LOG(INFO) << "Processing keyframes " << start_keyframe_id << " to " << end_keyframe_id;
    
    // Update statistics
    stats_.total_keyframes_processed += (end_keyframe_id - start_keyframe_id + 1);
    stats_.last_processed_keyframe_id = end_keyframe_id;
    
    return true;
}

bool GaussianSplatProcessor::readVSLAMStatus(core::proto::ProcessStatus& status) const {
    std::string vslam_status_path = config_.map_base_path + "_vslam_status";
    
    std::ifstream file(vslam_status_path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    std::string serialized_data((std::istreambuf_iterator<char>(file)),
                               std::istreambuf_iterator<char>());
    file.close();
    
    return status.ParseFromString(serialized_data);
}

bool GaussianSplatProcessor::writeGSStatus(const core::proto::ProcessStatus& status) const {
    std::ofstream file(config_.status_file_path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    std::string serialized_data;
    if (!status.SerializeToString(&serialized_data)) {
        return false;
    }
    
    file.write(serialized_data.data(), serialized_data.size());
    file.close();
    
    return file.good();
}

bool GaussianSplatProcessor::updateProcessStatus() {
    core::proto::ProcessStatus status;
    status.set_last_processed_keyframe_id(stats_.last_processed_keyframe_id.load());
    status.set_last_processed_splat_batch_id(stats_.last_generated_batch_id.load());
    status.set_timestamp(getCurrentTimestamp());
    status.set_process_name("gaussian_splat_processor");
    status.set_is_healthy(true);
    status.set_status_message("Processing normally");
    
    return writeGSStatus(status);
}

bool GaussianSplatProcessor::isVSLAMProcessHealthy() const {
    core::proto::ProcessStatus vslam_status;
    if (!readVSLAMStatus(vslam_status)) {
        return false;
    }
    
    // Check if VSLAM status is recent (within last 10 seconds)
    double current_time = getCurrentTimestamp();
    double status_age = current_time - vslam_status.timestamp();
    
    if (status_age > 10.0) {
        LOG(WARNING) << "VSLAM process status is stale (" << status_age << "s old)";
        return false;
    }
    
    return vslam_status.is_healthy();
}

void GaussianSplatProcessor::manageSplatBatchMemory() {
    // Remove old splat batches from memory if we exceed limit
    if (splat_batches_in_memory_.size() > config_.max_splat_batches_in_memory) {
        size_t batches_to_remove = splat_batches_in_memory_.size() - config_.max_splat_batches_in_memory;
        
        for (size_t i = 0; i < batches_to_remove; ++i) {
            // Save to disk before removing from memory
            saveSplatBatchToDisk(splat_batches_in_memory_[i]);
        }
        
        splat_batches_in_memory_.erase(splat_batches_in_memory_.begin(), 
                                      splat_batches_in_memory_.begin() + batches_to_remove);
        
        LOG(INFO) << "Removed " << batches_to_remove << " splat batches from memory";
    }
}

bool GaussianSplatProcessor::saveSplatBatchToDisk(const core::types::GaussianSplatBatch& batch) {
    // Save splat batch using MapStore
    return map_store_->addGaussianSplatBatch(batch);
}

double GaussianSplatProcessor::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

void GaussianSplatProcessor::logProcessingStats() const {
    LOG(INFO) << "GS Processor Stats - Keyframes: " << stats_.total_keyframes_processed.load()
              << ", Splat Batches: " << stats_.total_splat_batches_generated.load()
              << ", Total Splats: " << stats_.total_splats_generated.load()
              << ", Last Processing Time: " << stats_.last_processing_time_ms.load() << "ms"
              << ", Processing: " << (stats_.is_processing.load() ? "YES" : "NO");
}

// TODO: Implement splat generation from keyframes and map points
bool GaussianSplatProcessor::generateSplatBatch(const std::vector<core::types::KeyFrame::Ptr>& keyframes,
                                               const std::vector<core::types::Keypoint>& map_points,
                                               core::types::GaussianSplatBatch& output_batch) {
    LOG(INFO) << "generateSplatBatch placeholder - " << keyframes.size() << " keyframes, " 
              << map_points.size() << " map points";
    return false;
}

// TODO: Implement splat generation from triangulated keypoints
bool GaussianSplatProcessor::generateSplatsFromKeypoints(const std::vector<core::types::Keypoint>& keypoints,
                                                       std::vector<core::types::GaussianSplat>& output_splats) {
    LOG(INFO) << "generateSplatsFromKeypoints placeholder - " << keypoints.size() << " keypoints";
    return false;
}

// TODO: Generate single splat from keypoint with observations
bool GaussianSplatProcessor::generateSplatFromKeypoint(const core::types::Keypoint& keypoint,
                                                      core::types::GaussianSplat& output_splat) {
    return false;
}

// TODO: Estimate covariance matrix from keypoint observations
bool GaussianSplatProcessor::estimateCovarianceFromObservations(const core::types::Keypoint& keypoint,
                                                              Eigen::Matrix3d& covariance) {
    return false;
}

// TODO: Extract average color from keypoint observations in keyframes
bool GaussianSplatProcessor::extractColorFromObservations(const core::types::Keypoint& keypoint,
                                                         Eigen::Vector3f& color) {
    return false;
}

}  // namespace gaussian_splatting