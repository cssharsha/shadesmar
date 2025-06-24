#include "gs_processor.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <logging/logging.hpp>
#include <sstream>
#include <thread>

namespace gaussian_splatting {

void GaussianSplatProcessor::initializeLogging() {
    // Initialize glog for this specific processor
    google::InitGoogleLogging("gs_processor");

    // Create logs directory if it doesn't exist
    std::filesystem::create_directories("/logs");

    // Set log file path with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/logs/gs_processor_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << ".log";

    // Configure glog to write to separate files
    FLAGS_log_dir = "/workspace/logs";
    FLAGS_alsologtostderr = true;  // Also log to stderr
    FLAGS_logbufsecs = 0;          // Flush logs immediately

    // Set log file prefix for this processor
    google::SetLogDestination(google::GLOG_INFO, ss.str().c_str());
    google::SetLogDestination(google::GLOG_WARNING, ss.str().c_str());
    google::SetLogDestination(google::GLOG_ERROR, ss.str().c_str());
    google::SetLogDestination(google::GLOG_FATAL, ss.str().c_str());
}

GaussianSplatProcessor::GaussianSplatProcessor(const ProcessorConfig& config) : config_(config) {
    initializeLogging();
    LOG(INFO) << "GaussianSplatProcessor initialized with map path: " << config_.map_base_path;
}

GaussianSplatProcessor::~GaussianSplatProcessor() {
    stop();

    // Clean up shared memory - let SharedMemoryWrapper handle cleanup based on is_creator_
    if (shared_memory_) {
        // Update process health status before disconnecting
        if (shared_memory_->isValid()) {
            auto* region = shared_memory_->getRegion();
            if (region) {
                region->header.gs_process_healthy.store(false);
                LOG(INFO) << "Marked GS process as unhealthy in shared memory before exit";
            }
        }
        // SharedMemoryWrapper destructor will call shm_unlink() only if is_creator_ is true
        shared_memory_.reset();
    }

    LOG(INFO) << "GaussianSplatProcessor destroyed";
}

bool GaussianSplatProcessor::initialize() {
    LOG(INFO) << "Initializing Gaussian splat processor...";

    try {
        // Initialize MapStore for reading VSLAM data (cross-process reader)
        map_store_ = std::make_unique<core::storage::MapStore>(config_.map_base_path,
                                                               core::storage::ProcessRole::READER);
        if (!map_store_) {
            LOG(ERROR) << "Failed to create MapStore for path: " << config_.map_base_path;
            return false;
        }

        // Wait for map data (transform tree, keyframes, keypoints) to be available
        if (!waitForMapData()) {
            LOG(ERROR) << "Failed to wait for map data availability";
            return false;
        }

        // Load transform tree from map data
        transform_tree_ = map_store_->getTransformTree();
        if (!transform_tree_) {
            LOG(ERROR) << "Transform tree should be available but failed to load";
            return false;
        }

        // Print transform tree contents for debugging
        LOG(INFO) << "Successfully loaded transform tree";
        LOG(INFO) << "Transform tree contents:";
        transform_tree_->printTree();

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

    processing_thread_ =
        std::make_unique<std::thread>(&GaussianSplatProcessor::processingLoop, this);

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
                auto duration =
                    std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start);
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
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(config_.polling_interval_ms)));
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

    // DEBUG: Log detailed information about available keyframes and their poses
    LOG(INFO) << "VSLAM Status - Last keyframe ID: " << last_vslam_keyframe
              << ", GS last processed: " << last_gs_keyframe;

    if (last_vslam_keyframe > last_gs_keyframe) {
        LOG(INFO) << "New keyframes available for processing: " << last_gs_keyframe + 1 << " to "
                  << last_vslam_keyframe;

        // DEBUG: Read and log poses of available keyframes from MapStore
        logKeyframePosesDebug(last_gs_keyframe + 1, last_vslam_keyframe);

        return processNewKeyframes(last_gs_keyframe + 1, last_vslam_keyframe);
    }

    return false;
}

bool GaussianSplatProcessor::processNewKeyframes(uint64_t start_keyframe_id,
                                                 uint64_t end_keyframe_id) {
    LOG(INFO) << "Processing keyframes " << start_keyframe_id << " to " << end_keyframe_id;

    try {
        // Load triangulated keypoints from MapStore
        auto all_keypoints = map_store_->getAllKeyPoints();
        LOG(INFO) << "Loaded " << all_keypoints.size() << " keypoints from MapStore";

        // Filter keypoints that are relevant to new keyframes
        std::vector<core::types::Keypoint> relevant_keypoints;
        for (const auto& keypoint : all_keypoints) {
            // Check if keypoint has observations in the new keyframe range
            bool is_relevant = true;
            // for (const auto& location : keypoint.locations) {
            //     if (location.keyframe_id >= start_keyframe_id && location.keyframe_id <=
            //     end_keyframe_id) {
            //         is_relevant = true;
            //         break;
            //     }
            // }

            LOG(INFO) << "Keypoint positon: " << keypoint.position.transpose() << " "
                      << keypoint.needs_triangulation;
            if (is_relevant && !keypoint.needs_triangulation) {
                relevant_keypoints.push_back(keypoint);
            }
        }

        LOG(INFO) << "Found " << relevant_keypoints.size()
                  << " relevant keypoints for splat generation";

        if (relevant_keypoints.empty()) {
            LOG(INFO) << "No triangulated keypoints available for splat generation";
            stats_.last_processed_keyframe_id = end_keyframe_id;
            return true;
        }

        // Generate splats from keypoints
        std::vector<core::types::GaussianSplat> new_splats;
        if (!generateSplatsFromKeypoints(relevant_keypoints, new_splats)) {
            LOG(ERROR) << "Failed to generate splats from keypoints";
            return false;
        }

        // Create splat batch
        core::types::GaussianSplatBatch batch;
        batch.batch_id = next_batch_id_++;
        batch.timestamp = getCurrentTimestamp();
        batch.start_keyframe_id = start_keyframe_id;
        batch.end_keyframe_id = end_keyframe_id;
        batch.splats = std::move(new_splats);

        LOG(INFO) << "Created splat batch " << batch.batch_id << " with " << batch.size()
                  << " splats";

        // Add to in-memory storage
        splat_batches_in_memory_.push_back(batch);

        // Save to disk if configured
        if (!saveSplatBatchToDisk(batch)) {
            LOG(WARNING) << "Failed to save splat batch " << batch.batch_id << " to disk";
        }

        // Update statistics
        stats_.total_keyframes_processed += (end_keyframe_id - start_keyframe_id + 1);
        stats_.last_processed_keyframe_id = end_keyframe_id;
        stats_.total_splat_batches_generated++;
        stats_.total_splats_generated += batch.size();
        stats_.last_generated_batch_id = batch.batch_id;

        LOG(INFO) << "Successfully processed keyframes " << start_keyframe_id << " to "
                  << end_keyframe_id << ", generated " << batch.size() << " splats";

        // Notify visualization callback if set
        if (splat_batch_callback_) {
            try {
                splat_batch_callback_();
            } catch (const std::exception& e) {
                LOG(WARNING) << "Exception in splat batch callback: " << e.what();
            }
        }

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in processNewKeyframes: " << e.what();
        return false;
    }
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
        size_t batches_to_remove =
            splat_batches_in_memory_.size() - config_.max_splat_batches_in_memory;

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
bool GaussianSplatProcessor::generateSplatBatch(
    const std::vector<core::types::KeyFrame::Ptr>& keyframes,
    const std::vector<core::types::Keypoint>& map_points,
    core::types::GaussianSplatBatch& output_batch) {
    LOG(INFO) << "generateSplatBatch placeholder - " << keyframes.size() << " keyframes, "
              << map_points.size() << " map points";
    return false;
}

bool GaussianSplatProcessor::generateSplatsFromKeypoints(
    const std::vector<core::types::Keypoint>& keypoints,
    std::vector<core::types::GaussianSplat>& output_splats) {
    LOG(INFO) << "Generating " << keypoints.size() << " Gaussian splats from keypoints";

    output_splats.clear();
    output_splats.reserve(keypoints.size());

    size_t successful_splats = 0;
    size_t failed_splats = 0;

    for (const auto& keypoint : keypoints) {
        // Skip keypoints that need triangulation or have insufficient observations
        if (keypoint.needs_triangulation || keypoint.locations.size() < 2) {
            failed_splats++;
            continue;
        }

        core::types::GaussianSplat splat;
        if (generateSplatFromKeypoint(keypoint, splat)) {
            output_splats.push_back(splat);
            successful_splats++;
        } else {
            failed_splats++;
        }
    }

    LOG(INFO) << "Generated " << successful_splats << " splats, failed " << failed_splats;
    return successful_splats > 0;
}

bool GaussianSplatProcessor::generateSplatFromKeypoint(const core::types::Keypoint& keypoint,
                                                       core::types::GaussianSplat& output_splat) {
    // Initialize splat with keypoint data
    output_splat.id = next_splat_id_++;
    output_splat.position = keypoint.position;
    output_splat.source_keypoint_id = keypoint.id();
    output_splat.timestamp = getCurrentTimestamp();

    // Estimate covariance from observation geometry
    if (!estimateCovarianceFromObservations(keypoint, output_splat.covariance)) {
        LOG(WARNING) << "Failed to estimate covariance for keypoint " << keypoint.id();
        return false;
    }

    // Extract average color from observations
    if (!extractColorFromObservations(keypoint, output_splat.color)) {
        LOG(WARNING) << "Failed to extract color for keypoint " << keypoint.id();
        // Use default gray color if color extraction fails
        output_splat.color = Eigen::Vector3f(0.5f, 0.5f, 0.5f);
    }

    // Set initial opacity and confidence
    output_splat.opacity = 0.8f;  // High initial opacity
    output_splat.confidence = std::min(1.0f, static_cast<float>(keypoint.locations.size()) / 5.0f);

    // Validate splat before returning
    if (!output_splat.isValid()) {
        LOG(WARNING) << "Generated invalid splat for keypoint " << keypoint.id();
        return false;
    }

    return true;
}

bool GaussianSplatProcessor::estimateCovarianceFromObservations(
    const core::types::Keypoint& keypoint, Eigen::Matrix3d& covariance) {
    if (keypoint.locations.size() < 2) {
        LOG(WARNING) << "Insufficient observations for covariance estimation: "
                     << keypoint.locations.size();
        return false;
    }

    // Use a heuristic approach based on observation geometry
    // The covariance represents uncertainty in the 3D position estimate

    // Base covariance scale (in meters) - smaller for well-observed points
    double base_scale = 0.01;  // 1cm base uncertainty

    // Scale inversely with number of observations (more observations = higher confidence)
    double observation_factor = 1.0 / std::sqrt(static_cast<double>(keypoint.locations.size()));

    // Calculate spread of observations to estimate geometric uncertainty
    std::vector<Eigen::Vector3d> camera_positions;
    camera_positions.reserve(keypoint.locations.size());

    for (const auto& location : keypoint.locations) {
        // Try to get camera position from transform tree for this keyframe
        auto keyframe = map_store_->getKeyFrame(location.keyframe_id);
        if (keyframe) {
            camera_positions.push_back(keyframe->pose.position);
        }
    }

    double geometric_uncertainty = base_scale;
    if (camera_positions.size() >= 2) {
        // Calculate variance in camera positions as proxy for geometric uncertainty
        Eigen::Vector3d mean_position = Eigen::Vector3d::Zero();
        for (const auto& pos : camera_positions) {
            mean_position += pos;
        }
        mean_position /= static_cast<double>(camera_positions.size());

        double position_variance = 0.0;
        for (const auto& pos : camera_positions) {
            position_variance += (pos - mean_position).squaredNorm();
        }
        position_variance /= static_cast<double>(camera_positions.size());

        // Use position variance to modulate uncertainty
        geometric_uncertainty = base_scale * (1.0 + 0.1 * std::sqrt(position_variance));
    }

    // Final scale combines observation count and geometric factors
    double final_scale = geometric_uncertainty * observation_factor;

    // Create isotropic covariance matrix (can be made anisotropic later)
    covariance = Eigen::Matrix3d::Identity() * (final_scale * final_scale);

    // Add small regularization to ensure positive definite
    covariance.diagonal().array() += 1e-6;

    LOG(INFO) << "Estimated covariance for keypoint " << keypoint.id() << " with "
              << keypoint.locations.size() << " observations, scale: " << final_scale;

    return true;
}

bool GaussianSplatProcessor::extractColorFromObservations(const core::types::Keypoint& keypoint,
                                                          Eigen::Vector3f& color) {
    if (keypoint.locations.empty()) {
        LOG(WARNING) << "No observations available for color extraction";
        return false;
    }

    std::vector<Eigen::Vector3f> colors;
    colors.reserve(keypoint.locations.size());

    size_t successful_samples = 0;

    for (const auto& location : keypoint.locations) {
        // Get the keyframe for this observation
        auto keyframe = map_store_->getKeyFrame(location.keyframe_id);
        if (!keyframe || !keyframe->hasColorImage()) {
            continue;
        }

        const auto& color_image = keyframe->getColorImage();
        cv::Mat img = color_image.toCvMat();

        // Check if observation coordinates are within image bounds
        int x = static_cast<int>(std::round(location.x));
        int y = static_cast<int>(std::round(location.y));

        if (x >= 0 && x < img.cols && y >= 0 && y < img.rows) {
            // Sample pixel color at observation location
            Eigen::Vector3f pixel_color;

            if (img.channels() == 3) {
                cv::Vec3b bgr = img.at<cv::Vec3b>(y, x);
                // Convert BGR to RGB and normalize to [0,1]
                pixel_color = Eigen::Vector3f(static_cast<float>(bgr[2]) / 255.0f,  // R
                                              static_cast<float>(bgr[1]) / 255.0f,  // G
                                              static_cast<float>(bgr[0]) / 255.0f   // B
                );
            } else if (img.channels() == 1) {
                // Grayscale image
                uint8_t gray = img.at<uint8_t>(y, x);
                float normalized_gray = static_cast<float>(gray) / 255.0f;
                pixel_color = Eigen::Vector3f(normalized_gray, normalized_gray, normalized_gray);
            } else {
                LOG(WARNING) << "Unsupported image format: " << img.channels() << " channels";
                continue;
            }

            // Validate color values
            if (pixel_color.array().isFinite().all() && (pixel_color.array() >= 0.0f).all() &&
                (pixel_color.array() <= 1.0f).all()) {
                colors.push_back(pixel_color);
                successful_samples++;
            }
        }
    }

    if (successful_samples == 0) {
        LOG(WARNING) << "No valid color samples for keypoint " << keypoint.id();
        return false;
    }

    // Compute average color
    color = Eigen::Vector3f::Zero();
    for (const auto& c : colors) {
        color += c;
    }
    color /= static_cast<float>(colors.size());

    // Clamp to valid range
    color = color.cwiseMax(0.0f).cwiseMin(1.0f);

    LOG(INFO) << "Extracted color for keypoint " << keypoint.id() << " from " << successful_samples
              << "/" << keypoint.locations.size() << " observations: RGB(" << color.x() << ", "
              << color.y() << ", " << color.z() << ")";

    return true;
}

bool GaussianSplatProcessor::waitForMapData() {
    LOG(INFO) << "Waiting for map data (transform tree, keyframes, keypoints) to become available "
                 "via shared memory...";

    // Initialize shared memory for monitoring - use same naming scheme as MapStore
    std::string shared_memory_name = config_.map_base_path;
    std::replace(shared_memory_name.begin(), shared_memory_name.end(), '/', '_');
    std::replace(shared_memory_name.begin(), shared_memory_name.end(), '.', '_');

    shared_memory_ = std::make_unique<core::storage::SharedMemoryWrapper>(shared_memory_name);
    LOG(INFO) << "Connecting to shared memory: " << shared_memory_name;

    if (!shared_memory_->initialize()) {
        LOG(ERROR) << "Failed to initialize shared memory for transform tree monitoring: "
                   << shared_memory_name;
        return false;
    }

    const auto* region = shared_memory_->getRegion();
    if (!region) {
        LOG(ERROR) << "Shared memory region is null";
        return false;
    }

    // Event-driven waiting using data_version counter for efficient notifications
    const int max_wait_seconds = 300;   // 5 minutes timeout
    const int poll_interval_ms = 1000;  // Check every 1 second (much less frequent)

    uint64_t last_data_version = region->header.data_version.load();

    for (int elapsed_ms = 0; elapsed_ms < max_wait_seconds * 1000; elapsed_ms += poll_interval_ms) {
        // Check current data version - if it changed, new data might be available
        uint64_t current_data_version = region->header.data_version.load();
        if (current_data_version != last_data_version) {
            LOG(INFO) << "Data version changed from " << last_data_version << " to "
                      << current_data_version << ", checking for new map data";
            last_data_version = current_data_version;

            // Sync MapStore index to get latest data locations
            map_store_->syncIndexFromDisk();
        }

        // Check if all required data types are available
        bool tf_tree_ready = region->header.tf_tree_available.load();
        bool keyframes_ready = region->header.keyframes_available.load();
        bool keypoints_ready = region->header.keypoints_available.load();

        if (tf_tree_ready && keyframes_ready && keypoints_ready) {
            LOG(INFO) << "All map data types are available (tf_tree: " << tf_tree_ready
                      << ", keyframes: " << keyframes_ready << ", keypoints: " << keypoints_ready
                      << ")";
            return true;
        }

        // Log progress periodically
        if (elapsed_ms % 10000 == 0) {  // Every 10 seconds
            LOG(INFO) << "Still waiting for map data... (" << elapsed_ms / 1000 << "s elapsed) "
                      << "tf_tree: " << tf_tree_ready << ", keyframes: " << keyframes_ready
                      << ", keypoints: " << keypoints_ready;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
    }

    LOG(ERROR) << "Timeout waiting for map data after " << max_wait_seconds << " seconds";
    return false;
}

void GaussianSplatProcessor::logKeyframePosesDebug(uint64_t start_keyframe_id,
                                                   uint64_t end_keyframe_id) const {
    LOG(INFO) << "DEBUG: Reading keyframe poses from MapStore for keyframes " << start_keyframe_id
              << " to " << end_keyframe_id;

    // Get all keyframe poses efficiently
    auto all_poses = map_store_->getAllKeyFramePoses();
    LOG(INFO) << "DEBUG: Total keyframes in MapStore: " << all_poses.size();

    // Log poses for the specific range
    size_t found_keyframes = 0;
    for (uint64_t kf_id = start_keyframe_id; kf_id <= end_keyframe_id; ++kf_id) {
        auto pose_it = all_poses.find(kf_id);
        if (pose_it != all_poses.end()) {
            const auto& pose = pose_it->second;
            LOG(INFO) << "DEBUG: Keyframe " << kf_id << " pose - Position: [" << pose.position.x()
                      << ", " << pose.position.y() << ", " << pose.position.z()
                      << "], Orientation: [" << pose.orientation.w() << ", " << pose.orientation.x()
                      << ", " << pose.orientation.y() << ", " << pose.orientation.z() << "]";
            found_keyframes++;
        } else {
            LOG(WARNING) << "DEBUG: Keyframe " << kf_id << " not found in MapStore poses";
        }
    }

    LOG(INFO) << "DEBUG: Found " << found_keyframes << " keyframes out of "
              << (end_keyframe_id - start_keyframe_id + 1) << " requested";

    // Also try to read individual keyframes to compare
    for (uint64_t kf_id = start_keyframe_id;
         kf_id <= std::min(end_keyframe_id, start_keyframe_id + 2); ++kf_id) {
        auto keyframe = map_store_->getKeyFrame(kf_id);
        if (keyframe) {
            LOG(INFO) << "DEBUG: Direct keyframe " << kf_id << " read - ID: " << keyframe->id
                      << ", Timestamp: " << keyframe->pose.timestamp << ", Position: ["
                      << keyframe->pose.position.x() << ", " << keyframe->pose.position.y() << ", "
                      << keyframe->pose.position.z() << "]";
        } else {
            LOG(WARNING) << "DEBUG: Direct read of keyframe " << kf_id << " failed";
        }
    }

    // Log queue status for debugging
    LOG(INFO) << "DEBUG: MapStore queue status - Unprocessed: " << map_store_->getUnprocessedCount()
              << ", Non-optimized: " << map_store_->getProcessedNonOptimizedCount()
              << ", Optimized: " << map_store_->getProcessedOptimizedCount();

    // Check latest optimization status
    uint64_t last_optimized_id = map_store_->getLastOptimizedKeyFrameId();
    LOG(INFO) << "DEBUG: Last optimized keyframe ID: " << last_optimized_id;
}

void GaussianSplatProcessor::setSplatBatchCallback(SplatBatchCallback callback) {
    splat_batch_callback_ = callback;
    LOG(INFO) << "Splat batch callback set for GaussianSplatProcessor";
}

void GaussianSplatProcessor::clearSplatBatchCallback() {
    splat_batch_callback_ = nullptr;
    LOG(INFO) << "Splat batch callback cleared for GaussianSplatProcessor";
}

}  // namespace gaussian_splatting
