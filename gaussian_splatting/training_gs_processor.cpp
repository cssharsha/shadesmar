#include "training_gs_processor.hpp"
#include <algorithm>
#include <chrono>
#include <logging/logging.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <random>

namespace gaussian_splatting {

void initializeLogging() {
    google::InitGoogleLogging("training_gs_processor");

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/logs/training_gs_processor_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << ".log";

    FLAGS_alsologtostderr = true;
    FLAGS_logbufsecs = 0;
    FLAGS_log_prefix = true;

    google::SetLogDestination(google::GLOG_INFO, ss.str().c_str());
    google::SetLogDestination(google::GLOG_WARNING, "");
    google::SetLogDestination(google::GLOG_ERROR, "");
    google::SetLogDestination(google::GLOG_FATAL, "");
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                               RasterizationT>::TrainingGaussianSplatProcessor(const ConfigType&
                                                                                   config)
    : config_(config) {
    initializeLogging();
    LOG(INFO) << "Initializing asynchronous TrainingGaussianSplatProcessor with device: "
              << (config_.device.is_cuda() ? "CUDA" : "CPU");

    // Initialize training configuration
    training_config_.initial_width = 512;
    training_config_.initial_height = 384;
    training_config_.max_iterations_per_batch = config_.max_training_iterations;

    LOG(INFO) << "Training configuration: " << training_config_.initial_width << "x"
              << training_config_.initial_height
              << ", max iterations: " << training_config_.max_iterations_per_batch;
    LOG(INFO) << "Using shared recording ID: " << config_.training_viz_recording_id;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                               RasterizationT>::~TrainingGaussianSplatProcessor() {
    stop();
    LOG(INFO) << "Asynchronous TrainingGaussianSplatProcessor destroyed";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::initialize() {
    LOG(INFO) << "Initializing asynchronous training processor...";

    try {
        map_store_ = std::make_shared<core::storage::MapStore>(config_.map_base_path,
                                                               core::storage::ProcessRole::READER);
        if (!map_store_) {
            LOG(ERROR) << "Invalid MapStore provided";
            return false;
        }

        // Wait for map data availability
        if (!waitForMapData()) {
            LOG(ERROR) << "Failed to wait for map data";
            return false;
        }

        // Load transform tree
        transform_tree_ = map_store_->getTransformTree();
        if (!transform_tree_) {
            LOG(ERROR) << "Failed to load transform tree";
            return false;
        }
        transform_tree_->printTree();

        // Initialize template-parameterized components
        bilateral_grid_ =
            std::make_unique<BilateralGridT>(rendering::BilateralGridConfig{}, config_.device);

        density_controller_ = std::make_unique<DensityControllerT>(training_config_);

        // BatchTrainer handles all training logic, optimization, rendering, and loss computation
        batch_trainer_ = std::make_unique<training::BatchTrainer>(training_config_, map_store_);

        LOG(INFO) << "Initialized all template components successfully";

        // Reset statistics
        stats_.reset();

        LOG(INFO) << "Asynchronous training processor initialized successfully";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to initialize training processor: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::start() {
    if (main_thread_running_.load()) {
        LOG(WARNING) << "Training processor already running";
        return true;
    }

    if (!map_store_) {
        LOG(ERROR) << "Cannot start: not initialized";
        return false;
    }

    // Reset notification flags
    thread_notification_.should_stop = false;
    thread_notification_.splats_ready = false;
    thread_notification_.training_complete = false;

    // Start all threads
    main_thread_running_ = true;
    training_thread_running_ = true;
    visualization_thread_running_ = true;

    main_thread_ =
        std::make_unique<std::thread>(&TrainingGaussianSplatProcessor::mainThreadLoop, this);
    training_thread_ =
        std::make_unique<std::thread>(&TrainingGaussianSplatProcessor::trainingThreadLoop, this);
    visualization_thread_ = std::make_unique<std::thread>(
        &TrainingGaussianSplatProcessor::visualizationThreadLoop, this);

    LOG(INFO) << "Started all threads: main (map sync + splat generation), training, visualization";
    return true;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::stop() {
    if (!main_thread_running_.load()) {
        return;
    }

    LOG(INFO) << "Stopping asynchronous training processor...";

    // Signal all threads to stop
    thread_notification_.should_stop = true;
    main_thread_running_ = false;
    training_thread_running_ = false;
    visualization_thread_running_ = false;

    // Notify waiting threads
    thread_notification_.splats_ready_cv.notify_all();
    thread_notification_.training_complete_cv.notify_all();

    // Join all threads
    if (main_thread_ && main_thread_->joinable()) {
        main_thread_->join();
    }
    if (training_thread_ && training_thread_->joinable()) {
        training_thread_->join();
    }
    if (visualization_thread_ && visualization_thread_->joinable()) {
        visualization_thread_->join();
    }

    // Reset thread pointers
    main_thread_.reset();
    training_thread_.reset();
    visualization_thread_.reset();

    // Shutdown visualization
    shutdownTrainingVisualization();

    LOG(INFO) << "Asynchronous training processor stopped";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::mainThreadLoop() {
    LOG(INFO) << "Main thread started: Map sync + splat generation";

    while (main_thread_running_.load() && !thread_notification_.should_stop.load()) {
        try {
            // 1. Check for new keyframes from VSLAM
            if (checkForNewKeyframes()) {
                // 2. Sync map store from disk
                if (syncMapStore()) {
                    // 3. Generate splats from keypoints if we have enough keyframes
                    if (generateSplatsFromKeypoints()) {
                        // 4. Notify other threads that splats are ready
                        notifyThreads();
                    }
                }
            }

            // Sleep to avoid excessive polling
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

        } catch (const std::exception& e) {
            LOG(ERROR) << "Error in main thread loop: " << e.what();
        }
    }

    LOG(INFO) << "Main thread ended";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::checkForNewKeyframes() {
    try {
        // Read VSLAM status to get the latest processed keyframe ID
        core::proto::ProcessStatus vslam_status;
        if (!map_store_->readVSLAMStatus(vslam_status)) {
            // Not an error - VSLAM might not have written status yet
            return false;
        }

        // Check if VSLAM is healthy and has processed new keyframes
        if (!vslam_status.is_healthy()) {
            LOG(WARNING) << "VSLAM process is not healthy: " << vslam_status.status_message();
            return false;
        }

        uint64_t latest_vslam_keyframe = vslam_status.last_processed_keyframe_id();

        // Check if we have new keyframes to process for training
        if (latest_vslam_keyframe > last_processed_keyframe_id_) {
            LOG(INFO) << "New keyframes available for splat generation: "
                      << last_processed_keyframe_id_ << " -> " << latest_vslam_keyframe;
            return true;
        }

        return false;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error checking for new keyframes: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::syncMapStore() {
    try {
        // Sync map store index to get latest keyframes and keypoints on disk (not in memory)
        map_store_->syncIndexFromDisk();
        LOG(INFO) << "Map store synced from disk";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error syncing map store: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::generateSplatsFromKeypoints() {
    try {
        // Get current keyframe count from disk (non-blocking read)
        auto all_keyframes = map_store_->getAllKeyFrames();
        if (all_keyframes.size() < config_.keyframe_batch_threshold) {
            LOG(INFO) << "Not enough keyframes for splat generation: " << all_keyframes.size()
                      << " < " << config_.keyframe_batch_threshold;
            return false;
        }

        // Get recent keyframes for current batch
        std::vector<uint64_t> keyframe_ids;
        uint64_t start_id = last_processed_keyframe_id_ + 1;
        for (const auto& kf : all_keyframes) {
            if (kf->id >= start_id && keyframe_ids.size() < config_.keyframes_per_batch) {
                keyframe_ids.push_back(kf->id);
            }
        }

        if (keyframe_ids.empty()) {
            return false;
        }

        LOG(INFO) << "Generating splats for keyframes: " << keyframe_ids.front() << " to "
                  << keyframe_ids.back();

        // Get visible keypoints from these keyframes
        auto visible_keypoints = getVisibleKeypoints(keyframe_ids);
        if (visible_keypoints.empty()) {
            last_processed_keyframe_id_ = keyframe_ids.back();
            LOG(WARNING) << "No visible keypoints found for keyframes";
            LOG(WARNING) << "Setting last_processed_keyframe_id_: " << last_processed_keyframe_id_;
            return false;
        }

        // Initialize splats from visible keypoints
        if (initializeFromSfMKeypoints(visible_keypoints, keyframe_ids)) {
            // Update tracking
            last_processed_keyframe_id_ = keyframe_ids.back();
            current_batch_id_++;
            stats_.splats_initialized = true;
            LOG(INFO) << "Successfully generated splats for batch " << current_batch_id_;
            return true;
        }

        return false;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error generating splats from keypoints: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
std::vector<core::types::Keypoint> TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::getVisibleKeypoints(const std::vector<uint64_t>& keyframe_ids) {
    std::vector<core::types::Keypoint> visible_keypoints;

    try {
        // Get all keypoints from map store (non-blocking read)
        auto all_keypoints = map_store_->getAllKeyPoints();

        // Filter keypoints that are visible from the given keyframes
        for (const auto& keypoint : all_keypoints) {
            // Skip keypoints that need triangulation
            if (keypoint.needs_triangulation) {
                continue;
            }

            // Check if this keypoint is observed by any of the keyframes
            bool is_visible = false;
            for (const auto& location : keypoint.locations) {
                LOG(INFO) << "Location keyframe: " << location.keyframe_id;
                if (std::find(keyframe_ids.begin(), keyframe_ids.end(), location.keyframe_id) !=
                    keyframe_ids.end()) {
                    is_visible = true;
                    break;
                }
            }

            if (is_visible) {
                visible_keypoints.push_back(keypoint);
            }
        }

        LOG(INFO) << "Found " << visible_keypoints.size() << " visible keypoints from "
                  << all_keypoints.size() << " total keypoints for " << keyframe_ids.size()
                  << " keyframes";

        return visible_keypoints;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error getting visible keypoints: " << e.what();
        return {};
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::initializeFromSfMKeypoints(const std::vector<core::types::Keypoint>& keypoints,
                                                const std::vector<uint64_t>& keyframe_ids) {
    LOG(INFO) << "Initializing " << keypoints.size() << " splats from SfM keypoints";

    try {
        // Get keyframes for color extraction (non-blocking read)
        std::unordered_map<uint64_t, core::types::KeyFrame::Ptr> keyframe_map;
        for (const auto& kf_id : keyframe_ids) {
            auto keyframe = map_store_->getKeyFrame(kf_id);
            if (keyframe) {
                keyframe_map[kf_id] = keyframe;
            }
        }

        // Create splat batch from keypoints
        core::types::GaussianSplatBatch splat_batch;
        splat_batch.batch_id = current_batch_id_;
        splat_batch.timestamp = getCurrentTimestamp();
        
        // Store source keyframe IDs
        for (const auto& kf_id : keyframe_ids) {
            splat_batch.source_keyframe_ids.insert(kf_id);
        }

        // Convert each keypoint to a gaussian splat
        for (const auto& keypoint : keypoints) {
            core::types::GaussianSplat splat;
            splat.id = next_splat_id_++;
            splat.position = keypoint.position;
            splat.source_keypoint_id = keypoint.id();

            // Initialize color by extracting from keyframe observations
            Eigen::Vector3f extracted_color = extractColorFromKeyframes(keypoint, keyframe_map);
            splat.color = extracted_color;

            // Initialize covariance based on keypoint uncertainty and observations
            Eigen::Matrix3d covariance = computeKeypointCovariance(keypoint, keyframe_map);
            splat.covariance = covariance;

            // Initialize opacity and confidence
            splat.opacity = computeInitialOpacity(keypoint);
            splat.confidence = computeInitialConfidence(keypoint);
            splat.timestamp = getCurrentTimestamp();

            splat_batch.splats.push_back(splat);
        }

        // Write splat batch to disk (blocking write)
        if (writeSplatBatchToDisk(splat_batch)) {
            stats_.initialized_splat_count = splat_batch.splats.size();
            LOG(INFO) << "Successfully initialized and wrote " << splat_batch.splats.size()
                      << " splats to disk for batch ID: " << splat_batch.batch_id;
            LOG(INFO) << "Splat batch written to disk - notifying other threads";
            return true;
        }

        LOG(ERROR) << "Failed to write splat batch to disk";
        return false;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to initialize from SfM keypoints: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::writeSplatBatchToDisk(const core::types::GaussianSplatBatch& batch) {
    try {
        // This is a blocking write operation
        std::lock_guard<std::mutex> lock(thread_notification_.notification_mutex);

        if (!map_store_->addGaussianSplatBatch(batch)) {
            LOG(ERROR) << "Failed to write splat batch to disk";
            return false;
        }

        if (!map_store_->writeSplatBatchToDisk(batch.batch_id)) {
            LOG(ERROR) << "Failed to write splat batch to disk for " << batch.batch_id;
            return false;
        }

        // Update notification for other threads
        thread_notification_.latest_splat_batch_id = batch.batch_id;

        // Enqueue batch for training
        enqueueBatchForTraining(batch.batch_id);
        
        // Enqueue batch for visualization (initialized state)
        enqueueInitializedBatch(batch.batch_id);

        LOG(INFO) << "Wrote splat batch " << batch.batch_id << " with " << batch.splats.size()
                  << " splats to disk - available for training and visualization";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error writing splat batch to disk: " << e.what();
        return false;
    }
}

// =============================================================================
// TRAINING THREAD: Load splats from disk and train
// =============================================================================

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::trainingThreadLoop() {
    LOG(INFO) << "Training thread started";

    while (training_thread_running_.load() && !thread_notification_.should_stop.load()) {
        try {
            // Check if there are pending batches in the queue
            uint32_t batch_id;
            if (!dequeueBatchForTraining(batch_id)) {
                // No pending batches, wait briefly and check again
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            if (thread_notification_.should_stop.load()) {
                break;
            }

            LOG(INFO) << "Processing batch " << batch_id << " from training queue";

            // Load splat batch from disk (non-blocking read)
            core::types::GaussianSplatBatch splat_batch;
            if (!loadSplatBatchFromDisk(batch_id, splat_batch)) {
                LOG(WARNING) << "Failed to load splat batch " << batch_id << " from disk";
                continue;
            }

            // Load corresponding keyframes using stored keyframe IDs
            training::KeyframeBatch keyframe_batch;
            std::vector<uint64_t> keyframe_ids(splat_batch.source_keyframe_ids.begin(), 
                                               splat_batch.source_keyframe_ids.end());

            if (!loadKeyframeBatchFromDisk(keyframe_ids, keyframe_batch)) {
                LOG(WARNING) << "Failed to load keyframe batch from disk for batch " << batch_id;
                continue;
            }

            // Execute training on the loaded batches
            if (executeTrainingOnBatch(splat_batch, keyframe_batch)) {
                // Write trained splats back to disk (blocking write)
                writeTrainedSplatsToDisk(splat_batch);

                // Enqueue batch for visualization (trained state)
                enqueueTrainedBatch(batch_id);

                // Notify visualization thread of training completion
                thread_notification_.training_complete = true;
                thread_notification_.training_complete_cv.notify_all();
                
                LOG(INFO) << "Successfully completed training for batch " << batch_id;
            } else {
                LOG(ERROR) << "Training failed for batch " << batch_id;
            }

        } catch (const std::exception& e) {
            LOG(ERROR) << "Error in training thread loop: " << e.what();
        }

        // Sleep briefly before checking for new work
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG(INFO) << "Training thread ended";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::loadSplatBatchFromDisk(uint64_t batch_id,
                                            core::types::GaussianSplatBatch& batch) {
    try {
        // Non-blocking read from disk
        auto splat_batches = map_store_->getAllGaussianSplatBatches();
        for (const auto& splat_batch : splat_batches) {
            if (splat_batch.batch_id == batch_id) {
                batch = splat_batch;
                LOG(INFO) << "Loaded splat batch " << batch_id << " with " << batch.splats.size()
                          << " splats from disk";
                return true;
            }
        }

        LOG(WARNING) << "Splat batch " << batch_id << " not found on disk";
        return false;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error loading splat batch from disk: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::loadKeyframeBatchFromDisk(const std::vector<uint64_t>& keyframe_ids,
                                               training::KeyframeBatch& batch) {
    try {
        // Non-blocking read of keyframes from disk
        batch.batch_id = current_batch_id_;
        batch.keyframe_ids = keyframe_ids;
        batch.batch_size = keyframe_ids.size();

        std::vector<torch::Tensor> images;
        std::vector<torch::Tensor> camera_poses;
        std::vector<torch::Tensor> camera_intrinsics;

        for (const auto& kf_id : keyframe_ids) {
            auto keyframe = map_store_->getKeyFrame(kf_id);
            if (!keyframe) {
                LOG(WARNING) << "Failed to load keyframe " << kf_id;
                continue;
            }

            // Extract image tensor
            torch::Tensor image_tensor;
            core::types::CameraInfo camera_info;
            if (!extractImageTensor(keyframe, image_tensor, camera_info)) {
                LOG(WARNING) << "Failed to extract image tensor from keyframe " << kf_id;
                continue;
            }

            // Extract camera pose
            Eigen::Isometry3d camera_pose = keyframe->pose.getEigenIsometry();
            torch::Tensor pose_tensor = convertCameraPoseToTensor(camera_pose);

            // Extract camera intrinsics
            torch::Tensor intrinsics_tensor = convertCameraIntrinsicsToTensor(camera_info);

            images.push_back(image_tensor);
            camera_poses.push_back(pose_tensor);
            camera_intrinsics.push_back(intrinsics_tensor);
        }

        if (images.empty()) {
            LOG(ERROR) << "No valid keyframes loaded in batch";
            return false;
        }

        // Stack tensors
        batch.images = torch::stack(images, /*dim=*/0);
        batch.camera_poses = torch::stack(camera_poses, /*dim=*/0);
        batch.camera_intrinsics = torch::stack(camera_intrinsics, /*dim=*/0);

        LOG(INFO) << "Loaded keyframe batch with " << images.size() << " keyframes from disk";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error loading keyframe batch from disk: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::executeTrainingOnBatch(const core::types::GaussianSplatBatch& splat_batch,
                                            const training::KeyframeBatch& keyframe_batch) {
    LOG(INFO) << "Executing training on batch " << splat_batch.batch_id << " with "
              << splat_batch.splats.size() << " splats and " << keyframe_batch.batch_size
              << " keyframes";

    stats_.is_training = true;

    try {
        // Setup the batch in the trainer for training
        if (!batch_trainer_->setupBatchForTraining(splat_batch)) {
            LOG(ERROR) << "Failed to setup batch for training";
            stats_.is_training = false;
            return false;
        }

        // Setup multi-view rendering with the keyframes
        std::vector<uint64_t> keyframe_ids;
        for (size_t i = 0; i < keyframe_batch.batch_size; ++i) {
            keyframe_ids.push_back(keyframe_batch.keyframe_ids[i]);
        }
        batch_trainer_->setupMultiViewRendering(keyframe_ids);

        // Use the existing BatchTrainer to perform the actual training
        training::TrainingResults results;
        if (!batch_trainer_->trainBatch(splat_batch.batch_id, results)) {
            LOG(ERROR) << "Training failed for batch " << splat_batch.batch_id;
            stats_.is_training = false;
            return false;
        }

        // Update statistics from training results
        stats_.total_iterations += results.iterations_completed;
        stats_.current_loss = results.total_loss;
        stats_.is_training = false;

        // Non-blocking callback for training stats with detailed results
        if (training_progress_callback_) {
            training_progress_callback_(stats_.total_iterations.load(), results.total_loss,
                                        static_cast<int>(splat_batch.splats.size()));
        }

        // Update visualization callback with detailed loss components
        if (training_visualizer_) {
            handleTrainingStatsCallback(stats_.total_iterations.load(), results.total_loss,
                                        results.l1_loss, results.d_ssim_loss,
                                        static_cast<int>(splat_batch.splats.size()));
        }

        LOG(INFO) << "Completed training on batch " << splat_batch.batch_id
                  << " with total loss: " << results.total_loss << ", L1 loss: " << results.l1_loss
                  << ", D-SSIM loss: " << results.d_ssim_loss << " over "
                  << results.iterations_completed << " iterations";

        return results.success;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error during batch training: " << e.what();
        stats_.is_training = false;
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::writeTrainedSplatsToDisk(const core::types::GaussianSplatBatch& batch) {
    try {
        // This is a blocking write operation
        std::lock_guard<std::mutex> lock(thread_notification_.notification_mutex);

        // Create trained splat batch with updated timestamp
        core::types::GaussianSplatBatch trained_batch = batch;
        trained_batch.timestamp = getCurrentTimestamp();

        if (!map_store_->addGaussianSplatBatch(trained_batch)) {
            LOG(ERROR) << "Failed to write trained splat batch to disk";
            return false;
        }

        if (!map_store_->writeSplatBatchToDisk(trained_batch.batch_id)) {
            LOG(ERROR) << "Failed to write splat batch " << trained_batch.batch_id << " to disk";
            return false;
        }

        LOG(INFO) << "Wrote trained splat batch " << batch.batch_id << " with "
                  << batch.splats.size() << " splats to disk";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error writing trained splats to disk: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::visualizationThreadLoop() {
    LOG(INFO) << "Visualization thread started";

    // Initialize visualization
    if (!initializeTrainingVisualization()) {
        LOG(ERROR) << "Failed to initialize training visualization, thread exiting";
        return;
    }

    while (visualization_thread_running_.load() && !thread_notification_.should_stop.load()) {
        try {
            bool updated = false;
            
            // Check if there are new initialized batches
            if (hasNewInitializedBatches()) {
                LOG(INFO) << "New initialized batches detected, updating visualization";
                
                // Get all initialized batch IDs and visualize them
                auto initialized_batch_ids = getAllInitializedBatches();
                for (const auto& batch_id : initialized_batch_ids) {
                    visualizeInitializedSplatBatch(batch_id);
                }
                updated = true;
            }
            
            // Check if there are new trained batches
            if (hasNewTrainedBatches()) {
                LOG(INFO) << "New trained batches detected, updating visualization";
                
                // Get all trained batch IDs and visualize them
                auto trained_batch_ids = getAllTrainedBatches();
                for (const auto& batch_id : trained_batch_ids) {
                    visualizeTrainedSplatBatch(batch_id);
                }
                updated = true;
            }
            
            if (updated) {
                LOG(INFO) << "Visualization updated with new batches";
            }

            // Update visualization at reasonable frequency
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 1 Hz

        } catch (const std::exception& e) {
            LOG(ERROR) << "Error in visualization thread loop: " << e.what();
        }
    }

    LOG(INFO) << "Visualization thread ended";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::initializeTrainingVisualization() {
    LOG(INFO) << "Initializing training visualization with shared recording ID: "
              << config_.training_viz_recording_id;

    try {
        training_visualizer_ = std::make_unique<visualization::RerunTrainingVisualizer>(
            config_.training_viz_recording_id, config_.training_viz_host,
            config_.training_viz_port);

        if (!training_visualizer_->initialize()) {
            LOG(ERROR) << "Failed to initialize RerunTrainingVisualizer";
            training_visualizer_.reset();
            return false;
        }

        // Log initial training state
        training_visualizer_->visualizeTrainingState("Initialized",
                                                     "Asynchronous training processor ready");

        LOG(INFO) << "Training visualization initialized successfully with shared recording ID";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to initialize training visualization: " << e.what();
        training_visualizer_.reset();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::shutdownTrainingVisualization() {
    if (training_visualizer_) {
        training_visualizer_->visualizeTrainingState("Shutdown", "Asynchronous training completed");
        training_visualizer_->shutdown();
        training_visualizer_.reset();
        LOG(INFO) << "Training visualization shut down";
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::visualizeInitializedSplats() {
    if (!training_visualizer_) {
        return false;
    }

    try {
        // Non-blocking read of splat batches from disk
        auto splat_batches = map_store_->getAllGaussianSplatBatches();

        if (splat_batches.empty()) {
            return true;  // No splats to visualize yet
        }

        // Visualize all splat batches (both initialized and trained)
        for (const auto& batch : splat_batches) {
            // Visualize splats using shared recording ID
            training_visualizer_->visualizeCurrentSplats(batch.splats, batch.batch_id);
            LOG(INFO) << "Visualized " << batch.splats.size() << " splats from batch "
                      << batch.batch_id;
        }

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error visualizing initialized splats: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::visualizeTrainedSplats() {
    if (!training_visualizer_) {
        return false;
    }

    try {
        // Non-blocking read of trained splat batches from disk
        auto splat_batches = map_store_->getAllGaussianSplatBatches();

        if (splat_batches.empty()) {
            return true;  // No trained splats to visualize yet
        }

        // Visualize latest trained splats with enhanced logging
        const auto& latest_batch = splat_batches.back();
        training_visualizer_->visualizeCurrentSplats(latest_batch.splats, latest_batch.batch_id);
        LOG(INFO) << "Visualized latest trained splats: " << latest_batch.splats.size()
                  << " splats from batch " << latest_batch.batch_id;

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error visualizing trained splats: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::visualizeInitializedSplatBatch(uint32_t batch_id) {
    if (!training_visualizer_) {
        return false;
    }

    try {
        // Load specific splat batch from disk
        core::types::GaussianSplatBatch splat_batch;
        if (!loadSplatBatchFromDisk(batch_id, splat_batch)) {
            LOG(WARNING) << "Failed to load splat batch " << batch_id << " for visualization";
            return false;
        }

        // Visualize the initialized splats
        training_visualizer_->visualizeCurrentSplats(splat_batch.splats, batch_id);
        LOG(INFO) << "Visualized initialized splat batch " << batch_id << " with " 
                  << splat_batch.splats.size() << " splats";

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error visualizing initialized splat batch " << batch_id << ": " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::visualizeTrainedSplatBatch(uint32_t batch_id) {
    if (!training_visualizer_) {
        return false;
    }

    try {
        // Load specific trained splat batch from disk
        core::types::GaussianSplatBatch splat_batch;
        if (!loadSplatBatchFromDisk(batch_id, splat_batch)) {
            LOG(WARNING) << "Failed to load trained splat batch " << batch_id << " for visualization";
            return false;
        }

        // Visualize the trained splats with different identifier to distinguish from initialized
        training_visualizer_->visualizeCurrentSplats(splat_batch.splats, batch_id + 10000);  // Offset for trained visualization
        LOG(INFO) << "Visualized trained splat batch " << batch_id << " with " 
                  << splat_batch.splats.size() << " splats";

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error visualizing trained splat batch " << batch_id << ": " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::handleTrainingStatsCallback(int iteration,
                                                                                 float total_loss,
                                                                                 float l1_loss,
                                                                                 float d_ssim_loss,
                                                                                 int splat_count) {
    if (!training_visualizer_) {
        return;
    }

    // This is called as a non-blocking callback from training thread
    try {
        // Create training metrics for visualization using actual loss components
        visualization::TrainingMetrics metrics;
        metrics.epoch = 1;  // TODO: Track actual epochs
        metrics.batch_in_epoch = current_batch_id_;
        metrics.total_batches_processed = iteration;
        metrics.total_loss = total_loss;
        metrics.l1_loss = l1_loss;        // Use actual L1 loss from BatchTrainer
        metrics.ssim_loss = d_ssim_loss;  // Use actual D-SSIM loss from BatchTrainer
        metrics.learning_rate = training_config_.learning_rate;  // Use config learning rate
        metrics.batch_processing_time_ms = 0.0;
        metrics.timestamp = std::chrono::steady_clock::now();
        metrics.num_splats = splat_count;
        metrics.num_keyframes_in_batch = config_.keyframes_per_batch;

        // Log the metrics (non-blocking)
        training_visualizer_->logTrainingMetrics(metrics);

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error in training stats callback: " << e.what();
    }
}

// =============================================================================
// UTILITY METHODS (borrowed from original implementation)
// =============================================================================

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::setTrainingProgressCallback(TrainingProgressCallback callback) {
    training_progress_callback_ = callback;
    LOG(INFO) << "Training progress callback set";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::clearTrainingProgressCallback() {
    training_progress_callback_ = nullptr;
    LOG(INFO) << "Training progress callback cleared";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::notifyThreads() {
    std::lock_guard<std::mutex> lock(thread_notification_.notification_mutex);
    thread_notification_.splats_ready = true;
    thread_notification_.splats_ready_cv.notify_all();
    LOG(INFO) << "Notified threads that splats are ready";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::waitForMapData() {
    LOG(INFO) << "Waiting for map data to become available via shared memory...";

    // Initialize shared memory for monitoring
    std::string shared_memory_name = config_.map_base_path;
    std::replace(shared_memory_name.begin(), shared_memory_name.end(), '/', '_');
    std::replace(shared_memory_name.begin(), shared_memory_name.end(), '.', '_');

    shared_memory_ = std::make_unique<core::storage::SharedMemoryWrapper>(shared_memory_name);
    LOG(INFO) << "Connecting to shared memory: " << shared_memory_name;

    if (!shared_memory_->initialize()) {
        LOG(ERROR) << "Failed to initialize shared memory: " << shared_memory_name;
        return false;
    }

    const auto* region = shared_memory_->getRegion();
    if (!region) {
        LOG(ERROR) << "Shared memory region is null";
        return false;
    }

    const int max_wait_seconds = 300;
    const int poll_interval_ms = 1000;

    for (int elapsed_ms = 0; elapsed_ms < max_wait_seconds * 1000; elapsed_ms += poll_interval_ms) {
        bool tf_tree_ready = region->header.tf_tree_available.load();
        bool keyframes_ready = region->header.keyframes_available.load();
        bool keypoints_ready = region->header.keypoints_available.load();

        if (tf_tree_ready && keyframes_ready && keypoints_ready) {
            LOG(INFO) << "All map data types are available";
            return true;
        }

        if (elapsed_ms % 10000 == 0) {
            LOG(INFO) << "Still waiting for map data... (" << elapsed_ms / 1000 << "s elapsed)";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
    }

    LOG(ERROR) << "Timeout waiting for map data after " << max_wait_seconds << " seconds";
    return false;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
double TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                      RasterizationT>::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::logTrainingStats() const {
    LOG(INFO) << "Training Stats - Batch: " << stats_.current_batch_id.load()
              << ", Iterations: " << stats_.total_iterations.load()
              << ", Loss: " << stats_.current_loss.load()
              << ", Splats: " << stats_.current_splat_count.load()
              << ", Training: " << (stats_.is_training.load() ? "YES" : "NO")
              << ", Initialized: " << (stats_.splats_initialized.load() ? "YES" : "NO");
}

// =============================================================================
// HELPER METHODS (borrowed from original implementation)
// =============================================================================

// Helper method to extract color from keyframe observations
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
Eigen::Vector3f TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                               RasterizationT>::
    extractColorFromKeyframes(
        const core::types::Keypoint& keypoint,
        const std::unordered_map<uint64_t, core::types::KeyFrame::Ptr>& keyframe_map) {
    std::vector<Eigen::Vector3f> colors;
    std::vector<float> weights;

    // Extract color from each keyframe observation
    for (const auto& location : keypoint.locations) {
        auto kf_it = keyframe_map.find(location.keyframe_id);
        if (kf_it == keyframe_map.end())
            continue;

        const auto& keyframe = kf_it->second;
        if (!keyframe->hasColorImage())
            continue;

        // Extract pixel color at keypoint location
        const auto& color_image = keyframe->getColorImage();
        cv::Mat cv_image = color_image.toCvMat();

        // Convert pixel coordinates to image coordinates
        int x = static_cast<int>(location.x);
        int y = static_cast<int>(location.y);

        // Check bounds
        if (x >= 0 && x < cv_image.cols && y >= 0 && y < cv_image.rows) {
            cv::Vec3b pixel = cv_image.at<cv::Vec3b>(y, x);

            // Convert BGR to RGB and normalize
            Eigen::Vector3f color(pixel[2] / 255.0f, pixel[1] / 255.0f, pixel[0] / 255.0f);

            // Use uniform weighting since response is not available in Location struct
            float weight = 1.0f;

            colors.push_back(color);
            weights.push_back(weight);
        }
    }

    // Compute weighted average color
    if (!colors.empty()) {
        Eigen::Vector3f weighted_color = Eigen::Vector3f::Zero();
        float total_weight = 0.0f;

        for (size_t i = 0; i < colors.size(); ++i) {
            weighted_color += weights[i] * colors[i];
            total_weight += weights[i];
        }

        if (total_weight > 0) {
            return weighted_color / total_weight;
        }
    }

    // Fallback to neutral color if no valid observations
    return Eigen::Vector3f(0.7f, 0.7f, 0.7f);
}

// Helper method to compute covariance from keypoint observations
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
Eigen::Matrix3d TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                               RasterizationT>::
    computeKeypointCovariance(
        const core::types::Keypoint& keypoint,
        const std::unordered_map<uint64_t, core::types::KeyFrame::Ptr>& keyframe_map) {
    // Base covariance scaled by observation quality
    double base_variance = 0.01;

    // Scale based on number of observations (more observations = lower uncertainty)
    if (keypoint.locations.size() > 3) {
        base_variance = 0.005;
    } else if (keypoint.locations.size() > 1) {
        base_variance = 0.008;
    }

    // Compute viewing angle variance - points seen from more diverse angles are more certain
    std::vector<Eigen::Vector3d> viewing_directions;
    for (const auto& location : keypoint.locations) {
        auto kf_it = keyframe_map.find(location.keyframe_id);
        if (kf_it != keyframe_map.end()) {
            const auto& keyframe = kf_it->second;
            Eigen::Vector3d view_dir = (keypoint.position - keyframe->pose.position).normalized();
            viewing_directions.push_back(view_dir);
        }
    }

    // Compute viewing angle spread
    if (viewing_directions.size() > 1) {
        double angle_variance = 0.0;
        for (size_t i = 0; i < viewing_directions.size(); ++i) {
            for (size_t j = i + 1; j < viewing_directions.size(); ++j) {
                double angle = std::acos(
                    std::clamp(viewing_directions[i].dot(viewing_directions[j]), -1.0, 1.0));
                angle_variance += angle * angle;
            }
        }
        angle_variance /= (viewing_directions.size() * (viewing_directions.size() - 1) / 2);

        // More diverse viewing angles reduce uncertainty
        double angle_factor = std::exp(-angle_variance * 2.0);
        base_variance *= (0.5 + 0.5 * angle_factor);
    }

    // Scale by distance from cameras (closer points are more certain)
    if (!viewing_directions.empty()) {
        double avg_distance = 0.0;
        int count = 0;
        for (const auto& location : keypoint.locations) {
            auto kf_it = keyframe_map.find(location.keyframe_id);
            if (kf_it != keyframe_map.end()) {
                const auto& keyframe = kf_it->second;
                avg_distance += (keypoint.position - keyframe->pose.position).norm();
                count++;
            }
        }
        if (count > 0) {
            avg_distance /= count;
            // Scale variance with distance (farther points are less certain)
            base_variance *= std::max(0.5, std::min(2.0, avg_distance / 5.0));
        }
    }

    return Eigen::Matrix3d::Identity() * base_variance;
}

// Helper method to compute initial opacity
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
float TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::computeInitialOpacity(const core::types::Keypoint& keypoint) {
    // Higher opacity for keypoints with more observations
    float opacity = 0.5f + 0.3f * std::min(1.0f, keypoint.locations.size() / 5.0f);
    return std::clamp(opacity, 0.1f, 0.9f);
}

// Helper method to compute initial confidence
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
float TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::computeInitialConfidence(const core::types::Keypoint& keypoint) {
    // Confidence based on observation count and descriptor quality
    float base_confidence = 0.8f;
    if (keypoint.locations.size() > 2) {
        base_confidence = 0.95f;
    } else if (keypoint.locations.size() > 1) {
        base_confidence = 0.85f;
    }

    return std::clamp(base_confidence, 0.3f, 1.0f);
}

// Scene bounds estimation
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
std::pair<Eigen::Vector3f, Eigen::Vector3f> TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT, RasterizationT>::estimateSceneBounds() {
    Eigen::Vector3f scene_min = config_.scene_min;
    Eigen::Vector3f scene_max = config_.scene_max;

    try {
        // Try to estimate bounds from existing keyframes
        auto keyframes = map_store_->getAllKeyFrames();
        if (!keyframes.empty()) {
            LOG(INFO) << "Estimating scene bounds from " << keyframes.size() << " keyframes";

            // Initialize with first keyframe position
            Eigen::Vector3f first_pos = keyframes[0]->pose.position.cast<float>();
            scene_min = first_pos;
            scene_max = first_pos;

            // Expand bounds to include all keyframe positions
            for (const auto& kf : keyframes) {
                Eigen::Vector3f pos = kf->pose.position.cast<float>();
                scene_min = scene_min.cwiseMin(pos);
                scene_max = scene_max.cwiseMax(pos);
            }

            // Add padding around camera trajectory
            Eigen::Vector3f padding(2.0f, 2.0f, 2.0f);
            scene_min -= padding;
            scene_max += padding;

            LOG(INFO) << "Estimated scene bounds from keyframes: min(" << scene_min.transpose()
                      << ") max(" << scene_max.transpose() << ")";
        } else {
            LOG(INFO) << "No keyframes available, using config scene bounds";
        }

        // Also try to incorporate existing keypoints if available
        auto keypoints = map_store_->getAllKeyPoints();
        if (!keypoints.empty() && keyframes.empty()) {
            LOG(INFO) << "Estimating scene bounds from " << keypoints.size() << " keypoints";

            // Initialize with first keypoint
            Eigen::Vector3f first_pos = keypoints[0].position.cast<float>();
            scene_min = first_pos;
            scene_max = first_pos;

            // Expand to include all keypoints
            for (const auto& kp : keypoints) {
                if (!kp.needs_triangulation) {  // Only use triangulated keypoints
                    Eigen::Vector3f pos = kp.position.cast<float>();
                    scene_min = scene_min.cwiseMin(pos);
                    scene_max = scene_max.cwiseMax(pos);
                }
            }

            // Add padding
            Eigen::Vector3f padding(1.0f, 1.0f, 1.0f);
            scene_min -= padding;
            scene_max += padding;
        }

    } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to estimate scene bounds from map data: " << e.what()
                     << ", using config bounds";
    }

    return std::make_pair(scene_min, scene_max);
}

// Image processing
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::extractImageTensor(const core::types::KeyFrame::Ptr& keyframe,
                                        torch::Tensor& image_tensor,
                                        core::types::CameraInfo& camera_info) {
    try {
        // Check if keyframe has color image data
        if (!keyframe->hasColorImage()) {
            LOG(ERROR) << "Keyframe " << keyframe->id << " has no color image";
            return false;
        }

        // Get camera info
        if (!keyframe->hasCameraInfo()) {
            LOG(ERROR) << "Keyframe " << keyframe->id << " has no camera info";
            return false;
        }

        camera_info = keyframe->getCameraInfo();
        const auto& color_image = keyframe->getColorImage();

        // Convert OpenCV Mat to LibTorch tensor
        cv::Mat cv_image = color_image.toCvMat();

        // Ensure image is in RGB format (OpenCV uses BGR by default)
        if (cv_image.channels() == 3) {
            cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
        }

        // Resize to training resolution
        cv::Mat resized_image;
        cv::resize(cv_image, resized_image,
                   cv::Size(training_config_.initial_width, training_config_.initial_height));

        // Convert to float and normalize to [0, 1]
        cv::Mat float_image;
        resized_image.convertTo(float_image, CV_32F, 1.0 / 255.0);

        // Convert to LibTorch tensor (HWC -> CHW format)
        image_tensor =
            torch::from_blob(float_image.data,
                             {training_config_.initial_height, training_config_.initial_width, 3},
                             torch::kFloat32)
                .clone();

        // Permute dimensions from HWC to CHW (channels first for neural networks)
        image_tensor = image_tensor.permute({2, 0, 1});

        // Move to configured device
        image_tensor = image_tensor.to(config_.device);

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to extract image tensor: " << e.what();
        return false;
    }
}

// Camera parameter extraction
template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::extractCameraPoses(const std::vector<uint64_t>& keyframe_ids,
                                        std::vector<Eigen::Isometry3d>& camera_poses,
                                        const std::string& target_frame) {
    try {
        camera_poses.clear();
        camera_poses.reserve(keyframe_ids.size());

        if (!transform_tree_) {
            LOG(ERROR) << "Transform tree not available for camera pose extraction";
            return false;
        }

        // Get static transform from base_link to camera frame
        auto getTransformFromTree = [&](const std::string& camera_frame) {
            try {
                auto transform_result = transform_tree_->getTransform("base_link", "camera");
                return transform_result.transform;
            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to get base_link to camera transform: " << e.what()
                             << ". Using identity transform.";
                return Eigen::Isometry3d::Identity();
            }
        };

        for (const auto& kf_id : keyframe_ids) {
            auto keyframe = map_store_->getKeyFrame(kf_id);
            if (!keyframe) {
                LOG(WARNING) << "Failed to load keyframe " << kf_id << " for pose extraction";
                continue;
            }

            Eigen::Isometry3d T_baselink_camera =
                getTransformFromTree("camera_color_optical_frame");

            // Get keyframe's pose in world frame (this is base_link pose in odom frame)
            Eigen::Isometry3d T_world_baselink = keyframe->pose.getEigenIsometry();

            // Chain transforms: T_world_camera = T_world_baselink * T_baselink_camera
            Eigen::Isometry3d T_world_camera = T_world_baselink * T_baselink_camera;

            // Transform to target frame if different from world/odom
            if (target_frame != "world" && target_frame != "odom") {
                try {
                    auto transform_result = transform_tree_->getTransform(target_frame, "odom");
                    T_world_camera = transform_result.transform * T_world_camera;
                } catch (const std::exception& e) {
                    LOG(WARNING) << "Failed to transform to target frame " << target_frame << ": "
                                 << e.what() << ". Using odom frame.";
                }
            }

            camera_poses.push_back(T_world_camera);
        }

        return !camera_poses.empty();

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error extracting camera poses: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
torch::Tensor TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::convertCameraIntrinsicsToTensor(const core::types::CameraInfo& camera_info) {
    try {
        // Convert camera matrix to tensor
        Eigen::Matrix3d K = camera_info.getKInEigen();

        // Create tensor from camera matrix
        torch::Tensor K_tensor =
            torch::from_blob(K.data(), {3, 3}, torch::kFloat64).clone().to(torch::kFloat32);

        // Move to configured device
        K_tensor = K_tensor.to(config_.device);

        return K_tensor;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error converting camera intrinsics to tensor: " << e.what();
        return torch::Tensor();
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
torch::Tensor TrainingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::convertCameraPoseToTensor(const Eigen::Isometry3d& pose) {
    try {
        // Convert pose to 4x4 homogeneous transformation matrix
        Eigen::Matrix4d pose_matrix = pose.matrix();

        // Create tensor from pose matrix
        torch::Tensor pose_tensor = torch::from_blob(pose_matrix.data(), {4, 4}, torch::kFloat64)
                                        .clone()
                                        .to(torch::kFloat32);

        // Move to configured device
        pose_tensor = pose_tensor.to(config_.device);

        return pose_tensor;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error converting camera pose to tensor: " << e.what();
        return torch::Tensor();
    }
}

// Note: Rendering and training logic is now handled by the existing BatchTrainer class
// which uses the optimization components (ParameterTransforms, LossFunctions, etc.)

// =============================================================================
// BATCH QUEUE MANAGEMENT METHODS
// =============================================================================

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::enqueueBatchForTraining(uint32_t batch_id) {
    std::lock_guard<std::mutex> lock(batch_queue_mutex_);
    training_batch_queue_.push(batch_id);
    LOG(INFO) << "Enqueued batch " << batch_id << " for training. Queue size: " 
              << training_batch_queue_.size();
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::dequeueBatchForTraining(uint32_t& batch_id) {
    std::lock_guard<std::mutex> lock(batch_queue_mutex_);
    if (training_batch_queue_.empty()) {
        return false;
    }
    
    batch_id = training_batch_queue_.front();
    training_batch_queue_.pop();
    LOG(INFO) << "Dequeued batch " << batch_id << " for training. Queue size: " 
              << training_batch_queue_.size();
    return true;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::hasPendingBatches() const {
    std::lock_guard<std::mutex> lock(batch_queue_mutex_);
    return !training_batch_queue_.empty();
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
size_t TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                      RasterizationT>::getPendingBatchCount() const {
    std::lock_guard<std::mutex> lock(batch_queue_mutex_);
    return training_batch_queue_.size();
}

// =============================================================================
// VISUALIZATION QUEUE MANAGEMENT METHODS
// =============================================================================

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::enqueueInitializedBatch(uint32_t batch_id) {
    std::lock_guard<std::mutex> lock(visualization_queue_mutex_);
    initialized_batches_queue_.push(batch_id);
    LOG(INFO) << "Enqueued initialized batch " << batch_id << " for visualization. Queue size: " 
              << initialized_batches_queue_.size();
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::enqueueTrainedBatch(uint32_t batch_id) {
    std::lock_guard<std::mutex> lock(visualization_queue_mutex_);
    trained_batches_queue_.push(batch_id);
    LOG(INFO) << "Enqueued trained batch " << batch_id << " for visualization. Queue size: " 
              << trained_batches_queue_.size();
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::hasNewInitializedBatches() {
    std::lock_guard<std::mutex> lock(visualization_queue_mutex_);
    size_t current_size = initialized_batches_queue_.size();
    if (current_size != last_initialized_queue_size_) {
        last_initialized_queue_size_ = current_size;
        return true;
    }
    return false;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                    RasterizationT>::hasNewTrainedBatches() {
    std::lock_guard<std::mutex> lock(visualization_queue_mutex_);
    size_t current_size = trained_batches_queue_.size();
    if (current_size != last_trained_queue_size_) {
        last_trained_queue_size_ = current_size;
        return true;
    }
    return false;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
std::vector<uint32_t> TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                                     RasterizationT>::getAllInitializedBatches() const {
    std::lock_guard<std::mutex> lock(visualization_queue_mutex_);
    std::vector<uint32_t> batch_ids;
    
    // Create a copy of the queue to iterate through it
    std::queue<uint32_t> queue_copy = initialized_batches_queue_;
    while (!queue_copy.empty()) {
        batch_ids.push_back(queue_copy.front());
        queue_copy.pop();
    }
    
    return batch_ids;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
std::vector<uint32_t> TrainingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                                     RasterizationT>::getAllTrainedBatches() const {
    std::lock_guard<std::mutex> lock(visualization_queue_mutex_);
    std::vector<uint32_t> batch_ids;
    
    // Create a copy of the queue to iterate through it
    std::queue<uint32_t> queue_copy = trained_batches_queue_;
    while (!queue_copy.empty()) {
        batch_ids.push_back(queue_copy.front());
        queue_copy.pop();
    }
    
    return batch_ids;
}

// Explicit template instantiations for common configurations
template class TrainingGaussianSplatProcessor<
    rendering::BilateralGrid, optimization::DensificationController, training::TrainingConfig,
    rendering::DifferentiableRasterizer>;

}  // namespace gaussian_splatting
