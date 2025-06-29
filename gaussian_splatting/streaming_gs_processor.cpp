#include "streaming_gs_processor.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <random>

namespace gaussian_splatting {

void initializeLogging() {
    google::InitGoogleLogging("training_gs_processor");

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "/logs/streaming_gs_processor_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << ".log";

    FLAGS_alsologtostderr = true;
    FLAGS_logbufsecs = 0;
    FLAGS_log_prefix = true;

    google::SetLogDestination(google::GLOG_INFO, ss.str().c_str());
    google::SetLogDestination(google::GLOG_WARNING, "");
    google::SetLogDestination(google::GLOG_ERROR, "");
    google::SetLogDestination(google::GLOG_FATAL, "");
}

void SlidingWindow::addKeyframe(core::types::KeyFrame::Ptr keyframe) {
    std::lock_guard<std::mutex> lock(window_mutex);

    if (isFull()) {
        removeOldest();
    }

    keyframes.push_back(keyframe);
    keyframe_ids.push_back(keyframe->id);
}

void SlidingWindow::removeOldest() {
    if (!keyframes.empty()) {
        keyframes.pop_front();
        keyframe_ids.pop_front();
    }
}

bool SlidingWindow::isFull() const {
    return keyframes.size() >= max_size;
}

bool SlidingWindow::isEmpty() const {
    return keyframes.empty();
}

size_t SlidingWindow::size() const {
    std::lock_guard<std::mutex> lock(window_mutex);
    return keyframes.size();
}

std::vector<core::types::KeyFrame::Ptr> SlidingWindow::getKeyframes() const {
    std::lock_guard<std::mutex> lock(window_mutex);
    return std::vector<core::types::KeyFrame::Ptr>(keyframes.begin(), keyframes.end());
}

std::vector<uint64_t> SlidingWindow::getKeyframeIds() const {
    std::lock_guard<std::mutex> lock(window_mutex);
    return std::vector<uint64_t>(keyframe_ids.begin(), keyframe_ids.end());
}

void SlidingWindow::clear() {
    std::lock_guard<std::mutex> lock(window_mutex);
    keyframes.clear();
    keyframe_ids.clear();
}

void TrainingStatistics::reset() {
    total_keyframes_processed = 0;
    total_training_iterations = 0;
    current_loss = 0.0;
    best_loss = std::numeric_limits<double>::max();
    current_splat_count = 0;
    converged_splats = 0;
    is_training = false;
    is_converged = false;
    start_time = std::chrono::steady_clock::now();
    last_update_time = start_time;
}

double TrainingStatistics::getElapsedTimeSeconds() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time);
    return duration.count();
}

double TrainingStatistics::getTrainingRate() const {
    auto elapsed = getElapsedTimeSeconds();
    if (elapsed > 0.0) {
        return static_cast<double>(total_training_iterations.load()) / elapsed;
    }
    return 0.0;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                RasterizationT>::StreamingGaussianSplatProcessor(const ConfigType&
                                                                                     config)
    : config_(config), random_generator_(std::random_device{}()), uniform_dist_(0.0, 1.0) {
    initializeLogging();
    LOG(INFO) << "Initializing StreamingGaussianSplatProcessor with the following configuration:";
    LOG(INFO) << "  Device: " << (config_.device.is_cuda() ? "CUDA" : "CPU");
    LOG(INFO) << "  Map Base Path: " << config_.map_base_path;
    LOG(INFO) << "  Max Training Iterations: " << config_.max_training_iterations;
    LOG(INFO) << "  Learning Rate: " << config_.learning_rate;
    LOG(INFO) << "  Sliding Window Size: " << config_.window_config.max_window_size;
    LOG(INFO) << "  Min Window Size: " << config_.window_config.min_window_size;
    LOG(INFO) << "  Initial Splat Count: " << config_.init_config.initial_splat_count;
    LOG(INFO) << "  Visualization Recording ID: " << config_.training_viz_recording_id;

    // Initialize training configuration
    training_config_.initial_width = 512;
    training_config_.initial_height = 384;
    training_config_.max_iterations_per_batch = config_.max_training_iterations;
    training_config_.learning_rate = config_.learning_rate;

    // Initialize sliding window
    sliding_window_ = std::make_unique<SlidingWindow>(config_.window_config.max_window_size);

    LOG(INFO) << "Training configuration set: " << training_config_.initial_width << "x"
              << training_config_.initial_height
              << ", max iterations: " << training_config_.max_iterations_per_batch
              << ", window size: " << config_.window_config.max_window_size;
    LOG(INFO) << "Using shared recording ID: " << config_.training_viz_recording_id;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                RasterizationT>::~StreamingGaussianSplatProcessor() {
    stop();
    LOG(INFO) << "StreamingGaussianSplatProcessor destroyed";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::initialize() {
    LOG(INFO) << "Initializing streaming processor...";

    try {
        map_store_ = std::make_shared<core::storage::MapStore>(config_.map_base_path,
                                                               core::storage::ProcessRole::READER);
        if (!map_store_) {
            LOG(ERROR) << "Invalid MapStore provided";
            return false;
        }

        if (!waitForMapData()) {
            LOG(ERROR) << "Failed to wait for map data";
            return false;
        }

        transform_tree_ = map_store_->getTransformTree();
        if (!transform_tree_) {
            LOG(ERROR) << "Failed to load transform tree";
            return false;
        }
        transform_tree_->printTree();

        // Initializing some gs stuff
        bilateral_grid_ =
            std::make_unique<BilateralGridT>(rendering::BilateralGridConfig{}, config_.device);
        density_controller_ = std::make_unique<DensityControllerT>(training_config_);
        batch_trainer_ = std::make_unique<training::BatchTrainer>(training_config_, map_store_);

        LOG(INFO) << "Initialized all template components successfully";

        // Initialize visualization system
        if (!initializeVisualization()) {
            LOG(WARNING)
                << "Failed to initialize visualization system, continuing without visualization";
            // Don't fail initialization if visualization fails
        }

        // Reset statistics
        stats_.reset();

        LOG(INFO) << "Streaming processor initialized successfully";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to initialize streaming processor: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::start() {
    if (main_thread_running_.load()) {
        LOG(WARNING) << "Streaming processor already running";
        return true;
    }

    if (!map_store_) {
        LOG(ERROR) << "Cannot start: not initialized";
        return false;
    }

    // Reset notification flags
    thread_notification_.should_stop = false;
    thread_notification_.new_keyframe_available = false;
    thread_notification_.training_complete = false;
    thread_notification_.splats_converged = false;

    // Start all threads
    main_thread_running_ = true;
    training_thread_running_ = true;
    visualization_thread_running_ = true;

    main_thread_ =
        std::make_unique<std::thread>(&StreamingGaussianSplatProcessor::mainThreadLoop, this);
    training_thread_ =
        std::make_unique<std::thread>(&StreamingGaussianSplatProcessor::trainingThreadLoop, this);
    visualization_thread_ = std::make_unique<std::thread>(
        &StreamingGaussianSplatProcessor::visualizationThreadLoop, this);

    LOG(INFO) << "Started all threads: main (streaming sync), training, visualization";
    return true;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::stop() {
    if (!main_thread_running_.load()) {
        return;
    }

    LOG(INFO) << "Stopping streaming processor...";

    // Signal all threads to stop
    thread_notification_.should_stop = true;
    main_thread_running_ = false;
    training_thread_running_ = false;
    visualization_thread_running_ = false;

    // Notify waiting threads
    thread_notification_.new_keyframe_cv.notify_all();
    thread_notification_.training_complete_cv.notify_all();
    thread_notification_.convergence_cv.notify_all();

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
    shutdownVisualization();

    LOG(INFO) << "Streaming processor stopped";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::setTrainingProgressCallback(TrainingProgressCallback callback) {
    training_progress_callback_ = callback;
    LOG(INFO) << "Training progress callback set";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::clearTrainingProgressCallback() {
    training_progress_callback_ = nullptr;
    LOG(INFO) << "Training progress callback cleared";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
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

        if (tf_tree_ready && keyframes_ready) {
            LOG(INFO) << "Map data is available (transform tree and keyframes)";
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
double StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                       RasterizationT>::getCurrentTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::logTrainingStats() const {
    LOG(INFO) << "Training Stats - Keyframes: " << stats_.total_keyframes_processed.load()
              << ", Iterations: " << stats_.total_training_iterations.load()
              << ", Loss: " << stats_.current_loss.load()
              << ", Splats: " << stats_.current_splat_count.load()
              << ", Converged: " << stats_.converged_splats.load()
              << ", Training: " << (stats_.is_training.load() ? "YES" : "NO")
              << ", Rate: " << stats_.getTrainingRate() << " it/s";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::notifyThreads() {
    std::lock_guard<std::mutex> lock(thread_notification_.notification_mutex);
    thread_notification_.new_keyframe_available = true;
    thread_notification_.new_keyframe_cv.notify_all();
    LOG(INFO) << "Notified threads of new keyframe";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::mainThreadLoop() {
    LOG(INFO) << "Main thread started: Streaming keyframe sync (polling every "
              << config_.main_loop_interval_ms << "ms)";

    const auto polling_interval = std::chrono::milliseconds(config_.main_loop_interval_ms);

    while (main_thread_running_.load()) {
        LOG(INFO) << "Main thread running at all?";
        try {
            // Step 1: Sync map store and transform tree
            if (!syncMapStoreAndTransformTree()) {
                LOG(WARNING) << "Failed to sync map store and transform tree";
                std::this_thread::sleep_for(polling_interval);
                continue;
            }
            // Step 2: Check for new keyframes from VSLAM
            if (!checkForNewKeyframes()) {
                // No new keyframes, continue polling
                LOG(WARNING) << "Checking new keyframes failed";
                std::this_thread::sleep_for(polling_interval);
                continue;
            }

            if (sliding_window_->size() <= 0) {
                LOG(WARNING) << "Sliding window size less than 0";
                continue;
            }

            LOG(INFO) << "Booya: Sliding window size: " << sliding_window_->size();

            if (stats_.current_splat_count <= 0) {
                if (!initializeRandomSplats()) {
                    LOG(WARNING) << "Unabe to initialize random splats";
                    continue;
                }
            }

            // Step 3: Update sliding window with new keyframes
            if (!updateSlidingWindow()) {
                LOG(ERROR) << "Failed to update sliding window";
                std::this_thread::sleep_for(polling_interval);
                continue;
            }

            // Step 4: Notify training thread of new keyframes
            notifyThreads();

            // Log current status
            if (stats_.total_keyframes_processed.load() % 10 == 0) {
                logTrainingStats();
            }

        } catch (const std::exception& e) {
            LOG(ERROR) << "Exception in main thread loop: " << e.what();
        }

        // Sleep before next polling cycle
        std::this_thread::sleep_for(polling_interval);
    }

    LOG(INFO) << "Main thread ended after processing " << stats_.total_keyframes_processed.load()
              << " keyframes";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::checkForNewKeyframes() {
    if (!shared_memory_) {
        LOG(ERROR) << "Shared memory not initialized";
        return false;
    }

    const auto* region = shared_memory_->getRegion();
    if (!region) {
        LOG(ERROR) << "Shared memory region is null";
        return false;
    }

    // Check if new keyframes are available
    if (!region->header.keyframes_available.load()) {
        return false;  // No keyframes available yet
    }

    // Check for new keyframes since last processed
    uint64_t latest_keyframe_id = region->header.total_keyframes.load();
    uint64_t last_processed = last_processed_keyframe_id_.load();

    if (latest_keyframe_id <= last_processed) {
        return false;  // No new keyframes
    }

    LOG(INFO) << "New keyframes detected: latest=" << latest_keyframe_id
              << ", last_processed=" << last_processed;

    // Get new keyframes from map store
    try {
        auto all_keyframes = map_store_->getAllKeyFrames();

        if (all_keyframes.empty()) {
            LOG(WARNING) << "No keyframes available in map store despite shared memory flag";
            return false;
        }

        // Find keyframes newer than last processed
        std::vector<core::types::KeyFrame::Ptr> new_keyframes;
        for (const auto& kf : all_keyframes) {
            if (kf->id > last_processed) {
                new_keyframes.push_back(kf);
            }
        }

        if (new_keyframes.empty()) {
            LOG(WARNING) << "No new keyframes found despite ID difference";
            return false;
        }

        // Process each new keyframe
        for (const auto& keyframe : new_keyframes) {
            if (!handleNewKeyframe(keyframe)) {
                LOG(ERROR) << "Failed to handle keyframe " << keyframe->id;
                continue;
            }
        }

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception checking for new keyframes: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::syncMapStoreAndTransformTree() {
    if (!map_store_) {
        LOG(ERROR) << "Map store not initialized";
        return false;
    }

    if (!shared_memory_) {
        LOG(ERROR) << "Shared memory not initialized";
        return false;
    }

    try {
        // Check if transform tree is available in shared memory
        const auto* region = shared_memory_->getRegion();
        if (!region) {
            LOG(ERROR) << "Shared memory region is null";
            return false;
        }

        if (!region->header.tf_tree_available.load()) {
            // Transform tree not yet available, but this is not an error during startup
            return true;
        }

        // Sync transform tree from map store
        auto updated_transform_tree = map_store_->getTransformTree();
        if (!updated_transform_tree) {
            LOG(WARNING) << "Failed to get updated transform tree from map store";
            return false;
        }

        // Update our transform tree reference
        transform_tree_ = updated_transform_tree;

        map_store_->syncIndexFromDisk();

        // Optional: Log sync status every 50 cycles to avoid spam
        static int sync_count = 0;
        if (++sync_count % 50 == 0) {
            LOG(INFO) << "Transform tree synced successfully (sync #" << sync_count << ")";
        }

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception syncing map store and transform tree: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::updateSlidingWindow() {
    if (!sliding_window_) {
        LOG(ERROR) << "Sliding window not initialized";
        return false;
    }

    try {
        // The sliding window is already updated by handleNewKeyframe() calls
        // This method serves as a validation and potential cleanup step

        size_t current_window_size = sliding_window_->size();

        // Log window status if it's changed significantly
        static size_t last_logged_size = 0;
        if (current_window_size != last_logged_size) {
            LOG(INFO) << "Sliding window updated: " << current_window_size << "/"
                      << config_.window_config.max_window_size << " keyframes";
            last_logged_size = current_window_size;
        }

        // Check if we have minimum keyframes needed for training
        if (current_window_size >= config_.window_config.min_window_size) {
            // Window is ready for training
            return true;
        } else if (current_window_size > 0) {
            LOG(INFO) << "Sliding window has " << current_window_size << " keyframes, need "
                      << config_.window_config.min_window_size << " minimum for training";
        }

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception updating sliding window: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::handleNewKeyframe(core::types::KeyFrame::Ptr
                                                                            keyframe) {
    if (!keyframe) {
        LOG(ERROR) << "Null keyframe provided";
        return false;
    }

    if (!sliding_window_) {
        LOG(ERROR) << "Sliding window not initialized";
        return false;
    }

    try {
        LOG(INFO) << "Processing new keyframe " << keyframe->id;

        // Validate keyframe has required data
        if (!keyframe->hasCameraInfo()) {
            LOG(WARNING) << "Keyframe " << keyframe->id << " missing camera info";
            // Continue processing - camera info might be available via transform tree
        }

        if (!keyframe->hasColorImage() && !keyframe->hasImage()) {
            LOG(ERROR) << "Keyframe " << keyframe->id << " missing image data";
            return false;
        }

        // Add keyframe to sliding window (this handles overflow automatically)
        sliding_window_->addKeyframe(keyframe);

        // Update statistics
        stats_.total_keyframes_processed.fetch_add(1);
        last_processed_keyframe_id_.store(keyframe->id);
        thread_notification_.latest_keyframe_id.store(keyframe->id);

        LOG(INFO) << "Added keyframe " << keyframe->id << " to sliding window. "
                  << "Window size: " << sliding_window_->size() << "/"
                  << config_.window_config.max_window_size;

        // Check if we should trigger training (train on every new keyframe as specified)
        if (sliding_window_->size() >= config_.window_config.min_window_size) {
            // Mark that new keyframe is available for training
            std::lock_guard<std::mutex> lock(thread_notification_.notification_mutex);
            thread_notification_.new_keyframe_available = true;
            LOG(INFO) << "Keyframe " << keyframe->id << " ready for training "
                      << "(window has " << sliding_window_->size() << " keyframes)";
        }

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception handling keyframe " << keyframe->id << ": " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::extractImageTensor(const core::types::KeyFrame::Ptr& keyframe,
                                        torch::Tensor& image_tensor,
                                        core::types::CameraInfo& camera_info) {
    if (!keyframe) {
        LOG(ERROR) << "Null keyframe provided for image extraction";
        return false;
    }

    try {
        // Extract camera info from keyframe or use default
        if (keyframe->hasCameraInfo()) {
            camera_info = keyframe->getCameraInfo();
        } else {
            LOG(WARNING) << "Keyframe " << keyframe->id << " missing camera info, using defaults";
            // Use default camera info - this should be improved in production
            camera_info.width = training_config_.initial_width;
            camera_info.height = training_config_.initial_height;
            // Create default camera matrix K with standard values
            camera_info.k.resize(9);
            camera_info.k[0] = 500.0;  // fx
            camera_info.k[1] = 0.0;
            camera_info.k[2] = camera_info.width / 2.0;  // cx
            camera_info.k[3] = 0.0;
            camera_info.k[4] = 500.0;                     // fy
            camera_info.k[5] = camera_info.height / 2.0;  // cy
            camera_info.k[6] = 0.0;
            camera_info.k[7] = 0.0;
            camera_info.k[8] = 1.0;
        }

        // Extract image data
        cv::Mat image_cv;
        if (keyframe->hasColorImage()) {
            // Prefer color image
            const auto& color_image = keyframe->getColorImage();
            image_cv = color_image.data.clone();
        } else if (keyframe->hasImage()) {
            // Fall back to depth/grayscale image
            const auto& depth_image = keyframe->getImage();
            image_cv = depth_image.data.clone();

            // Convert single channel to 3-channel if needed
            if (image_cv.channels() == 1) {
                cv::cvtColor(image_cv, image_cv, cv::COLOR_GRAY2BGR);
            }
        } else {
            LOG(ERROR) << "Keyframe " << keyframe->id << " has no image data";
            return false;
        }

        // Resize image to training resolution
        if (image_cv.cols != camera_info.width || image_cv.rows != camera_info.height) {
            cv::resize(image_cv, image_cv, cv::Size(camera_info.width, camera_info.height));
        }

        // Convert to torch tensor [C, H, W] format
        if (image_cv.type() != CV_32FC3) {
            image_cv.convertTo(image_cv, CV_32FC3, 1.0 / 255.0);
        }

        image_tensor =
            torch::from_blob(image_cv.data, {image_cv.rows, image_cv.cols, 3}, torch::kFloat32);
        image_tensor = image_tensor.permute({2, 0, 1});  // HWC -> CHW
        image_tensor = image_tensor.to(config_.device);

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception extracting image tensor from keyframe " << keyframe->id << ": "
                   << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::extractCameraPoses(const std::vector<uint64_t>& keyframe_ids,
                                        std::vector<Eigen::Isometry3d>& camera_poses,
                                        const std::string& target_frame) {
    if (!transform_tree_) {
        LOG(ERROR) << "Transform tree not initialized for camera pose extraction";
        return false;
    }

    camera_poses.clear();
    camera_poses.reserve(keyframe_ids.size());

    try {
        for (uint64_t keyframe_id : keyframe_ids) {
            // Get keyframe from sliding window
            auto keyframes = sliding_window_->getKeyframes();
            core::types::KeyFrame::Ptr target_keyframe = nullptr;

            for (const auto& kf : keyframes) {
                if (kf->id == keyframe_id) {
                    target_keyframe = kf;
                    break;
                }
            }

            if (!target_keyframe) {
                LOG(ERROR) << "Keyframe " << keyframe_id << " not found in sliding window";
                return false;
            }

            // Get camera pose in target frame (usually "odom")
            Eigen::Isometry3d camera_pose;

            // Try to get base_link to camera transform from transform tree
            std::string camera_frame = "camera_color_optical_frame";  // Default camera frame

            // Get pose from keyframe (base_link in odom frame)
            Eigen::Isometry3d base_pose = target_keyframe->pose.getEigenIsometry();

            // Get camera transform relative to base_link
            Eigen::Isometry3d base_to_camera = Eigen::Isometry3d::Identity();
            try {
                auto transform_result = transform_tree_->getTransform("base_link", camera_frame);
                base_to_camera = transform_result.transform;
            } catch (const std::exception& e) {
                LOG(WARNING) << "Failed to get base_link to camera transform: " << e.what()
                             << ", using identity";
            }

            // Compose transforms: camera_pose = odom_T_base * base_T_camera
            camera_pose = base_pose * base_to_camera;

            camera_poses.push_back(camera_pose);
        }

        LOG(INFO) << "Extracted " << camera_poses.size() << " camera poses for keyframes";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception extracting camera poses: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
torch::Tensor StreamingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::convertCameraIntrinsicsToTensor(const core::types::CameraInfo& camera_info) {
    // Create 3x3 camera intrinsics matrix
    torch::Tensor intrinsics =
        torch::zeros({3, 3}, torch::TensorOptions().dtype(torch::kFloat32).device(config_.device));

    // Extract from camera matrix K [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    if (camera_info.k.size() >= 9) {
        intrinsics[0][0] = camera_info.k[0];  // fx
        intrinsics[1][1] = camera_info.k[4];  // fy
        intrinsics[0][2] = camera_info.k[2];  // cx
        intrinsics[1][2] = camera_info.k[5];  // cy
    } else {
        // Fallback defaults
        intrinsics[0][0] = 500.0;  // fx
        intrinsics[1][1] = 500.0;  // fy
        intrinsics[0][2] = 320.0;  // cx
        intrinsics[1][2] = 240.0;  // cy
    }
    intrinsics[2][2] = 1.0;  // homogeneous coordinate

    return intrinsics;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
torch::Tensor StreamingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::convertCameraPoseToTensor(const Eigen::Isometry3d& pose) {
    // Convert 4x4 pose matrix to torch tensor
    torch::Tensor pose_tensor =
        torch::zeros({4, 4}, torch::TensorOptions().dtype(torch::kFloat32).device(config_.device));

    Eigen::Matrix4d pose_matrix = pose.matrix();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            pose_tensor[i][j] = static_cast<float>(pose_matrix(i, j));
        }
    }

    return pose_tensor;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::trainingThreadLoop() {
    LOG(INFO) << "Training thread started (polling every " << config_.training_loop_interval_ms
              << "ms)";

    const auto polling_interval = std::chrono::milliseconds(config_.training_loop_interval_ms);

    while (training_thread_running_.load()) {
        try {
            // Wait for notification from main thread or timeout
            std::unique_lock<std::mutex> lock(thread_notification_.notification_mutex);

            // Wait for new keyframe notification or timeout
            if (!thread_notification_.new_keyframe_cv.wait_for(lock, polling_interval, [this] {
                    return thread_notification_.new_keyframe_available.load() ||
                           thread_notification_.should_stop.load();
                })) {
                // Timeout - continue polling
                continue;
            }

            // Check if we should stop
            if (thread_notification_.should_stop.load()) {
                break;
            }

            // Check if we have new keyframes to train on
            if (!thread_notification_.new_keyframe_available.load()) {
                continue;
            }

            // Reset the flag before processing
            thread_notification_.new_keyframe_available = false;
            lock.unlock();

            // Check if sliding window has enough keyframes for training
            if (sliding_window_->size() < config_.window_config.min_window_size) {
                LOG(INFO) << "Sliding window has " << sliding_window_->size() << " keyframes, need "
                          << config_.window_config.min_window_size << " minimum for training";
                continue;
            }

            // Train on current sliding window
            if (!trainOnCurrentWindow()) {
                LOG(ERROR) << "Failed to train on current sliding window";
                continue;
            }

            // Check convergence status
            if (checkConvergence()) {
                LOG(INFO) << "Training converged for current window";
                thread_notification_.splats_converged = true;
                thread_notification_.convergence_cv.notify_all();
            }

            // Update training statistics
            stats_.is_training = false;
            thread_notification_.training_complete = true;
            thread_notification_.training_complete_cv.notify_all();

            LOG(INFO) << "Training iteration completed. Window size: " << sliding_window_->size()
                      << " keyframes";

        } catch (const std::exception& e) {
            LOG(ERROR) << "Exception in training thread loop: " << e.what();
            stats_.is_training = false;
        }
    }

    LOG(INFO) << "Training thread ended after " << stats_.total_training_iterations.load()
              << " iterations";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::trainOnCurrentWindow() {
    if (!batch_trainer_) {
        LOG(ERROR) << "BatchTrainer not initialized";
        return false;
    }

    if (!sliding_window_) {
        LOG(ERROR) << "Sliding window not initialized";
        return false;
    }

    try {
        LOG(INFO) << "Training on sliding window with " << sliding_window_->size() << " keyframes";

        // Set training flag
        stats_.is_training = true;

        // Load current sliding window into training batch format
        training::KeyframeBatch keyframe_batch;
        if (!loadWindowForTraining(keyframe_batch)) {
            LOG(ERROR) << "Failed to load sliding window for training";
            stats_.is_training = false;
            return false;
        }

        // Execute incremental training using BatchTrainer
        if (!executeIncrementalTraining(keyframe_batch)) {
            LOG(ERROR) << "Failed to execute incremental training";
            stats_.is_training = false;
            return false;
        }

        LOG(INFO) << "Successfully completed training on sliding window";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in trainOnCurrentWindow: " << e.what();
        stats_.is_training = false;
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::loadWindowForTraining(training::KeyframeBatch&
                                                                                keyframe_batch) {
    LOG(INFO) << "Loading the window into a keyframe batch";
    if (!sliding_window_) {
        LOG(ERROR) << "Sliding window not initialized";
        return false;
    }

    try {
        // Get current keyframes from sliding window
        auto keyframes = sliding_window_->getKeyframes();
        auto keyframe_ids = sliding_window_->getKeyframeIds();

        if (keyframes.empty()) {
            LOG(ERROR) << "No keyframes in sliding window";
            return false;
        }

        // Extract image dimensions from first keyframe
        core::types::CameraInfo first_camera_info;
        torch::Tensor first_image_tensor;
        if (!extractImageTensor(keyframes[0], first_image_tensor, first_camera_info)) {
            LOG(ERROR) << "Failed to extract image tensor from first keyframe";
            return false;
        }

        // Prepare tensors for all keyframes
        std::vector<torch::Tensor> image_tensors;
        std::vector<torch::Tensor> pose_tensors;
        std::vector<torch::Tensor> intrinsic_tensors;

        image_tensors.reserve(keyframes.size());
        pose_tensors.reserve(keyframes.size());
        intrinsic_tensors.reserve(keyframes.size());

        // Extract camera poses for all keyframes
        std::vector<Eigen::Isometry3d> camera_poses;
        if (!extractCameraPoses(keyframe_ids, camera_poses)) {
            LOG(ERROR) << "Failed to extract camera poses";
            return false;
        }

        LOG(INFO) << "Keyframes camera poses extracted: " << camera_poses.size();
        // Process each keyframe
        for (size_t i = 0; i < keyframes.size(); ++i) {
            // Extract image tensor and camera info
            torch::Tensor image_tensor;
            core::types::CameraInfo camera_info;
            if (!extractImageTensor(keyframes[i], image_tensor, camera_info)) {
                LOG(ERROR) << "Failed to extract image tensor from keyframe " << keyframes[i]->id;
                continue;
            }

            // Dumber doing dumb things
            keyframe_batch.image_height = camera_info.height;
            keyframe_batch.image_width = camera_info.width;

            // Convert camera intrinsics to tensor
            torch::Tensor intrinsics_tensor = convertCameraIntrinsicsToTensor(camera_info);

            // Convert camera pose to tensor
            torch::Tensor pose_tensor = convertCameraPoseToTensor(camera_poses[i]);

            keyframe_batch.keyframe_ids.push_back(keyframes[i]->id);
            keyframe_batch.keyframes.push_back(keyframes[i]);

            image_tensors.push_back(image_tensor);
            intrinsic_tensors.push_back(intrinsics_tensor);
            pose_tensors.push_back(pose_tensor);
        }

        LOG(INFO) << "POse: " << pose_tensors.size()
                  << " intrinsic_tensors: " << intrinsic_tensors.size()
                  << " image_tensors: " << image_tensors.size();

        assert(image_tensors.size() == intrinsic_tensors.size());
        assert(image_tensors.size() == pose_tensors.size());
        // Clear and initialize the batch
        keyframe_batch.clear();
        keyframe_batch.batch_id = static_cast<uint32_t>(stats_.total_keyframes_processed.load());

        keyframe_batch.batch_size = image_tensors.size();
        keyframe_batch.device = config_.device;

        // Stack individual tensors into batch tensors
        keyframe_batch.images = torch::stack(image_tensors, 0);                 // [N, 3, H, W]
        keyframe_batch.camera_intrinsics = torch::stack(intrinsic_tensors, 0);  // [N, 3, 3]
        keyframe_batch.camera_poses = torch::stack(pose_tensors, 0);            // [N, 4, 4]

        LOG(INFO) << "Printing stuff";
        keyframe_batch.print();

        // Move to target device
        keyframe_batch.to(config_.device);

        LOG(INFO) << "Loaded " << keyframe_batch.batch_size << " keyframes into training batch. "
                  << "Image size: " << keyframe_batch.image_width << "x"
                  << keyframe_batch.image_height;

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in loadWindowForTraining: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::executeIncrementalTraining(const training::KeyframeBatch& keyframe_batch) {
    if (!batch_trainer_) {
        LOG(ERROR) << "BatchTrainer not initialized";
        return false;
    }

    try {
        LOG(INFO) << "Executing incremental training with " << keyframe_batch.batch_size
                  << " keyframes";

        // Get the current Gaussian splat batch from map store
        // We need to train on the existing splats with the new keyframes
        auto current_splat_batches = map_store_->getAllGaussianSplatBatches();
        if (current_splat_batches.empty()) {
            LOG(WARNING) << "No existing splat batches found, creating initial batch";
            // This should not happen since we initialized random splats
            return false;
        }

        // Use the most recent splat batch for training
        auto latest_batch = current_splat_batches.back();

        // Setup multi-view rendering with current sliding window keyframes
        batch_trainer_->setupMultiViewRendering(keyframe_batch.keyframe_ids);

        // Setup the splat batch for training
        if (!batch_trainer_->setupBatchForTraining(latest_batch)) {
            LOG(ERROR) << "Failed to setup splat batch for training";
            return false;
        }

        // Perform training iterations
        training::TrainingResults results;
        results.success = false;

        int completed_iterations = 0;
        for (int iter = 0; iter < config_.max_training_iterations; ++iter) {
            if (!batch_trainer_->performTrainingStep(iter)) {
                LOG(ERROR) << "Training step " << iter << " failed";
                break;
            }

            completed_iterations = iter + 1;

            // Update training statistics
            stats_.total_training_iterations.fetch_add(1);
            thread_notification_.current_training_iteration = iter;

            // Update training statistics callback (would need actual loss from BatchTrainer)
            // For now using placeholder values - in a real implementation,
            // BatchTrainer would provide actual loss values
            handleTrainingStatsCallback(iter, 0.0f, 0.0f, 0.0f, latest_batch.splats.size());

            // Log progress periodically
            if (iter % 50 == 0) {
                LOG(INFO) << "Training iteration " << iter << " completed";
            }

            // Check convergence periodically
            if (iter % config_.convergence_config.convergence_check_interval == 0) {
                if (checkConvergence()) {
                    LOG(INFO) << "Training converged at iteration " << iter;
                    break;
                }
            }
        }

        results.iterations_completed = completed_iterations;
        results.success = true;

        // Update splat count
        stats_.current_splat_count = latest_batch.splats.size();

        // Save trained splats back to map store
        if (!map_store_->writeSplatBatchToDisk(latest_batch.batch_id)) {
            LOG(WARNING) << "Failed to write trained splat batch to disk";
        }

        LOG(INFO) << "Incremental training completed: " << completed_iterations << " iterations, "
                  << stats_.current_splat_count.load() << " splats";

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in executeIncrementalTraining: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::checkConvergence() {
    if (!config_.convergence_config.enable_early_stopping) {
        return false;  // Early stopping disabled
    }

    try {
        // Get current splat batches to check convergence
        auto current_splat_batches = map_store_->getAllGaussianSplatBatches();
        if (current_splat_batches.empty()) {
            return false;  // No splats to check
        }

        auto latest_batch = current_splat_batches.back();

        // Check covariance convergence
        int converged_splats = 0;
        int total_splats = latest_batch.splats.size();

        for (const auto& splat : latest_batch.splats) {
            // Check if splat covariance is below threshold
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(splat.covariance);
            double max_eigenvalue = solver.eigenvalues().maxCoeff();

            if (max_eigenvalue < config_.convergence_config.covariance_threshold) {
                converged_splats++;
            }

            // Check opacity threshold (remove splats that are too transparent)
            if (splat.opacity < config_.convergence_config.min_opacity_threshold) {
                converged_splats++;  // Consider low-opacity splats as "converged" (to be pruned)
            }
        }

        // Update converged splats count
        stats_.converged_splats = converged_splats;

        // Calculate convergence ratio
        double convergence_ratio =
            total_splats > 0 ? static_cast<double>(converged_splats) / total_splats : 0.0;

        // Check loss convergence (simplified - would need actual loss tracking)
        double current_loss = stats_.current_loss.load();
        double best_loss = stats_.best_loss.load();

        bool loss_converged = false;
        if (current_loss > 0.0 && best_loss < std::numeric_limits<double>::max()) {
            double loss_improvement = (best_loss - current_loss) / best_loss;
            loss_converged =
                loss_improvement < config_.convergence_config.loss_convergence_threshold;
        }

        // Update best loss if current is better
        if (current_loss > 0.0 && current_loss < best_loss) {
            stats_.best_loss = current_loss;
        }

        // Check iteration limit per splat
        double avg_iterations_per_splat =
            total_splats > 0
                ? static_cast<double>(stats_.total_training_iterations.load()) / total_splats
                : 0.0;
        bool iteration_limit_reached =
            avg_iterations_per_splat >= config_.convergence_config.max_iterations_per_splat;

        // Determine convergence
        bool converged = false;

        if (convergence_ratio >= 0.95) {  // 95% of splats converged
            LOG(INFO) << "Convergence achieved: " << convergence_ratio * 100.0
                      << "% splats converged";
            converged = true;
        } else if (loss_converged && convergence_ratio >= 0.8) {  // 80% splats + loss converged
            LOG(INFO) << "Convergence achieved: loss stabilized and " << convergence_ratio * 100.0
                      << "% splats converged";
            converged = true;
        } else if (iteration_limit_reached) {
            LOG(INFO) << "Convergence by iteration limit: " << avg_iterations_per_splat
                      << " avg iterations per splat";
            converged = true;
        }

        if (converged) {
            stats_.is_converged = true;
            LOG(INFO) << "Training convergence detected: " << converged_splats << "/"
                      << total_splats << " splats converged, current loss: " << current_loss;
        }

        return converged;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in checkConvergence: " << e.what();
        return false;
    }
}

// =============================================================================
// Visualization and Callbacks
// =============================================================================

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::handleTrainingStatsCallback(int iteration,
                                                                                  float total_loss,
                                                                                  float l1_loss,
                                                                                  float d_ssim_loss,
                                                                                  int splat_count) {
    try {
        // Update training statistics
        stats_.current_loss = total_loss;
        stats_.last_update_time = std::chrono::steady_clock::now();
        thread_notification_.current_training_iteration = iteration;
        thread_notification_.current_loss = total_loss;

        // Update best loss if this is better
        if (total_loss > 0.0 && total_loss < stats_.best_loss.load()) {
            stats_.best_loss = total_loss;
        }

        // Log detailed training statistics
        LOG(INFO) << "Training iteration " << iteration << " - Total loss: " << total_loss
                  << ", L1 loss: " << l1_loss << ", D-SSIM loss: " << d_ssim_loss
                  << ", Splats: " << splat_count;

        // Call user-provided progress callback if set
        if (training_progress_callback_) {
            training_progress_callback_(iteration, total_loss, splat_count);
        }

        // Update visualization with current training stats
        if (training_visualizer_) {
            // Update loss curves with real training data
            training_visualizer_->updateLossCurves(total_loss, l1_loss, d_ssim_loss, iteration);

            // Create training metrics for detailed logging
            visualization::TrainingMetrics metrics;
            metrics.epoch = 0;  // Streaming doesn't use epochs
            metrics.batch_in_epoch = iteration;
            metrics.total_batches_processed = iteration;
            metrics.total_loss = total_loss;
            metrics.l1_loss = l1_loss;
            metrics.ssim_loss = d_ssim_loss;
            metrics.learning_rate = config_.learning_rate;
            metrics.batch_processing_time_ms = 0.0;  // Would need to measure this
            metrics.timestamp = std::chrono::steady_clock::now();
            metrics.num_splats = splat_count;
            metrics.num_keyframes_in_batch = sliding_window_ ? sliding_window_->size() : 0;

            // Log the training metrics
            training_visualizer_->logTrainingMetrics(metrics);

            // Update learning rate visualization
            training_visualizer_->visualizeLearningRate(config_.learning_rate, iteration);
        }

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception in handleTrainingStatsCallback: " << e.what();
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::visualizationThreadLoop() {
    LOG(INFO) << "Visualization thread started (updating every "
              << config_.visualization_loop_interval_ms << "ms)";

    const auto update_interval = std::chrono::milliseconds(config_.visualization_loop_interval_ms);

    while (visualization_thread_running_.load()) {
        try {
            // Visualize current training state
            bool training_active = stats_.is_training.load();
            bool has_new_data = stats_.total_training_iterations.load() > 0;

            if (training_active || has_new_data) {
                // Update current splats visualization
                if (!visualizeCurrentSplats()) {
                    LOG(WARNING) << "Failed to visualize current splats";
                }

                // Update sliding window visualization
                if (!visualizeSlidingWindow()) {
                    LOG(WARNING) << "Failed to visualize sliding window";
                }

                // Log visualization update
                static int viz_update_count = 0;
                if (++viz_update_count % 10 == 0) {
                    LOG(INFO) << "Visualization updated (#" << viz_update_count
                              << ") - Training: " << (training_active ? "ACTIVE" : "IDLE")
                              << ", Iterations: " << stats_.total_training_iterations.load()
                              << ", Splats: " << stats_.current_splat_count.load();
                }
            }

            // Check for training completion notifications
            std::unique_lock<std::mutex> lock(thread_notification_.notification_mutex);
            if (thread_notification_.training_complete_cv.wait_for(lock, update_interval, [this] {
                    return thread_notification_.training_complete.load() ||
                           thread_notification_.should_stop.load();
                })) {
                if (thread_notification_.should_stop.load()) {
                    break;
                }

                if (thread_notification_.training_complete.load()) {
                    LOG(INFO) << "Training completed, updating final visualization";
                    // Reset the flag
                    thread_notification_.training_complete = false;

                    // Update visualization with final results
                    visualizeCurrentSplats();
                    visualizeSlidingWindow();
                }
            }

        } catch (const std::exception& e) {
            LOG(ERROR) << "Exception in visualization thread loop: " << e.what();
        }

        // Sleep if no notification received
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    LOG(INFO) << "Visualization thread ended after " << stats_.total_training_iterations.load()
              << " training iterations";
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::initializeRandomSplats() {
    LOG(INFO) << "Initializing random splats...";

    try {
        // Estimate scene bounds from camera trajectory
        auto [scene_min, scene_max] = estimateSceneBoundsFromTrajectory();

        LOG(INFO) << "Estimated scene bounds: min(" << scene_min.transpose() << ") max("
                  << scene_max.transpose() << ")";

        // Generate random splats within scene bounds
        int splat_count = config_.init_config.initial_splat_count;
        if (config_.init_config.adaptive_density) {
            // Adjust splat count based on scene volume
            float scene_volume = (scene_max - scene_min).prod();
            splat_count = static_cast<int>(splat_count * config_.init_config.density_scale_factor *
                                           std::min(scene_volume / 1000.0f, 2.0f));
        }

        auto random_splats = generateRandomSplats(scene_min, scene_max, splat_count);

        LOG(INFO) << "Generated " << random_splats.size() << " random splats";

        // Create initial splat batch
        core::types::GaussianSplatBatch initial_batch;
        initial_batch.batch_id = 0;  // Initial batch
        initial_batch.timestamp = getCurrentTimestamp();
        initial_batch.splats = random_splats;

        // Write initial splats to map store
        if (!map_store_->addGaussianSplatBatch(initial_batch)) {
            LOG(ERROR) << "Failed to add initial splat batch to map store";
            return false;
        }

        if (!map_store_->writeSplatBatchToDisk(initial_batch.batch_id)) {
            LOG(ERROR) << "Failed to write initial splat batch to disk";
            return false;
        }

        // Update statistics
        stats_.current_splat_count = random_splats.size();
        next_splat_id_ = random_splats.size() + 1;

        LOG(INFO) << "Successfully initialized " << random_splats.size()
                  << " random splats and wrote to disk";
        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Error initializing random splats: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::initializeVisualization() {
    LOG(INFO) << "Initializing visualization system...";

    try {
        // Create RerunTrainingVisualizer with config parameters
        training_visualizer_ = std::make_unique<visualization::RerunTrainingVisualizer>(
            config_.training_viz_recording_id, config_.training_viz_host,
            config_.training_viz_port);

        if (!training_visualizer_) {
            LOG(ERROR) << "Failed to create RerunTrainingVisualizer";
            return false;
        }

        // Initialize the visualizer
        if (!training_visualizer_->initialize()) {
            LOG(ERROR) << "Failed to initialize RerunTrainingVisualizer";
            training_visualizer_.reset();
            return false;
        }

        // Configure visualization settings
        training_visualizer_->enableSplatVisualization(true);
        training_visualizer_->enableBatchVisualization(true);
        training_visualizer_->setVisualizationFrequency(1);  // Update every iteration

        // Set initial training state
        training_visualizer_->visualizeTrainingState("INITIALIZING",
                                                     "Setting up streaming processor");

        LOG(INFO) << "Visualization system initialized successfully";
        LOG(INFO) << "Recording ID: " << config_.training_viz_recording_id;
        LOG(INFO) << "Visualization host: " << config_.training_viz_host << ":"
                  << config_.training_viz_port;

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception initializing visualization: " << e.what();
        training_visualizer_.reset();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::visualizeCurrentSplats() {
    if (!training_visualizer_) {
        return false;  // Visualization not initialized
    }

    try {
        // Get current splat batches from map store
        auto current_splat_batches = map_store_->getAllGaussianSplatBatches();
        if (current_splat_batches.empty()) {
            return true;  // No splats to visualize yet
        }

        // Use the most recent splat batch
        auto latest_batch = current_splat_batches.back();

        if (latest_batch.splats.empty()) {
            return true;  // No splats in batch
        }

        // Get current training iteration for visualization context
        uint32_t current_iteration = static_cast<uint32_t>(stats_.total_training_iterations.load());

        // Visualize the current splats
        training_visualizer_->visualizeCurrentSplats(latest_batch.splats, current_iteration);

        // Update training state visualization
        if (stats_.is_training.load()) {
            training_visualizer_->visualizeTrainingState(
                "TRAINING", "Iteration " + std::to_string(current_iteration) +
                                ", Splats: " + std::to_string(latest_batch.splats.size()));
        } else {
            training_visualizer_->visualizeTrainingState(
                "IDLE", "Window size: " + std::to_string(sliding_window_->size()) + " keyframes");
        }

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception visualizing current splats: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
bool StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::visualizeSlidingWindow() {
    if (!training_visualizer_) {
        return false;  // Visualization not initialized
    }

    if (!sliding_window_) {
        return false;  // Sliding window not initialized
    }

    try {
        // Get current keyframes from sliding window
        auto keyframes = sliding_window_->getKeyframes();
        auto keyframe_ids = sliding_window_->getKeyframeIds();

        if (keyframes.empty()) {
            return true;  // No keyframes to visualize
        }

        // Create a training batch for visualization
        training::KeyframeBatch viz_batch;
        if (!loadWindowForTraining(viz_batch)) {
            LOG(WARNING) << "Failed to load sliding window for visualization";
            return false;
        }

        // Visualize the keyframe batch (sliding window)
        training_visualizer_->visualizeKeyframeBatch(viz_batch, "sliding_window");

        LOG(INFO) << "Visualized sliding window with " << keyframes.size() << " keyframes";

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception visualizing sliding window: " << e.what();
        return false;
    }
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
void StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                     RasterizationT>::shutdownVisualization() {
    LOG(INFO) << "Shutting down visualization...";

    try {
        if (training_visualizer_) {
            // Set final training state
            training_visualizer_->visualizeTrainingState(
                "SHUTDOWN", "Training completed with " +
                                std::to_string(stats_.total_training_iterations.load()) +
                                " iterations");

            // Shutdown the visualizer
            training_visualizer_->shutdown();
            training_visualizer_.reset();

            LOG(INFO) << "Visualization system shutdown complete";
        }

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception during visualization shutdown: " << e.what();
    }
}

// =============================================================================
// Random Splat Initialization Methods
// =============================================================================

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
std::pair<Eigen::Vector3f, Eigen::Vector3f>
StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                RasterizationT>::estimateSceneBoundsFromTrajectory() {
    Eigen::Vector3f scene_min = config_.scene_min;
    Eigen::Vector3f scene_max = config_.scene_max;

    try {
        // Get existing keyframes to estimate trajectory bounds
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
            Eigen::Vector3f padding(config_.init_config.scene_bounds_padding,
                                    config_.init_config.scene_bounds_padding,
                                    config_.init_config.scene_bounds_padding);
            scene_min -= padding;
            scene_max += padding;

            LOG(INFO) << "Estimated scene bounds from trajectory: min(" << scene_min.transpose()
                      << ") max(" << scene_max.transpose() << ")";
        } else {
            LOG(INFO) << "No keyframes available, using config scene bounds";
        }

    } catch (const std::exception& e) {
        LOG(WARNING) << "Failed to estimate scene bounds from trajectory: " << e.what()
                     << ", using config bounds";
    }

    return std::make_pair(scene_min, scene_max);
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
std::vector<core::types::GaussianSplat> StreamingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::generateRandomSplats(const Eigen::Vector3f& scene_min,
                                          const Eigen::Vector3f& scene_max, int count) {
    std::vector<core::types::GaussianSplat> splats;
    splats.reserve(count);

    LOG(INFO) << "Generating " << count << " random splats in bounds: min(" << scene_min.transpose()
              << ") max(" << scene_max.transpose() << ")";

    for (int i = 0; i < count; ++i) {
        core::types::GaussianSplat splat;

        // Generate unique ID
        splat.id = next_splat_id_ + i;

        // Random position within scene bounds
        splat.position = generateRandomPosition(scene_min, scene_max).template cast<double>();

        // Random initial color
        splat.color = generateRandomColor();

        // Initial covariance matrix
        splat.covariance = generateInitialCovariance();

        // Initial opacity
        splat.opacity = generateInitialOpacity();

        // Initial confidence (high for random initialization)
        splat.confidence = 0.5f;  // Medium confidence initially

        // Timestamp
        splat.timestamp = getCurrentTimestamp();

        // No source keypoint for random initialization
        splat.source_keypoint_id = 0;

        splats.push_back(splat);
    }

    return splats;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
Eigen::Vector3f StreamingGaussianSplatProcessor<
    BilateralGridT, DensityControllerT, TrainingConfigT,
    RasterizationT>::generateRandomPosition(const Eigen::Vector3f& min_bounds,
                                            const Eigen::Vector3f& max_bounds) {
    Eigen::Vector3f position;
    for (int i = 0; i < 3; ++i) {
        position[i] =
            min_bounds[i] + uniform_dist_(random_generator_) * (max_bounds[i] - min_bounds[i]);
    }
    return position;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
Eigen::Vector3f StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                                RasterizationT>::generateRandomColor() {
    // Generate neutral colors with some variation
    float base_brightness = 0.5f + uniform_dist_(random_generator_) * 0.3f;  // 0.5-0.8
    float color_variation = 0.1f;

    Eigen::Vector3f color;
    color[0] =
        std::clamp(static_cast<float>(base_brightness +
                                      (uniform_dist_(random_generator_) - 0.5) * color_variation),
                   0.0f, 1.0f);
    color[1] =
        std::clamp(static_cast<float>(base_brightness +
                                      (uniform_dist_(random_generator_) - 0.5) * color_variation),
                   0.0f, 1.0f);
    color[2] =
        std::clamp(static_cast<float>(base_brightness +
                                      (uniform_dist_(random_generator_) - 0.5) * color_variation),
                   0.0f, 1.0f);

    return color;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
Eigen::Matrix3d StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                                RasterizationT>::generateInitialCovariance() {
    // Start with isotropic covariance scaled by config
    double initial_variance =
        config_.init_config.initial_covariance_scale * config_.init_config.initial_covariance_scale;

    // Add small random perturbation to break symmetry
    double variance_perturbation = uniform_dist_(random_generator_) * 0.1 * initial_variance;

    Eigen::Matrix3d covariance =
        Eigen::Matrix3d::Identity() * (initial_variance + variance_perturbation);

    // Add small random rotation to avoid perfect alignment
    double angle = uniform_dist_(random_generator_) * 0.1;  // Small rotation
    Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::Random().normalized());
    Eigen::Matrix3d R = rotation.toRotationMatrix();

    // Apply rotation: C' = R * C * R^T
    covariance = R * covariance * R.transpose();

    return covariance;
}

template <typename BilateralGridT, typename DensityControllerT, typename TrainingConfigT,
          typename RasterizationT>
float StreamingGaussianSplatProcessor<BilateralGridT, DensityControllerT, TrainingConfigT,
                                      RasterizationT>::generateInitialOpacity() {
    // Random opacity within configured range
    float min_opacity = config_.init_config.initial_opacity_range_min;
    float max_opacity = config_.init_config.initial_opacity_range_max;

    return min_opacity + uniform_dist_(random_generator_) * (max_opacity - min_opacity);
}

// Explicit template instantiations for common configurations
template class StreamingGaussianSplatProcessor<
    rendering::BilateralGrid, optimization::DensificationController, training::TrainingConfig,
    rendering::DifferentiableRasterizer>;

}  // namespace gaussian_splatting
