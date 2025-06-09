#include "core/graph/graph_adapter.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include "core/graph/util.hpp"
#include "logging/logging.hpp"

namespace core {
namespace graph {

GraphAdapter::GraphAdapter(FactorGraph& graph, storage::MapStore& store)
    : graph_(graph),
      store_(store),
      keyframe_manager_(KeyframeThresholds{}),
      synchronizer_(
          [this](
              const types::Pose& pose, const std::optional<types::Image>& image,
              const std::optional<types::CameraInfo>& camera_info,
              const std::optional<types::ImuData>& imu_data) -> std::shared_ptr<types::KeyFrame> {
              return this->createKeyframe(pose, image, camera_info, std::nullopt, imu_data);
          },
          1.0),  // Only time threshold - KeyframeManager handles distance decisions
      cumulative_distance_(0.0),
      next_dump_distance_(1.0),
      imu_preintegrator_(std::make_unique<SimpleImuPreintegrator>()),
      bias_initialized_(false),
      bias_estimation_window_(100),
      optimization_keyframe_interval_(10) {  // Set to 10 keyframes as specified

    LOG(INFO) << "GraphAdapter initialized with callback-based MapStore write system (optimization every "
              << optimization_keyframe_interval_ << " keyframes)";
}

// Destructor - ensure optimization thread is properly stopped
GraphAdapter::~GraphAdapter() {
    LOG(INFO) << "GraphAdapter destructor: stopping optimization thread";
    stopOptimizationThread();

    LOG(INFO) << "GraphAdapter destructor completed (callback-based write system)";
}

// Modify input handlers to use synchronizer
void GraphAdapter::handleOdometryInput(const types::Pose& pose, double timestamp) {
    odometry_count_++;
    LOG(INFO) << "Graph adapter processing odometry message #" << odometry_count_;
    synchronizer_.addPoseMessage(pose, timestamp);
}

void GraphAdapter::handleImageInput(const types::Image& image, double timestamp) {
    synchronizer_.addMessage(image, timestamp);
}

void GraphAdapter::handleCameraInfo(const types::CameraInfo& camera_info, double timestamp) {
    synchronizer_.addMessage(camera_info, timestamp);
    orb_tracker_.addCameraInfo(camera_info);

    // Only try to get transform if transform tree is available and we have the required frames
    try {
        auto tfResult = transform_tree_->getTransform(base_link_frame_id_, camera_info.frame_id);
        core::types::Pose camera_pose;
        camera_pose.position = tfResult.transform.translation();
        camera_pose.orientation = Eigen::Quaterniond(tfResult.transform.rotation());
        orb_tracker_.addCameraPose(camera_pose, camera_info.frame_id);
    } catch (const std::exception& e) {
        LOG(INFO) << "Transform not available for camera info (using visual odometry): "
                  << e.what();
    }
}

void GraphAdapter::setKeyframeDistanceThreshold(double threshold) {
    auto thresholds = keyframe_manager_.getThresholds();
    thresholds.distance_threshold = threshold;
    keyframe_manager_.setThresholds(thresholds);
}

void GraphAdapter::setCallbacks(const GraphCallbacks& callbacks) {
    callbacks_ = callbacks;
}

std::shared_ptr<types::KeyFrame> GraphAdapter::createKeyframe(
    const types::Pose& pose, const std::optional<types::Image>& image,
    const std::optional<types::CameraInfo>& camera_info,
    const std::optional<types::PointCloud>& cloud, const std::optional<types::ImuData>& imu_data) {
    if (!keyframe_manager_.shouldCreateKeyframe(pose, image, camera_info, imu_data)) {
        LOG(INFO) << "Skipping keyframe creation: " << keyframe_manager_.getLastDecisionReason();

        if (imu_data.has_value()) {
            imu_preintegrator_->addMeasurement(imu_data.value(), pose.timestamp);

            static size_t imu_count = 0;
            if (++imu_count % 50 == 0) {
                LOG(INFO) << "Accumulated " << imu_preintegrator_->getDataCount()
                          << " synchronized IMU measurements in preintegrator (total: " << imu_count
                          << ")";
            }
        }

        return nullptr;
    }

    LOG(INFO) << std::fixed << "Creating keyframe at pose: " << pose.position.transpose()
              << " with timestamp: " << pose.timestamp
              << " (image: " << (image.has_value() ? "available" : "unavailable")
              << ", camera_info: " << (camera_info.has_value() ? "available" : "unavailable")
              << ", cloud: " << (cloud.has_value() ? "available" : "unavailable")
              << ", imu: " << (imu_data.has_value() ? "AVAILABLE" : "MISSING") << ")";

    auto keyframe = std::make_shared<types::KeyFrame>();
    keyframe->id = ++current_keyframe_id_;
    keyframe->pose = pose;

    if (cloud) {
        keyframe->depth_data = *cloud;
    }

    if (image) {
        keyframe->color_data = *image;
    }

    if (camera_info) {
        keyframe->camera_info = *camera_info;
    }

    if (image && camera_info) {
        keyframe_ids_with_images_.push_back(keyframe->id);
    }

    if (imu_data.has_value()) {
        imu_preintegrator_->addMeasurement(imu_data.value(), pose.timestamp);
        LOG(INFO) << "Added synchronized IMU measurement to preintegrator at keyframe creation"
                  << " (preintegrator now has " << imu_preintegrator_->getDataCount()
                  << " measurements)";
    } else {
        LOG(WARNING) << "No IMU data synchronized for keyframe " << keyframe->id << " at timestamp "
                     << pose.timestamp;
    }

    // Preintegrate IMU data(TODO: check if this is correct) and update KeyframeManager with the new
    // keyframe
    keyframe_manager_.updateLastKeyframe(pose, image, camera_info, imu_data);

    addKeyframeToGraph(keyframe);

    return keyframe;
}

double GraphAdapter::calculateDistance(const types::Pose& relative_pose) {
    return relative_pose.position.norm();
}

void GraphAdapter::handleLoopClosure(uint64_t from_id, uint64_t to_id,
                                     const types::Pose& relative_pose) {
    types::Factor loop_factor;
    loop_factor.id = next_factor_id_++;
    loop_factor.type = proto::FactorType::LOOP_CLOSURE;
    loop_factor.connected_nodes = {from_id, to_id};
    loop_factor.measurement.emplace<1>(relative_pose);
    loop_factor.information = Eigen::Matrix<double, 6, 6>::Identity();
    store_.addFactor(loop_factor);

    LOG(INFO) << "Created loop closure factor #" << loop_factor.id << " between keyframes "
              << from_id << " → " << to_id;
}

void GraphAdapter::addOdometryFactor(uint64_t from_id, uint64_t to_id,
                                     const types::Pose& relative_pose) {
    types::Factor odom_factor;
    odom_factor.id = next_factor_id_++;
    odom_factor.type = proto::FactorType::ODOMETRY;
    odom_factor.connected_nodes = {from_id, to_id};
    odom_factor.measurement.emplace<1>(relative_pose);
    odom_factor.information = Eigen::Matrix<double, 6, 6>::Identity();
    store_.addFactor(odom_factor);

    cumulative_distance_ += calculateDistance(relative_pose);

    LOG(INFO) << "Created odometry factor #" << odom_factor.id << " between keyframes " << from_id
              << " → " << to_id;
}

void GraphAdapter::maybeDumpGraph(bool force) {
    constexpr double DUMP_INTERVAL = 1.0;  // meters

    if (cumulative_distance_ >= next_dump_distance_ || force) {
        LOG(INFO) << "Dumping factor graph at distance " << cumulative_distance_ << "m";

        std::string filename_prefix = "factor_graph_" + 
                                     std::to_string(static_cast<int>(cumulative_distance_)) + "m";

        try {
            util::dumpFactorGraphWithAutoPath(store_, filename_prefix);
            LOG(INFO) << "Factor graph dumped with prefix: " << filename_prefix;
        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to dump factor graph: " << e.what();
        }
        
        next_dump_distance_ = cumulative_distance_ + DUMP_INTERVAL;
        LOG(INFO) << "Next graph dump at " << next_dump_distance_ << "m";
    }
}

void GraphAdapter::addKeyframeToGraph(const std::shared_ptr<types::KeyFrame>& keyframe) {
    LOG(INFO) << "Adding keyframe to three-queue processing pipeline at pose: "
              << keyframe->pose.position.transpose()
              << " with timestamp: " << keyframe->pose.timestamp;

    // Add keyframe to unprocessed cache
    store_.addToUnprocessedCache(keyframe);
    LOG(INFO) << "Keyframe " << keyframe->id << " added to unprocessed cache";

    // Create odometry factor if we have a previous keyframe
    if (current_keyframe_id_ > 1) {
        // Get the previous keyframe pose from unprocessed cache or other queues
        auto prev_keyframe = getKeyFrameFromAnyQueue(current_keyframe_id_ - 1);
        if (prev_keyframe) {
            types::Pose relative_pose = prev_keyframe->pose.inverse() * keyframe->pose;
            double distance = calculateDistance(relative_pose);
            total_keyframe_distance_ += distance;

            LOG(INFO) << "Distance to previous keyframe: " << std::fixed << std::setprecision(2)
                      << distance
                      << "m, Total distance between keyframes: " << total_keyframe_distance_ << "m";

            // Create and store odometry factor
            addOdometryFactor(current_keyframe_id_ - 1, current_keyframe_id_, relative_pose);

            // Create IMU factor if we have IMU data
            if (imu_preintegrator_->hasData() &&
                last_keyframe_for_imu_ != current_keyframe_id_ - 1) {
                auto imu_measurement = imu_preintegrator_->getPreintegratedMeasurement();

                types::Factor imu_factor;
                imu_factor.id = next_factor_id_++;
                imu_factor.type = proto::FactorType::IMU_PREINTEGRATED;
                imu_factor.connected_nodes = {current_keyframe_id_ - 1, current_keyframe_id_};
                imu_factor.measurement.emplace<2>(imu_measurement);
                imu_factor.information =
                    Eigen::MatrixXd::Identity(9, 9) * 10.0;  // Information matrix for IMU factor
                imu_factor.timestamp = keyframe->pose.timestamp;
                imu_factor.description = "IMU preintegrated measurement";

                store_.addFactor(imu_factor);

                LOG(INFO) << "Created IMU factor #" << imu_factor.id << " between keyframes "
                          << (current_keyframe_id_ - 1) << " → " << current_keyframe_id_ << " with "
                          << imu_preintegrator_->getDataCount() << " IMU measurements"
                          << " (dt=" << std::fixed << std::setprecision(3)
                          << imu_measurement.delta_time << "s)";

                // Reset preintegrator for next keyframe interval
                imu_preintegrator_->reset();
                last_keyframe_for_imu_ = current_keyframe_id_;
            }
        }

        // ===== ORB PROCESSING WITH 2-FRAME DELAY =====

        // Check if we can process ORB features (need current frame N and previous frame N-1)
        if (current_keyframe_id_ >= 2) {
            auto current_frame_for_orb = getKeyFrameFromAnyQueue(current_keyframe_id_);  // Frame N
            auto previous_frame_for_orb =
                getKeyFrameFromAnyQueue(current_keyframe_id_ - 1);  // Frame N-1

            if (current_frame_for_orb && previous_frame_for_orb &&
                current_frame_for_orb->hasColorImage() && previous_frame_for_orb->hasColorImage()) {
                try {
                    LOG(INFO) << "Processing ORB features for frames " << (current_keyframe_id_ - 1)
                              << " → " << current_keyframe_id_;

                    // Get current map keypoints from MapStore
                    auto current_map_keypoints = getMapPointsCopy();

                    // Process ORB features
                    orb_tracker_(*current_frame_for_orb, *previous_frame_for_orb,
                                 current_map_keypoints);

                    // Update MapStore with new/updated map keypoints
                    updateMapPoints(current_map_keypoints);

                    LOG(INFO) << "ORB processing completed successfully for frames "
                              << (current_keyframe_id_ - 1) << " → " << current_keyframe_id_;

                } catch (const std::exception& e) {
                    LOG(WARNING) << "ORB processing failed for frames "
                                 << (current_keyframe_id_ - 1) << " → " << current_keyframe_id_
                                 << ": " << e.what();
                    // Continue processing even if ORB fails
                }
            }

            // Move frame N-2 to processed non-optimized queue (2-frame delay)
            if (current_keyframe_id_ >= 3) {
                uint64_t frame_to_move = current_keyframe_id_ - 2;
                store_.moveToProcessedNonOptimized(frame_to_move);
                LOG(INFO) << "Moved keyframe " << frame_to_move
                          << " to processed non-optimized queue";

                // Update map point eviction tracking
                store_.evictOldMapPoints(current_keyframe_id_,
                                         30);  // Evict points older than 30 keyframes
            }
        }
    }

    // Trigger storage-based visualization callback using MapStore as single source of truth
    if (callbacks_.on_storage_updated) {
        // Visualization gets keypoints directly from MapStore - no need to pass them
        callbacks_.on_storage_updated(store_, current_keyframe_id_, getPreviousKeyframeId());
    }

    // Debug: Dump map keypoints after each keyframe addition
    dumpMapKeypointsToJson();

    keyframes_since_last_optimization_++;

    if (shouldTriggerOptimization()) {
        LOG(INFO) << "Triggering batch optimization after "
                  << keyframes_since_last_optimization_.load() << " keyframes";
        triggerOptimization();
    }
}

void GraphAdapter::handlePointCloudInput(const types::PointCloud& cloud, double timestamp) {
    // synchronizer_.addMessage(cloud, timestamp);
}

void GraphAdapter::handleImuInput(const types::ImuData& imu_data, double timestamp) {
    // Initialize bias from static period if not yet done
    if (!bias_initialized_ && imu_preintegrator_->getDataCount() >= bias_estimation_window_) {
        initializeBiasFromStaticPeriod();
        bias_initialized_ = true;
    }

    // Add IMU data to message synchronizer for proper temporal synchronization
    synchronizer_.addMessage(imu_data, timestamp);

    static size_t imu_count = 0;
    if (++imu_count % 200 == 0) {
        LOG(INFO) << "Added " << imu_count << " IMU messages to synchronizer"
                  << " (queue size: " << synchronizer_.getQueueSize<types::ImuData>() << ")"
                  << " (bias initialized: " << (bias_initialized_ ? "YES" : "NO") << ")";
    }
}

void GraphAdapter::updateBiasEstimate() {
    // Simple bias update using exponential moving average
    // Might actually need something sophisticated
    const double alpha = 0.01;  // Learning rate for bias adaptation

    // TODO: bias would be estimated from:
    // 1. Static periods detection
    // 2. Optimization feedback from factor graph
    // 3. Kalman filter-style estimation

    LOG(INFO) << "Current bias estimate - Accel: "
              << current_bias_estimate_.accelerometer_bias.transpose()
              << ", Gyro: " << current_bias_estimate_.gyroscope_bias.transpose();
}

void GraphAdapter::initializeBiasFromStaticPeriod() {
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    size_t count = 0;

    const auto& measurements = imu_preintegrator_->getMeasurements();
    for (const auto& measurement : measurements) {
        if (count >= bias_estimation_window_)
            break;

        accel_sum += measurement.first.linear_acceleration;
        gyro_sum += measurement.first.angular_velocity;
        count++;
    }

    if (count > 0) {
        Eigen::Vector3d accel_mean = accel_sum / count;
        Eigen::Vector3d gyro_mean = gyro_sum / count;

        // Gyroscope bias is simply the mean (should be zero when stationary)
        current_bias_estimate_.gyroscope_bias = gyro_mean;

        // Accelerometer bias is mean minus gravity vector
        // Assuming robot starts level, gravity is [0, 0, -9.81]
        Eigen::Vector3d expected_gravity(0, 0, -9.81);
        current_bias_estimate_.accelerometer_bias = accel_mean - expected_gravity;

        // Apply the estimated bias to the preintegrator
        imu_preintegrator_->setBiasState(current_bias_estimate_);

        LOG(INFO) << "Initialized IMU bias from " << count << " static measurements:"
                  << "\n  Accelerometer bias: "
                  << current_bias_estimate_.accelerometer_bias.transpose()
                  << "\n  Gyroscope bias: " << current_bias_estimate_.gyroscope_bias.transpose();
    } else {
        LOG(WARNING) << "Failed to initialize IMU bias - insufficient measurements";
    }
}

void GraphAdapter::initializeFirstFrame(const types::Image& image,
                                        const types::CameraInfo& camera_info, double timestamp) {
    current_pose_.position = Eigen::Vector3d::Zero();
    current_pose_.orientation = Eigen::Quaterniond::Identity();
    current_pose_.timestamp = timestamp;
    current_pose_.frame_id = reference_frame_id_;

    if (image.encoding == "rgb8" || image.encoding == "bgr8") {
        if (image.encoding == "rgb8") {
            cv::cvtColor(image.data, previous_image_, cv::COLOR_RGB2GRAY);
        } else {
            cv::cvtColor(image.data, previous_image_, cv::COLOR_BGR2GRAY);
        }
    } else if (image.encoding == "mono8" || image.encoding == "8UC1") {
        previous_image_ = image.data.clone();
    } else {
        LOG(ERROR) << "Unsupported image encoding for visual odometry: " << image.encoding;
        return;
    }

    last_visual_timestamp_ = timestamp;
    first_frame_processed_ = true;

    LOG(INFO) << "Visual odometry initialized at timestamp " << timestamp;

    synchronizer_.addPoseMessage(current_pose_, timestamp);
}

std::optional<types::CameraInfo> GraphAdapter::findSynchronizedCameraInfo(double timestamp) {
    if (latest_camera_info_.has_value()) {
        double time_diff = std::abs(timestamp - latest_camera_info_timestamp_);
        if (time_diff < 0.5) {
            return latest_camera_info_;
        }
    }
    return std::nullopt;
}

void GraphAdapter::setFrameIds(const std::string& base_link_frame_id,
                               const std::string& camera_frame_id) {
    base_link_frame_id_ = base_link_frame_id;
    camera_frame_id_ = camera_frame_id;
    orb_tracker_.setBaseFrameId(base_link_frame_id_);
}

void GraphAdapter::setFrameIds(const std::string& reference_frame_id,
                               const std::string& base_link_frame_id,
                               const std::string& camera_frame_id) {
    reference_frame_id_ = reference_frame_id;
    base_link_frame_id_ = base_link_frame_id;
    camera_frame_id_ = camera_frame_id;
    orb_tracker_.setBaseFrameId(base_link_frame_id_);
}

void GraphAdapter::enableVisualOdometryMode() {
    visual_odometry_mode_ = true;
    first_frame_processed_ = false;

    if (!base_link_frame_id_.empty()) {
        orb_tracker_.enableVisualOdometry(true);
        orb_tracker_.setVisualOdometryParams(0.5);  // Default speed
    }

    LOG(INFO) << "Visual odometry mode enabled for pose-free processing";
}

void GraphAdapter::dumpMapKeypointsToJson(const std::string& filepath) const {
    static bool debug_enabled = true;
    if (!debug_enabled)
        return;

    std::ofstream json_file(filepath);
    if (!json_file.is_open()) {
        LOG(WARNING) << "Failed to open map debug file: " << filepath;
        return;
    }

    // Get map keypoints from MapStore instead of local storage
    auto map_keypoints_copy = getMapPointsCopy();

    json_file << "{\n";
    json_file << "  \"timestamp\": " << std::time(nullptr) << ",\n";
    json_file << "  \"total_keyframes\": " << current_keyframe_id_ << ",\n";
    json_file << "  \"total_map_points\": " << map_keypoints_copy.size() << ",\n";
    json_file << "  \"visual_odometry_mode\": " << (visual_odometry_mode_ ? "true" : "false")
              << ",\n";
    json_file << "  \"reference_frame_id\": \"" << reference_frame_id_ << "\",\n";
    json_file << "  \"base_link_frame_id\": \"" << base_link_frame_id_ << "\",\n";
    json_file << "  \"camera_frame_id\": \"" << camera_frame_id_ << "\",\n";

    size_t total_observations = 0;
    size_t duplicate_observations = 0;
    std::map<uint64_t, std::set<uint32_t>> keyframe_to_points;

    for (const auto& [keypoint_id, keypoint] : map_keypoints_copy) {
        total_observations += keypoint.locations.size();

        std::set<uint64_t> seen_keyframes;
        for (const auto& location : keypoint.locations) {
            if (seen_keyframes.count(location.keyframe_id)) {
                duplicate_observations++;
                LOG(WARNING) << "DEBUG: Duplicate observation found for map point " << keypoint_id
                             << " in keyframe " << location.keyframe_id;
            }
            seen_keyframes.insert(location.keyframe_id);
            keyframe_to_points[location.keyframe_id].insert(keypoint_id);
        }
    }

    json_file << "  \"statistics\": {\n";
    json_file << "    \"total_observations\": " << total_observations << ",\n";
    json_file << "    \"duplicate_observations\": " << duplicate_observations << ",\n";
    json_file << "    \"avg_observations_per_point\": "
              << (map_keypoints_copy.empty()
                      ? 0.0
                      : static_cast<double>(total_observations) / map_keypoints_copy.size())
              << "\n";
    json_file << "  },\n";

    json_file << "  \"coordinate_frame_info\": {\n";
    json_file
        << "    \"note\": \"All 3D positions are in reference_frame_id coordinate system\",\n";
    json_file << "    \"transform_chain\": \""
              << (visual_odometry_mode_ ? "camera_frame -> reference_frame (direct)"
                                        : "camera_frame -> base_link_frame -> reference_frame")
              << "\"\n";
    json_file << "  },\n";

    json_file << "  \"map_keypoints\": [\n";

    bool first_keypoint = true;
    for (const auto& [keypoint_id, keypoint] : map_keypoints_copy) {
        if (!first_keypoint) {
            json_file << ",\n";
        }
        first_keypoint = false;

        json_file << "    {\n";
        json_file << "      \"id\": " << keypoint.id() << ",\n";
        json_file << "      \"position_3d\": {\n";
        json_file << "        \"x\": " << std::fixed << std::setprecision(6)
                  << keypoint.position.x() << ",\n";
        json_file << "        \"y\": " << std::fixed << std::setprecision(6)
                  << keypoint.position.y() << ",\n";
        json_file << "        \"z\": " << std::fixed << std::setprecision(6)
                  << keypoint.position.z() << "\n";
        json_file << "      },\n";
        json_file << "      \"observations\": [\n";

        bool first_location = true;
        for (const auto& location : keypoint.locations) {
            if (!first_location) {
                json_file << ",\n";
            }
            first_location = false;

            json_file << "        {\n";
            json_file << "          \"keyframe_id\": " << location.keyframe_id << ",\n";
            json_file << "          \"frame_id\": \"" << location.frame_id << "\",\n";
            json_file << "          \"pixel_x\": " << std::fixed << std::setprecision(2)
                      << location.x << ",\n";
            json_file << "          \"pixel_y\": " << std::fixed << std::setprecision(2)
                      << location.y << "\n";
            json_file << "        }";
        }

        json_file << "\n      ]\n";
        json_file << "    }";
    }

    json_file << "\n  ]\n";
    json_file << "}\n";
    json_file.close();

    LOG(INFO) << "DEBUG: Dumped " << map_keypoints_copy.size() << " map keypoints to " << filepath;
}

bool GraphAdapter::performOptimization() {
    auto optimization_start = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Starting batch bundle adjustment optimization with landmarks";

    // Get batch of keyframes for optimization from processed non-optimized queue
    auto optimization_batch = store_.getBatchForOptimization(50);  // Max 50 keyframes per batch
    if (optimization_batch.empty()) {
        LOG(INFO) << "No keyframes ready for optimization in processed non-optimized queue";
        return true;
    }

    // Extract keyframe IDs from batch
    std::vector<uint64_t> batch_keyframe_ids;
    for (const auto& kf : optimization_batch) {
        batch_keyframe_ids.push_back(kf->id);
    }

    LOG(INFO) << "Optimizing batch of " << optimization_batch.size() << " keyframes: ["
              << batch_keyframe_ids.front() << " to " << batch_keyframe_ids.back() << "]";

    // Get factors relevant to this batch
    auto all_factors = store_.getAllFactors();
    std::vector<types::Factor> batch_factors;

    for (const auto& factor : all_factors) {
        // Include factor if any of its connected nodes are in the batch
        bool include_factor = false;
        for (uint64_t node_id : factor.connected_nodes) {
            if (std::find(batch_keyframe_ids.begin(), batch_keyframe_ids.end(), node_id) !=
                batch_keyframe_ids.end()) {
                include_factor = true;
                break;
            }
        }
        if (include_factor) {
            batch_factors.push_back(factor);
        }
    }

    LOG(INFO) << "Using " << batch_factors.size() << " factors for batch optimization";

    // VISUALIZATION: Dump factor graph before optimization for debugging
    if (!batch_keyframe_ids.empty()) {
        std::string filename_prefix = "pre_optimization_kf" + 
                                     std::to_string(batch_keyframe_ids.front()) + "_to_" +
                                     std::to_string(batch_keyframe_ids.back());
        try {
            util::dumpFactorGraphWithAutoPath(store_, filename_prefix);
            LOG(INFO) << "Dumped factor graph before optimization with prefix: " << filename_prefix;
        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to dump factor graph before optimization: " << e.what();
        }
    }

    // Get current map keypoints for bundle adjustment
    auto current_map_keypoints = getMapPointsCopy();
    if (current_map_keypoints.empty()) {
        LOG(INFO) << "No map keypoints available, falling back to pose-only optimization";
        // ALWAYS move batch to processed optimized and sync to disk, even without optimization
        store_.markBatchAsOptimized(batch_keyframe_ids);
        return true;
    }

    // NEW: Triangulation phase - triangulate keypoints that need 3D positions
    LOG(INFO) << "Starting triangulation phase before batch optimization";
    bool triangulation_success = triangulateMapKeypoints(current_map_keypoints);
    
    if (triangulation_success) {
        // Update MapStore with newly triangulated keypoints
        updateMapPoints(current_map_keypoints);
        LOG(INFO) << "Updated MapStore with newly triangulated keypoints";
        
        // Refresh map keypoints after triangulation
        current_map_keypoints = getMapPointsCopy();
    }

    // Count landmarks that have valid 3D positions for optimization
    size_t valid_landmarks = 0;
    for (const auto& [id, keypoint] : current_map_keypoints) {
        if (!keypoint.needs_triangulation && keypoint.position.norm() > 1e-6) {
            valid_landmarks++;
        }
    }

    LOG(INFO) << "Batch bundle adjustment input: " << optimization_batch.size() << " keyframes, "
              << batch_factors.size() << " factors, " << current_map_keypoints.size()
              << " total keypoints (" << valid_landmarks << " with valid 3D positions)";

    // Variables to track optimization results
    bool optimization_succeeded = false;
    bool poses_success = false;
    bool landmarks_success = false;

    auto gtsam_start = std::chrono::high_resolution_clock::now();

    // Try to perform GTSAM optimization
    if (!graph_.optimizeFromStorageMemoryEfficient(store_, current_map_keypoints)) {
        LOG(ERROR) << "GTSAM batch bundle adjustment failed - will still sync batch to disk";
        optimization_succeeded = false;
    } else {
        auto gtsam_end = std::chrono::high_resolution_clock::now();
        auto gtsam_duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(gtsam_end - gtsam_start);

        auto optimized_poses = graph_.getOptimizedPoses();
        auto optimized_landmarks = graph_.getOptimizedLandmarks();

        if (optimized_poses.empty()) {
            LOG(WARNING) << "No optimized poses received from GTSAM batch bundle adjustment";
            optimization_succeeded = false;
        } else if (optimized_landmarks.empty()) {
            LOG(WARNING) << "No optimized landmarks received from GTSAM batch bundle adjustment";
            optimization_succeeded = false;
        } else {
            LOG(INFO) << "GTSAM batch bundle adjustment completed in " << gtsam_duration.count()
                      << "ms, received " << optimized_poses.size() << " optimized poses and "
                      << optimized_landmarks.size() << " optimized landmarks";

            // Update poses and landmarks through MapStore
            LOG(INFO) << "Updating batch optimization results via MapStore coordination";
            poses_success = store_.updateOptimizedPoses(optimized_poses);
            landmarks_success = updateMapKeypointsFromOptimizedLandmarks(optimized_landmarks);

            if (!poses_success) {
                LOG(WARNING) << "Failed to update optimized poses via MapStore";
            }
            if (!landmarks_success) {
                LOG(WARNING) << "Failed to update optimized landmarks via MapStore";
            }

            optimization_succeeded = poses_success && landmarks_success;
        }
    }

    // ALWAYS move batch to processed optimized queue and sync to disk
    // This ensures data is persisted regardless of optimization success/failure
    LOG(INFO) << "Moving batch to processed optimized queue for disk sync (optimization "
              << (optimization_succeeded ? "SUCCEEDED" : "FAILED") << ")";
    store_.markBatchAsOptimized(batch_keyframe_ids);

    auto optimization_end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                                                optimization_start);

    LOG(INFO) << "Batch bundle adjustment summary:";
    LOG(INFO) << "    Batch size: " << optimization_batch.size() << " keyframes";
    LOG(INFO) << "    Total time: " << total_duration.count() << "ms";
    LOG(INFO) << "    Optimization result: " << (optimization_succeeded ? "SUCCESS" : "FAILED");
    LOG(INFO) << "    Poses updated: " << (poses_success ? "SUCCESS" : "FAILED");
    LOG(INFO) << "    Landmarks updated: " << (landmarks_success ? "SUCCESS" : "FAILED");
    LOG(INFO) << "    Batch moved to processed optimized queue: YES (always)";
    LOG(INFO) << "    Background sync: AUTOMATIC (triggered by batch completion)";

    LOG(INFO) << "Batch bundle adjustment pipeline completed via MapStore coordination";

    return optimization_succeeded;
}

void GraphAdapter::setOptimizationInterval(size_t keyframe_interval) {
    optimization_keyframe_interval_ = keyframe_interval;
    LOG(INFO) << "Set optimization interval to every " << keyframe_interval << " keyframes";
}

void GraphAdapter::enableOptimizationThread() {
    if (!optimization_thread_running_.load()) {
        startOptimizationThread();
        LOG(INFO) << "Optimization thread enabled (interval: " << optimization_keyframe_interval_
                  << " keyframes)";
    } else {
        LOG(INFO) << "Optimization thread already running";
    }
}

void GraphAdapter::disableOptimizationThread() {
    if (optimization_thread_running_.load()) {
        stopOptimizationThread();
        LOG(INFO) << "Optimization thread disabled";
    }
}

void GraphAdapter::startOptimizationThread() {
    if (optimization_thread_running_.load()) {
        LOG(WARNING) << "Optimization thread already running";
        return;
    }

    should_stop_optimization_ = false;
    optimization_thread_running_ = true;

    optimization_thread_ =
        std::make_unique<std::thread>(&GraphAdapter::optimizationThreadLoop, this);
    LOG(INFO) << "Optimization thread started";
}

void GraphAdapter::stopOptimizationThread() {
    if (!optimization_thread_running_.load()) {
        return;
    }

    should_stop_optimization_ = true;
    optimization_thread_running_ = false;

    {
        std::lock_guard<std::mutex> lock(optimization_mutex_);
        optimization_trigger_.notify_all();
    }

    if (optimization_thread_ && optimization_thread_->joinable()) {
        optimization_thread_->join();
    }

    optimization_thread_.reset();
    LOG(INFO) << "Optimization thread stopped";
}

void GraphAdapter::triggerOptimization() {
    if (!optimization_thread_running_.load()) {
        return;
    }

    if (optimization_in_progress_.load()) {
        LOG(INFO) << "Optimization already in progress, skipping trigger";
        return;
    }

    optimization_pending_ = true;
    optimization_trigger_.notify_one();
    LOG(INFO) << "Optimization trigger signaled";
}

bool GraphAdapter::shouldTriggerOptimization() const {
    return keyframes_since_last_optimization_.load() >= optimization_keyframe_interval_;
}

bool GraphAdapter::updateMapKeypointsAfterOptimization(
    const std::map<uint64_t, types::Pose>& optimized_poses) {
    LOG(INFO) << "updateMapKeypointsAfterOptimization: DEPRECATED - MapStore handles optimization "
                 "results";
    LOG(INFO) << "Use MapStore.updateOptimizedPoses() and background sync instead";
    return true;  // MapStore handles this automatically
}

std::map<uint32_t, core::types::Keypoint> GraphAdapter::recomputeMapKeypointsFromOptimizedPoses(
    const std::map<uint64_t, types::Pose>& optimized_poses) {
    LOG(INFO)
        << "recomputeMapKeypointsFromOptimizedPoses: DEPRECATED - MapStore handles optimization";
    return {};  // Return empty map - MapStore handles triangulation and updates
}

bool GraphAdapter::validateMapKeypointConsistency(
    const std::map<uint32_t, core::types::Keypoint>& keypoints) {
    LOG(INFO) << "Validating map keypoint consistency";

    size_t valid_points = 0;
    size_t invalid_points = 0;

    for (const auto& [id, keypoint] : keypoints) {
        if (!keypoint.position.allFinite()) {
            LOG(WARNING) << "Invalid position for map point " << id << ": "
                         << keypoint.position.transpose();
            invalid_points++;
            continue;
        }

        if (keypoint.position.norm() > 1000.0) {
            LOG(WARNING) << "Map point " << id << " too far: " << keypoint.position.norm() << "m";
            invalid_points++;
            continue;
        }

        if (keypoint.locations.size() < 2) {
            LOG(WARNING) << "Map point " << id
                         << " has insufficient observations: " << keypoint.locations.size();
            invalid_points++;
            continue;
        }

        valid_points++;
    }

    double validity_ratio =
        (keypoints.empty()) ? 1.0 : (static_cast<double>(valid_points) / keypoints.size());

    LOG(INFO) << "Map keypoint validation: " << valid_points << "/" << keypoints.size()
              << " valid (" << std::fixed << std::setprecision(1) << (validity_ratio * 100.0)
              << "%)";

    return validity_ratio >= 0.8;  // Require 80% validity
}

bool GraphAdapter::storeOptimizedMapKeypoints() {
    LOG(INFO) << "storeOptimizedMapKeypoints: DEPRECATED - MapStore handles optimization storage";
    LOG(INFO) << "MapStore background sync automatically persists optimization results";
    return true;  // MapStore handles storage automatically
}

bool GraphAdapter::exportFinalOptimizedMap(const std::string& export_path) {
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Savingfinal optimized map to " << export_path;

    try {
        std::ofstream export_file(export_path);
        if (!export_file.is_open()) {
            LOG(ERROR) << "Failed to open export file: " << export_path;
            return false;
        }

        auto all_keyframes = store_.getAllKeyFrames();
        auto all_keypoints = store_.getAllKeyPoints();
        auto all_factors = store_.getAllFactors();

        export_file << "{\n";
        export_file << "  \"export_info\": {\n";
        export_file << "    \"timestamp\": " << std::time(nullptr) << ",\n";

        auto current_time = std::time(nullptr);
        export_file << "    \"export_time\": \""
                    << std::put_time(std::localtime(&current_time), "%Y-%m-%d %H:%M:%S") << "\",\n";
        export_file << "    \"source\": \"GraphAdapter Step 5 Final Storage\",\n";
        export_file << "    \"coordinate_frame\": \"" << reference_frame_id_ << "\",\n";
        export_file << "    \"visual_odometry_mode\": "
                    << (visual_odometry_mode_ ? "true" : "false") << "\n";
        export_file << "  },\n";

        export_file << "  \"summary\": {\n";
        export_file << "    \"total_keyframes\": " << all_keyframes.size() << ",\n";
        export_file << "    \"total_map_points\": " << all_keypoints.size() << ",\n";
        export_file << "    \"total_factors\": " << all_factors.size() << ",\n";
        export_file << "    \"optimization_complete\": true\n";
        export_file << "  },\n";

        export_file << "  \"optimized_trajectory\": [\n";
        bool first_keyframe = true;
        for (const auto& keyframe : all_keyframes) {
            if (!first_keyframe)
                export_file << ",\n";
            first_keyframe = false;

            export_file << "    {\n";
            export_file << "      \"id\": " << keyframe->id << ",\n";
            export_file << "      \"timestamp\": " << std::fixed << std::setprecision(6)
                        << keyframe->pose.timestamp << ",\n";
            export_file << "      \"position\": {\n";
            export_file << "        \"x\": " << std::setprecision(8) << keyframe->pose.position.x()
                        << ",\n";
            export_file << "        \"y\": " << std::setprecision(8) << keyframe->pose.position.y()
                        << ",\n";
            export_file << "        \"z\": " << std::setprecision(8) << keyframe->pose.position.z()
                        << "\n";
            export_file << "      },\n";
            export_file << "      \"orientation\": {\n";
            export_file << "        \"x\": " << std::setprecision(8)
                        << keyframe->pose.orientation.x() << ",\n";
            export_file << "        \"y\": " << std::setprecision(8)
                        << keyframe->pose.orientation.y() << ",\n";
            export_file << "        \"z\": " << std::setprecision(8)
                        << keyframe->pose.orientation.z() << ",\n";
            export_file << "        \"w\": " << std::setprecision(8)
                        << keyframe->pose.orientation.w() << "\n";
            export_file << "      }\n";
            export_file << "    }";
        }
        export_file << "\n  ],\n";

        export_file << "  \"optimized_map_points\": [\n";
        bool first_point = true;
        for (const auto& keypoint : all_keypoints) {
            if (!first_point)
                export_file << ",\n";
            first_point = false;

            export_file << "    {\n";
            export_file << "      \"id\": " << keypoint.id() << ",\n";
            export_file << "      \"position\": {\n";
            export_file << "        \"x\": " << std::setprecision(8) << keypoint.position.x()
                        << ",\n";
            export_file << "        \"y\": " << std::setprecision(8) << keypoint.position.y()
                        << ",\n";
            export_file << "        \"z\": " << std::setprecision(8) << keypoint.position.z()
                        << "\n";
            export_file << "      },\n";
            export_file << "      \"observations\": [\n";

            bool first_obs = true;
            for (const auto& location : keypoint.locations) {
                if (!first_obs)
                    export_file << ",\n";
                first_obs = false;

                export_file << "        {\n";
                export_file << "          \"keyframe_id\": " << location.keyframe_id << ",\n";
                export_file << "          \"pixel_coordinates\": {\n";
                export_file << "            \"u\": " << std::setprecision(2) << location.x << ",\n";
                export_file << "            \"v\": " << std::setprecision(2) << location.y << "\n";
                export_file << "          }\n";
                export_file << "        }";
            }
            export_file << "\n      ]\n";
            export_file << "    }";
        }
        export_file << "\n  ],\n";

        export_file << "  \"factor_graph\": {\n";

        std::map<std::string, size_t> factor_counts;
        for (const auto& factor : all_factors) {
            std::string factor_type;
            switch (factor.type) {
                case proto::FactorType::PRIOR:
                    factor_type = "PRIOR";
                    break;
                case proto::FactorType::ODOMETRY:
                    factor_type = "ODOMETRY";
                    break;
                case proto::FactorType::LOOP_CLOSURE:
                    factor_type = "LOOP_CLOSURE";
                    break;
                case proto::FactorType::IMU_PREINTEGRATED:
                    factor_type = "IMU_PREINTEGRATED";
                    break;
                default:
                    factor_type = "UNKNOWN";
                    break;
            }
            factor_counts[factor_type]++;
        }

        export_file << "    \"factor_summary\": {\n";
        bool first_factor_type = true;
        for (const auto& [type, count] : factor_counts) {
            if (!first_factor_type)
                export_file << ",\n";
            first_factor_type = false;
            export_file << "      \"" << type << "\": " << count;
        }
        export_file << "\n    }\n";
        export_file << "  }\n";

        export_file << "}\n";
        export_file.close();

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        LOG(INFO) << "Exported final optimized map with " << all_keyframes.size()
                  << " keyframes and " << all_keypoints.size() << " map points in "
                  << duration.count() << "ms";

        return true;

    } catch (const std::exception& e) {
        LOG(ERROR) << "Exception during map export: " << e.what();
        return false;
    }
}

void GraphAdapter::optimizationThreadLoop() {
    LOG(INFO) << "Optimization thread loop started";

    size_t optimization_attempts = 0;
    size_t successful_optimizations = 0;
    size_t failed_optimizations = 0;
    auto thread_start = std::chrono::high_resolution_clock::now();

    while (optimization_thread_running_.load() && !should_stop_optimization_.load()) {
        std::unique_lock<std::mutex> lock(optimization_mutex_);

        // Wait indefinitely for optimization trigger (no timeout to avoid continuous iteration)
        optimization_trigger_.wait(lock, [this]() {
            return optimization_pending_.load() || should_stop_optimization_.load();
        });

        if (should_stop_optimization_.load()) {
            break;
        }

        if (optimization_pending_.load() && !optimization_in_progress_.load()) {
            optimization_pending_ = false;
            optimization_in_progress_ = true;
            lock.unlock();

            optimization_attempts++;

            LOG(INFO) << "Background bundle adjustment triggered (#" << optimization_attempts
                      << ") after " << keyframes_since_last_optimization_.load() << " keyframes";

            auto attempt_start = std::chrono::high_resolution_clock::now();

            try {
                bool success = performOptimization();

                auto attempt_end = std::chrono::high_resolution_clock::now();
                auto attempt_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    attempt_end - attempt_start);

                // ALWAYS reset keyframes counter since we processed a batch regardless of
                // success/failure
                keyframes_since_last_optimization_ = 0;

                if (success) {
                    successful_optimizations++;
                    LOG(INFO) << "Background bundle adjustment #" << optimization_attempts
                              << " completed successfully in " << attempt_duration.count() << "ms";

                    if (successful_optimizations % 10 == 0) {
                        LOG(INFO) << "Exporting final optimized map after "
                                  << successful_optimizations << " bundle adjustments";
                        if (!exportFinalOptimizedMap()) {
                            LOG(WARNING) << "Failed to export final optimized map";
                        }
                    }
                } else {
                    failed_optimizations++;
                    LOG(WARNING) << "Background bundle adjustment #" << optimization_attempts
                                 << " failed after " << attempt_duration.count() << "ms"
                                 << " - batch still moved to processed optimized queue";
                }

            } catch (const std::exception& e) {
                failed_optimizations++;
                // Reset counter even for exceptions since batch was likely processed
                keyframes_since_last_optimization_ = 0;
                LOG(ERROR) << "Background bundle adjustment #" << optimization_attempts
                           << " exception: " << e.what();
            } catch (...) {
                failed_optimizations++;
                // Reset counter even for unknown exceptions
                keyframes_since_last_optimization_ = 0;
                LOG(ERROR) << "Background bundle adjustment #" << optimization_attempts
                           << " unknown exception";
            }

            optimization_in_progress_ = false;

            if (optimization_attempts % 5 == 0) {
                auto current_time = std::chrono::high_resolution_clock::now();
                auto total_runtime =
                    std::chrono::duration_cast<std::chrono::minutes>(current_time - thread_start);

                double success_rate =
                    (optimization_attempts > 0)
                        ? (100.0 * successful_optimizations / optimization_attempts)
                        : 0.0;

                LOG(INFO) << "Bundle adjustment thread stats (runtime: " << total_runtime.count()
                          << " min):";
                LOG(INFO) << "    Total attempts: " << optimization_attempts;
                LOG(INFO) << "    Successful: " << successful_optimizations << " (" << std::fixed
                          << std::setprecision(1) << success_rate << "%)";
                LOG(INFO) << "    Failed: " << failed_optimizations;
                LOG(INFO) << "    Note: Failed optimizations still sync data to disk";
            }
        }
    }

    auto thread_end = std::chrono::high_resolution_clock::now();
    auto total_runtime =
        std::chrono::duration_cast<std::chrono::minutes>(thread_end - thread_start);

    LOG(INFO) << "Bundle adjustment thread loop ended after " << total_runtime.count()
              << " minutes";
    LOG(INFO) << "Final stats: " << successful_optimizations << "/" << optimization_attempts
              << " successful bundle adjustments";
}

bool GraphAdapter::updateMapKeypointsFromOptimizedLandmarks(
    const std::map<uint32_t, Eigen::Vector3d>& optimized_landmarks) {
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Updating map keypoints with optimized landmark positions from bundle adjustment";

    if (optimized_landmarks.empty()) {
        LOG(WARNING) << "No optimized landmarks provided for map keypoint update";
        return false;
    }

    // Use MapStore's optimization result handling instead of direct manipulation
    bool success = store_.updateOptimizedLandmarks(optimized_landmarks);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    LOG(INFO) << "Map keypoint update from bundle adjustment completed in " << duration.count()
              << "ms"
              << " via MapStore coordination (success: " << (success ? "YES" : "NO") << ")";

    return success;
}

// ===== NEW TRIANGULATION PHASE =====

bool GraphAdapter::triangulateMapKeypoints(std::map<uint32_t, core::types::Keypoint>& map_keypoints) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    size_t keypoints_needing_triangulation = 0;
    size_t keypoints_triangulated = 0;
    size_t keypoints_insufficient_observations = 0;
    
    // Count keypoints needing triangulation
    for (const auto& [id, keypoint] : map_keypoints) {
        if (keypoint.needs_triangulation) {
            keypoints_needing_triangulation++;
        }
    }
    
    LOG(INFO) << "Triangulation phase: " << keypoints_needing_triangulation 
              << " keypoints need triangulation";
    
    if (keypoints_needing_triangulation == 0) {
        return true;  // Nothing to triangulate
    }
    
    // Triangulate keypoints that need it
    for (auto& [id, keypoint] : map_keypoints) {
        if (!keypoint.needs_triangulation) {
            continue;  // Skip keypoints that already have 3D position
        }
        
        // Check minimum observations (need at least 2, recommend 3+ for robustness)
        if (keypoint.locations.size() < 2) {
            keypoints_insufficient_observations++;
            LOG(INFO) << "Keypoint " << id << " has only " << keypoint.locations.size() 
                      << " observations (need minimum 2), keeping for future triangulation";
            continue;  // Keep for future attempts
        }
        
        Eigen::Vector3d triangulated_position;
        if (triangulateKeypoint(keypoint, map_keypoints, triangulated_position)) {
            // Successfully triangulated
            keypoint.position = triangulated_position;
            keypoint.needs_triangulation = false;
            keypoints_triangulated++;
            
            LOG(INFO) << "Triangulated keypoint " << id << " at position: " 
                      << triangulated_position.transpose() 
                      << " from " << keypoint.locations.size() << " observations";
        } else {
            LOG(WARNING) << "Failed to triangulate keypoint " << id 
                        << " with " << keypoint.locations.size() << " observations";
            // Keep needs_triangulation=true for future attempts
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    LOG(INFO) << "Triangulation phase completed in " << duration.count() << "ms:";
    LOG(INFO) << "    Total needing triangulation: " << keypoints_needing_triangulation;
    LOG(INFO) << "    Successfully triangulated: " << keypoints_triangulated;
    LOG(INFO) << "    Insufficient observations: " << keypoints_insufficient_observations;
    LOG(INFO) << "    Still pending: " << (keypoints_needing_triangulation - keypoints_triangulated);
    
    return keypoints_triangulated > 0;
}

bool GraphAdapter::triangulateKeypoint(const core::types::Keypoint& keypoint, 
                                       const std::map<uint32_t, core::types::Keypoint>& map_keypoints,
                                       Eigen::Vector3d& triangulated_position) {
    if (keypoint.locations.size() < 2) {
        return false;  // Need at least 2 observations
    }
    
    if (!transform_tree_) {
        LOG(ERROR) << "Transform tree not available for triangulation";
        return false;
    }
    
    try {
        // Use GTSAM triangulation API that handles multiple observations automatically
        bool success = gtsam_reconstructor_.triangulateFromMapKeypoint(
            keypoint, store_, *transform_tree_, triangulated_position, base_link_frame_id_);
        
        if (success) {
            // Validate triangulated point
            if (!triangulated_position.allFinite()) {
                LOG(WARNING) << "Triangulated position is not finite";
                return false;
            }
            
            if (triangulated_position.norm() > 1000.0) {
                LOG(WARNING) << "Triangulated position too far: " << triangulated_position.norm() << "m";
                return false;
            }
            
            LOG(INFO) << "Successfully triangulated keypoint " << keypoint.id() 
                      << " at position: " << triangulated_position.transpose()
                      << " from " << keypoint.locations.size() << " observations";
            return true;
        }
        
    } catch (const std::exception& e) {
        LOG(WARNING) << "Triangulation failed with exception: " << e.what();
    }
    
    return false;
}

// Helper method to get keyframe from any queue
std::shared_ptr<types::KeyFrame> GraphAdapter::getKeyFrameFromAnyQueue(uint64_t keyframe_id) const {
    // Try unprocessed cache first
    auto keyframe = store_.getUnprocessedKeyFrame(keyframe_id);
    if (keyframe) {
        return keyframe;
    }

    // Try processed non-optimized cache
    keyframe = store_.getProcessedNonOptimizedKeyFrame(keyframe_id);
    if (keyframe) {
        return keyframe;
    }

    // Try processed optimized cache
    keyframe = store_.getProcessedOptimizedKeyFrame(keyframe_id);
    if (keyframe) {
        return keyframe;
    }

    // Finally try the main storage (disk)
    return store_.getKeyFrame(keyframe_id);
}

void GraphAdapter::updateMapPoints(const std::map<uint32_t, core::types::Keypoint>& updated_map_keypoints) {
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Updating map keypoints with " << updated_map_keypoints.size() << " ORB-processed points";

    size_t new_points = 0;
    size_t updated_points = 0;

    for (const auto& [id, keypoint] : updated_map_keypoints) {
        bool is_new = !store_.hasKeyPoint(id);

        if (is_new) {
            new_points++;
            LOG(INFO) << "Adding new map point " << id << " at position: "
                      << keypoint.position.transpose();
        } else {
            updated_points++;
        }

        // Add/update keypoint and mark as dirty for disk write
        store_.addKeyPoint(keypoint);
        store_.markMapPointDirty(id);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    LOG(INFO) << "Updated map keypoints from ORB processing in " << duration.count() << "ms"
              << " (" << new_points << " new, " << updated_points << " updated)"
              << " - all marked dirty for disk write";
}

}  // namespace graph
}  // namespace core
