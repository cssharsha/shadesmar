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
      bias_estimation_window_(100) {}

// Destructor - ensure optimization thread is properly stopped
GraphAdapter::~GraphAdapter() {
    stopOptimizationThread();
}

// Modify input handlers to use synchronizer
void GraphAdapter::handleOdometryInput(const types::Pose& pose, double timestamp) {
    odometry_count_++;
    LOG(INFO) << "Graph adapter processing odometry message #" << odometry_count_;
    synchronizer_.addPoseMessage(pose, timestamp);
}

void GraphAdapter::handleImageInput(const types::Image& image, double timestamp) {
    synchronizer_.addMessage(image, timestamp);

    if (visual_odometry_mode_) {
        if (latest_camera_info_.has_value()) {
            double time_diff = std::abs(timestamp - latest_camera_info_timestamp_);
            if (time_diff < 0.5) {
                if (!first_frame_processed_) {
                    // Initialize with first frame
                    initializeFirstFrame(image, latest_camera_info_.value(), timestamp);
                } else {
                    // Estimate pose from visual odometry
                    types::Pose estimated_pose;
                    if (estimatePoseFromVisualOdometry(image, latest_camera_info_.value(),
                                                       timestamp, estimated_pose)) {
                        // Add generated pose to synchronizer
                        synchronizer_.addPoseMessage(estimated_pose, timestamp);
                    } else {
                        LOG(WARNING) << "Visual odometry failed for timestamp " << timestamp;
                    }
                }
            } else {
                LOG(WARNING) << "Camera info too old for visual odometry: " << time_diff << "s";
            }
        } else {
            LOG(INFO) << "Waiting for camera info for visual odometry at timestamp " << timestamp;
        }
    }
}

void GraphAdapter::handleCameraInfo(const types::CameraInfo& camera_info, double timestamp) {
    synchronizer_.addMessage(camera_info, timestamp);

    // Store latest camera info for visual odometry
    if (visual_odometry_mode_) {
        latest_camera_info_ = camera_info;
        latest_camera_info_timestamp_ = timestamp;
    }

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

    graph_.addFactor(loop_factor);
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

    graph_.addFactor(odom_factor);
    store_.addFactor(odom_factor);

    cumulative_distance_ += calculateDistance(relative_pose);

    LOG(INFO) << "Created odometry factor #" << odom_factor.id << " between keyframes " << from_id
              << " → " << to_id;
}

void GraphAdapter::maybeDumpGraph(bool force) {
    constexpr double DUMP_INTERVAL = 1.0;  // meters

    if (cumulative_distance_ >= next_dump_distance_ || force) {
        LOG(INFO) << "Dumping factor graph at distance " << cumulative_distance_ << "m";

        std::string filename = "/data/robot/factor_graph_" +
                               std::to_string(static_cast<int>(cumulative_distance_)) + "m.vtk";

        util::dumpFactorGraph(graph_, filename);
        next_dump_distance_ = cumulative_distance_ + DUMP_INTERVAL;

        LOG(INFO) << "Next graph dump at " << next_dump_distance_ << "m";
    }
}

void GraphAdapter::addKeyframeToGraph(const std::shared_ptr<types::KeyFrame>& keyframe) {
    LOG(INFO) << "Adding keyframe to storage at pose: " << keyframe->pose.position.transpose()
              << " with timestamp: " << keyframe->pose.timestamp;

    store_.addKeyFrame(keyframe);
    LOG(INFO) << "Keyframe " << keyframe->id << " stored to persistent storage";

    {
        std::unique_lock<std::shared_mutex> lock(in_memory_keyframes_mutex_);
        graph_.addKeyFrame(keyframe);

        manageInMemoryKeyframesUnsafe();
    }

    if (current_keyframe_id_ > 1) {
        // Get the previous keyframe pose from the KeyframeManager's last pose
        auto prev_keyframe = graph_.getKeyFramePtr(current_keyframe_id_ - 1);
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

                graph_.addFactor(imu_factor);
                store_.addFactor(imu_factor);

                LOG(INFO) << "Created IMU factor #" << imu_factor.id << " between keyframes "
                          << (current_keyframe_id_ - 1) << " → " << current_keyframe_id_ << " with "
                          << imu_preintegrator_->getDataCount() << " IMU measurements"
                          << " (dt=" << std::fixed << std::setprecision(3)
                          << imu_measurement.delta_time << "s)";

                // Reset preintegrator for next keyframe interval
                imu_preintegrator_->reset();
                last_keyframe_for_imu_ = current_keyframe_id_;
            } else {
                LOG(INFO) << "IMU factor check: hasData=" << imu_preintegrator_->hasData()
                          << ", last_keyframe_for_imu=" << last_keyframe_for_imu_
                          << ", current_keyframe_id=" << current_keyframe_id_
                          << ", data_count=" << imu_preintegrator_->getDataCount();
            }
        }

        if (keyframe_ids_with_images_.size() > 1 &&
            last_keyframe_for_orb_ != keyframe_ids_with_images_.back()) {
            if (visual_odometry_mode_) {
                LOG(INFO) << "Visual odometry mode: using direct triangulation instead of full ORB "
                             "tracking";
                try {
                    std::unique_lock<std::shared_mutex> lock(map_keypoints_mutex_);
                    uint64_t current_kf_id = keyframe_ids_with_images_.back();
                    uint64_t previous_kf_id =
                        keyframe_ids_with_images_.at(keyframe_ids_with_images_.size() - 2);

                    // Use new interface that leverages stored odometry factors
                    orb_tracker_.performDirectTriangulationWithMapStore(
                        current_kf_id, previous_kf_id, store_, map_keypoints_);
                    lock.unlock();  // Release lock after triangulation
                    last_keyframe_for_orb_ = keyframe_ids_with_images_.back();
                    LOG(INFO) << "Direct triangulation with map_store completed successfully";
                } catch (const std::exception& e) {
                    LOG(WARNING) << "Direct triangulation failed: " << e.what();
                }
            } else {
                LOG(INFO) << "Standard mode: performing traditional ORB tracking";
                try {
                    std::unique_lock<std::shared_mutex> lock(map_keypoints_mutex_);
                    LOG(INFO) << "Current number of keyframes with images: "
                              << keyframe_ids_with_images_.size();
                    LOG(INFO) << "Current: " << keyframe_ids_with_images_.back() << " "
                              << keyframe_ids_with_images_.at(keyframe_ids_with_images_.size() - 2);
                    orb_tracker_(*store_.getKeyFrame(keyframe_ids_with_images_.back()),
                                 *store_.getKeyFrame(keyframe_ids_with_images_.at(
                                     keyframe_ids_with_images_.size() - 2)),
                                 map_keypoints_);
                    // orb_tracker_(graph_.getKeyFrame(keyframe_ids_with_images_.back()),
                    //              graph_.getKeyFrame(keyframe_ids_with_images_.at(
                    //                  keyframe_ids_with_images_.size() - 2)),
                    //              map_keypoints_);
                    lock.unlock();  // Release lock after tracking
                    last_keyframe_for_orb_ = keyframe_ids_with_images_.back();
                    LOG(INFO) << "Traditional ORB tracking completed successfully";
                } catch (const std::exception& e) {
                    LOG(WARNING) << "Traditional ORB tracking failed: " << e.what();
                }
            }
        }
    }

    if (callbacks_.on_graph_updated) {
        callbacks_.on_graph_updated();
    }

    // New storage-based visualization callback
    if (callbacks_.on_storage_updated) {
        std::shared_lock<std::shared_mutex> lock(map_keypoints_mutex_);
        auto map_keypoints_copy = map_keypoints_;  // Make a copy for thread safety
        lock.unlock();
        callbacks_.on_storage_updated(store_, map_keypoints_copy, current_keyframe_id_,
                                      getPreviousKeyframeId());
    }

    // Debug: Dump map keypoints after each keyframe addition
    dumpMapKeypointsToJson();

    // Step 3: Check if optimization should be triggered
    keyframes_since_last_optimization_++;

    if (shouldTriggerOptimization()) {
        LOG(INFO) << "Triggering background optimization after "
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

    if (visual_odometry_mode_) {
        if (!base_link_frame_id_.empty()) {
            orb_tracker_.enableVisualOdometry(true);
            orb_tracker_.setVisualOdometryParams(0.5);  // Default speed
        }

        types::Image prev_image = image;

        auto result = orb_tracker_.estimateVisualOdometryPose(image, prev_image, camera_info,
                                                              timestamp, timestamp - 0.1);

        if (result.has_value()) {
            current_pose_ = result.value();
            current_pose_.frame_id = reference_frame_id_;
            LOG(INFO) << "Visual odometry initialized using OrbTracker with "
                      << " features at timestamp " << timestamp;
        } else {
            LOG(INFO) << "Visual odometry initialization using OrbTracker, pose at origin";
        }
    }

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

bool GraphAdapter::estimatePoseFromVisualOdometry(const types::Image& image,
                                                  const types::CameraInfo& camera_info,
                                                  double timestamp, types::Pose& estimated_pose) {
    if (visual_odometry_mode_) {
        if (!first_frame_processed_) {
            LOG(WARNING)
                << "Cannot estimate pose: first frame not yet processed for visual odometry";
            return false;
        }

        types::Image previous_image;
        previous_image.data = previous_image_;
        previous_image.encoding = "mono8";

        auto result = orb_tracker_.estimateVisualOdometryPose(image, previous_image, camera_info,
                                                              timestamp, last_visual_timestamp_);

        if (result.has_value()) {
            estimated_pose = result.value();
            estimated_pose.frame_id = reference_frame_id_;

            cv::Mat current_gray;
            if (image.encoding == "rgb8" || image.encoding == "bgr8") {
                if (image.encoding == "rgb8") {
                    cv::cvtColor(image.data, current_gray, cv::COLOR_RGB2GRAY);
                } else {
                    cv::cvtColor(image.data, current_gray, cv::COLOR_BGR2GRAY);
                }
            } else if (image.encoding == "mono8" || image.encoding == "8UC1") {
                current_gray = image.data.clone();
            }

            previous_image_ = current_gray.clone();
            last_visual_timestamp_ = timestamp;

            LOG(INFO) << "Visual odometry (OrbTracker) estimated pose: "
                      << estimated_pose.position.transpose();
            return true;
        } else {
            LOG(WARNING)
                << "OrbTracker visual odometry failed, should probably use the other method";
        }
    }

    return false;
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

    if (visual_odometry_mode_) {
        orb_tracker_.enableVisualOdometry(true);
        orb_tracker_.setVisualOdometryParams(0.5);  // Default speed
        LOG(INFO) << "Enabled visual odometry in OrbTracker for TUM-style processing";
    }
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

    std::shared_lock<std::shared_mutex> lock(map_keypoints_mutex_);
    // TODO Check for something more efficient
    auto map_keypoints_copy = map_keypoints_;
    lock.unlock();

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

void GraphAdapter::manageInMemoryKeyframes() {
    std::unique_lock<std::shared_mutex> lock(in_memory_keyframes_mutex_);
    manageInMemoryKeyframesUnsafe();
}

void GraphAdapter::manageInMemoryKeyframesUnsafe() {
    auto all_keyframes = graph_.getAllKeyFrames();

    if (all_keyframes.size() <= in_memory_keyframe_limit_) {
        // Nothing to remove yet
        return;
    }

    std::sort(all_keyframes.begin(), all_keyframes.end(),
              [](const auto& a, const auto& b) { return a->id < b->id; });

    size_t keyframes_to_remove = all_keyframes.size() - in_memory_keyframe_limit_;

    LOG(INFO) << "Memory management: removing " << keyframes_to_remove
              << " keyframes from in-memory factor graph (keeping " << in_memory_keyframe_limit_
              << " most recent)";

    for (size_t i = 0; i < keyframes_to_remove; ++i) {
        uint64_t keyframe_id = all_keyframes[i]->id;

        auto stored_keyframe = store_.getKeyFrame(keyframe_id);
        if (stored_keyframe) {
            if (graph_.removeKeyFrame(keyframe_id)) {
                LOG(INFO) << "Removed keyframe " << keyframe_id
                          << " from memory (still in storage)";
            } else {
                LOG(WARNING) << "Failed to remove keyframe " << keyframe_id << " from memory";
            }
        } else {
            LOG(WARNING) << "Keyframe " << keyframe_id
                         << " not found in storage, skipping memory removal";
        }
    }

    LOG(INFO) << "Memory management complete: " << graph_.getAllKeyFrames().size()
              << " keyframes remain in memory";
}

bool GraphAdapter::performOptimization() {
    auto optimization_start = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Starting comprehensive bundle adjustment optimization with landmarks";

    auto all_keyframes = store_.getAllKeyFrames();
    if (all_keyframes.size() < 2) {
        LOG(INFO) << "Insufficient keyframes for optimization (" << all_keyframes.size()
                  << " < 2), skipping";
        return true;
    }

    auto all_factors = store_.getAllFactors();
    if (all_factors.empty()) {
        LOG(WARNING) << "No factors found in storage, skipping optimization";
        return true;
    }

    // Get current map keypoints for bundle adjustment
    auto current_map_keypoints = getMapPointsCopy();
    if (current_map_keypoints.empty()) {
        LOG(INFO) << "No map keypoints available, falling back to pose-only optimization";
        return graph_.optimizeFromStorage(store_);
    }

    LOG(INFO) << "Bundle adjustment input: " << all_keyframes.size() << " keyframes, "
              << all_factors.size() << " factors, " << current_map_keypoints.size() << " landmarks";

    auto gtsam_start = std::chrono::high_resolution_clock::now();

    // Use bundle adjustment with landmarks instead of pose-only optimization
    if (!graph_.optimizeFromStorageWithLandmarks(store_, current_map_keypoints)) {
        LOG(ERROR) << "GTSAM bundle adjustment failed";
        return false;
    }

    auto gtsam_end = std::chrono::high_resolution_clock::now();
    auto gtsam_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(gtsam_end - gtsam_start);

    auto optimized_poses = graph_.getOptimizedPoses();
    auto optimized_landmarks = graph_.getOptimizedLandmarks();

    if (optimized_poses.empty()) {
        LOG(WARNING) << "No optimized poses received from GTSAM bundle adjustment";
        return false;
    }

    if (optimized_landmarks.empty()) {
        LOG(WARNING) << "No optimized landmarks received from GTSAM bundle adjustment";
        return false;
    }

    LOG(INFO) << "GTSAM bundle adjustment completed in " << gtsam_duration.count()
              << "ms, received " << optimized_poses.size() << " optimized poses and "
              << optimized_landmarks.size() << " optimized landmarks";

    updateInMemoryKeyframesWithOptimizedPoses(optimized_poses);

    // Update map keypoints with optimized landmark positions (no retriangulation needed!)
    if (!updateMapKeypointsFromOptimizedLandmarks(optimized_landmarks)) {
        LOG(WARNING) << "Failed to update map keypoints with optimized landmarks";
    }

    if (!storeOptimizedMapKeypoints()) {
        LOG(WARNING) << "Failed to store optimized map keypoints to disk";
    }

    size_t poses_updated = 0;
    double max_position_change = 0.0;
    double max_rotation_change = 0.0;
    double total_position_change = 0.0;

    auto storage_update_start = std::chrono::high_resolution_clock::now();

    for (const auto& [keyframe_id, optimized_pose] : optimized_poses) {
        auto keyframe = store_.getKeyFrame(keyframe_id);
        if (keyframe) {
            double position_change = (keyframe->pose.position - optimized_pose.position).norm();
            double rotation_change =
                keyframe->pose.orientation.angularDistance(optimized_pose.orientation);

            max_position_change = std::max(max_position_change, position_change);
            max_rotation_change = std::max(max_rotation_change, rotation_change);
            total_position_change += position_change;

            // keyframe->pose = optimized_pose;

            if (store_.addKeyFrame(keyframe)) {
                poses_updated++;

                if (position_change > 0.01 || rotation_change > 0.05) {
                    LOG(INFO) << "Keyframe " << keyframe_id << " pose updated: Δpos=" << std::fixed
                              << std::setprecision(3) << position_change
                              << "m, Δrot=" << std::setprecision(2)
                              << (rotation_change * 180.0 / M_PI) << "°";
                }
            } else {
                LOG(WARNING) << "Failed to update keyframe " << keyframe_id << " in storage";
            }
        } else {
            LOG(WARNING) << "Keyframe " << keyframe_id
                         << " not found in storage during pose update";
        }
    }

    auto storage_update_end = std::chrono::high_resolution_clock::now();
    auto storage_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        storage_update_end - storage_update_start);

    auto persistence_start = std::chrono::high_resolution_clock::now();

    if (!store_.saveChanges()) {
        LOG(WARNING) << "Failed to persist updated map indices to disk after optimization";
    } else {
        LOG(INFO) << "Successfully persisted updated map indices to disk";
    }

    auto persistence_end = std::chrono::high_resolution_clock::now();
    auto persistence_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(persistence_end - persistence_start);

    auto optimization_end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(optimization_end -
                                                                                optimization_start);

    LOG(INFO) << "Bundle adjustment summary:";
    LOG(INFO) << "    Total time: " << total_duration.count() << "ms";
    LOG(INFO) << "    GTSAM time: " << gtsam_duration.count() << "ms ("
              << (100.0 * gtsam_duration.count() / total_duration.count()) << "%)";
    LOG(INFO) << "    Storage update: " << storage_duration.count() << "ms ("
              << (100.0 * storage_duration.count() / total_duration.count()) << "%)";
    LOG(INFO) << "    Persistence: " << persistence_duration.count() << "ms ("
              << (100.0 * persistence_duration.count() / total_duration.count()) << "%)";
    LOG(INFO) << "    Poses updated: " << poses_updated << "/" << optimized_poses.size();
    LOG(INFO) << "    Landmarks updated: " << optimized_landmarks.size();

    if (poses_updated > 0) {
        double avg_position_change = total_position_change / poses_updated;
        LOG(INFO) << "    Position changes: max=" << std::fixed << std::setprecision(3)
                  << max_position_change << "m, avg=" << avg_position_change << "m";
        LOG(INFO) << "    Rotation changes: max=" << std::fixed << std::setprecision(2)
                  << (max_rotation_change * 180.0 / M_PI) << "°";
    }

    LOG(INFO) << "Bundle adjustment pipeline completed successfully";

    return true;
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
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Updating map keypoints after pose optimization";

    if (optimized_poses.empty()) {
        LOG(WARNING) << "No optimized poses provided for map keypoint update";
        return false;
    }

    // Recompute 3D positions of map keypoints based on optimized camera poses
    auto updated_keypoints = recomputeMapKeypointsFromOptimizedPoses(optimized_poses);

    if (updated_keypoints.empty()) {
        LOG(WARNING) << "No map keypoints updated after pose optimization";
        return true;
    }

    // Validate consistency of updated keypoints
    if (!validateMapKeypointConsistency(updated_keypoints)) {
        LOG(ERROR) << "Map keypoint consistency validation failed";
        return false;
    }

    {
        std::unique_lock<std::shared_mutex> lock(map_keypoints_mutex_);
        for (const auto& [id, keypoint] : updated_keypoints) {
            if (map_keypoints_.find(id) != map_keypoints_.end()) {
                // map_keypoints_[id] = keypoint;
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    LOG(INFO) << "Updated " << updated_keypoints.size() << " map keypoints in " << duration.count()
              << "ms";

    return true;
}

std::map<uint32_t, core::types::Keypoint> GraphAdapter::recomputeMapKeypointsFromOptimizedPoses(
    const std::map<uint64_t, types::Pose>& optimized_poses) {
    LOG(INFO) << "Recomputing 3D map keypoint positions from optimized poses";

    std::map<uint32_t, core::types::Keypoint> updated_keypoints;

    auto current_keypoints = getMapPointsCopy();
    if (current_keypoints.empty()) {
        LOG(INFO) << "No map keypoints to update";
        return updated_keypoints;
    }

    size_t points_updated = 0;
    size_t points_failed = 0;
    double total_position_change = 0.0;
    double max_position_change = 0.0;

    for (const auto& [keypoint_id, original_keypoint] : current_keypoints) {
        try {
            std::vector<std::pair<uint64_t, types::Pose>> observing_keyframes;

            for (const auto& keyframe_pose_pair : optimized_poses) {
                uint64_t keyframe_id = keyframe_pose_pair.first;
                const types::Pose& pose = keyframe_pose_pair.second;

                auto observation_it = std::find_if(original_keypoint.locations.begin(),
                                                   original_keypoint.locations.end(),
                                                   [keyframe_id](const auto& loc) -> bool {
                                                       return loc.keyframe_id == keyframe_id;
                                                   });

                if (observation_it == original_keypoint.locations.end()) {
                    continue;
                }

                observing_keyframes.push_back({keyframe_id, pose});
            }

            if (observing_keyframes.size() < 2) {
                points_failed++;
                continue;
            }

            Eigen::Vector3d triangulated_position;
            if (triangulatePoint(original_keypoint, observing_keyframes, triangulated_position)) {
                core::types::Keypoint updated_keypoint = original_keypoint;

                double position_change = (updated_keypoint.position - triangulated_position).norm();
                total_position_change += position_change;
                max_position_change = std::max(max_position_change, position_change);

                updated_keypoint.position = triangulated_position;
                updated_keypoints[keypoint_id] = updated_keypoint;
                points_updated++;

                if (position_change > 0.05) {
                    LOG(INFO) << "Map point " << keypoint_id
                              << " position updated: Δ=" << std::fixed << std::setprecision(3)
                              << position_change << "m";
                }
            } else {
                points_failed++;
            }

        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to update map point " << keypoint_id << ": " << e.what();
            points_failed++;
        }
    }

    double avg_position_change =
        (points_updated > 0) ? (total_position_change / points_updated) : 0.0;

    LOG(INFO) << "Map keypoint update summary:";
    LOG(INFO) << "    Total points: " << current_keypoints.size();
    LOG(INFO) << "    Updated: " << points_updated;
    LOG(INFO) << "    Failed: " << points_failed;
    if (points_updated > 0) {
        LOG(INFO) << "    Position changes: max=" << std::fixed << std::setprecision(3)
                  << max_position_change << "m, avg=" << avg_position_change << "m";
    }

    return updated_keypoints;
}

bool GraphAdapter::triangulatePoint(
    const core::types::Keypoint& original_keypoint,
    const std::vector<std::pair<uint64_t, types::Pose>>& observing_keyframes,
    Eigen::Vector3d& triangulated_position) {
    if (observing_keyframes.size() < 2) {
        return false;
    }

    // Use simple linear triangulation for now
    // In a full implementation, might need to use DLT or iterative optimization and actually reuse
    // the one in orbtracker

    Eigen::MatrixXd A(2 * observing_keyframes.size(), 4);
    int row = 0;

    for (size_t i = 0; i < observing_keyframes.size(); ++i) {
        uint64_t keyframe_id = observing_keyframes[i].first;
        const types::Pose& pose = observing_keyframes[i].second;

        auto observation_it = std::find_if(
            original_keypoint.locations.begin(), original_keypoint.locations.end(),
            [keyframe_id](const auto& loc) -> bool { return loc.keyframe_id == keyframe_id; });

        if (observation_it == original_keypoint.locations.end()) {
            continue;
        }

        double u = observation_it->x;
        double v = observation_it->y;

        // TODO: get fx, fy, cx, cy from camera info
        double fx = 525.0, fy = 525.0, cx = 319.5, cy = 239.5;

        double x_norm = (u - cx) / fx;
        double y_norm = (v - cy) / fy;

        Eigen::Matrix3d R = pose.orientation.toRotationMatrix();
        Eigen::Vector3d t = pose.position;

        // Projection matrix P = K[R|t]
        Eigen::Matrix<double, 3, 4> P;
        P.block<3, 3>(0, 0) = R.transpose();
        P.block<3, 1>(0, 3) = -R.transpose() * t;

        // Set up linear system for DLT triangulation
        A.row(row) = x_norm * P.row(2) - P.row(0);
        A.row(row + 1) = y_norm * P.row(2) - P.row(1);
        row += 2;
    }

    // Solve using SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d homogeneous_point = svd.matrixV().col(3);

    if (std::abs(homogeneous_point(3)) < 1e-8) {
        return false;
    }

    triangulated_position = homogeneous_point.head<3>() / homogeneous_point(3);

    if (!triangulated_position.allFinite() || triangulated_position.norm() > 100.0) {
        return false;
    }

    return true;
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
    auto start_time = std::chrono::high_resolution_clock::now();

    LOG(INFO) << "Storing optimized map keypoints to persistent storage";

    // Get current optimized map keypoints (thread-safe)
    auto optimized_keypoints = getMapPointsCopy();
    if (optimized_keypoints.empty()) {
        LOG(INFO) << "No map keypoints to store";
        return true;
    }

    size_t stored_count = 0;
    size_t failed_count = 0;

    for (const auto& [id, keypoint] : optimized_keypoints) {
        try {
            if (store_.addKeyPoint(keypoint)) {
                stored_count++;
            } else {
                failed_count++;
                LOG(WARNING) << "Failed to store map keypoint " << id;
            }
        } catch (const std::exception& e) {
            failed_count++;
            LOG(WARNING) << "Exception storing map keypoint " << id << ": " << e.what();
        }
    }

    if (!store_.saveChanges()) {
        LOG(ERROR) << "Failed to persist map keypoints to disk";
        return false;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    LOG(INFO) << "Stored " << stored_count << " map keypoints to disk in " << duration.count()
              << "ms (failed: " << failed_count << ")";

    return failed_count == 0;
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

        optimization_trigger_.wait_for(lock, std::chrono::seconds(1), [this]() {
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

                if (success) {
                    successful_optimizations++;
                    keyframes_since_last_optimization_ = 0;
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
                                 << " failed after " << attempt_duration.count() << "ms";
                }

            } catch (const std::exception& e) {
                failed_optimizations++;
                LOG(ERROR) << "Background bundle adjustment #" << optimization_attempts
                           << " exception: " << e.what();
            } catch (...) {
                failed_optimizations++;
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

    size_t points_updated = 0;
    size_t points_not_found = 0;
    double total_position_change = 0.0;
    double max_position_change = 0.0;

    {
        std::unique_lock<std::shared_mutex> lock(map_keypoints_mutex_);

        for (const auto& [landmark_id, optimized_position] : optimized_landmarks) {
            auto it = map_keypoints_.find(landmark_id);
            if (it != map_keypoints_.end()) {
                double position_change = (it->second.position - optimized_position).norm();
                total_position_change += position_change;
                max_position_change = std::max(max_position_change, position_change);

                // Update the 3D position with optimized result
                it->second.position = optimized_position;
                points_updated++;

                if (position_change > 0.05) {
                    LOG(INFO) << "Map point " << landmark_id
                              << " position updated by bundle adjustment: Δ=" << std::fixed
                              << std::setprecision(3) << position_change << "m";
                }
            } else {
                points_not_found++;
                LOG(WARNING) << "Optimized landmark " << landmark_id
                             << " not found in current map keypoints";
            }
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    double avg_position_change =
        (points_updated > 0) ? (total_position_change / points_updated) : 0.0;

    LOG(INFO) << "Map keypoint update from bundle adjustment summary:";
    LOG(INFO) << "    Total optimized landmarks: " << optimized_landmarks.size();
    LOG(INFO) << "    Updated: " << points_updated;
    LOG(INFO) << "    Not found: " << points_not_found;
    LOG(INFO) << "    Update time: " << duration.count() << "ms";

    if (points_updated > 0) {
        LOG(INFO) << "    Position changes: max=" << std::fixed << std::setprecision(3)
                  << max_position_change << "m, avg=" << avg_position_change << "m";
    }

    return points_not_found == 0;  // Success if all landmarks were found and updated
}

}  // namespace graph
}  // namespace core
