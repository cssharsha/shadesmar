#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <vector>
#include "2d/orb_tracker.hpp"
#include "core/graph/factor_graph.hpp"
#include "core/graph/keyframe_manager.hpp"
#include "core/storage/map_store.hpp"
#include "core/types/imu.hpp"
#include "core/types/keyframe.hpp"
#include "stf/transform_tree.hpp"
#include "utils/message_synchronizer/message_synchronizer.hpp"

#include "logging/logging.hpp"
namespace core {
namespace graph {

// Enhanced IMU preintegrator with bias estimation and better numerical integration
class SimpleImuPreintegrator {
public:
    struct BiasState {
        Eigen::Vector3d accelerometer_bias;
        Eigen::Vector3d gyroscope_bias;

        BiasState() {
            accelerometer_bias = Eigen::Vector3d::Zero();
            gyroscope_bias = Eigen::Vector3d::Zero();
        }
    };

    void reset() {
        imu_measurements_.clear();
        preintegrated_position_ = Eigen::Vector3d::Zero();
        preintegrated_velocity_ = Eigen::Vector3d::Zero();
        preintegrated_rotation_ = Eigen::Quaterniond::Identity();
        delta_time_ = 0.0;
        start_time_ = 0.0;

        // Reset covariance to small initial uncertainty
        covariance_matrix_ = Eigen::Matrix<double, 9, 9>::Identity() * 1e-8;

        // Initialize process noise (these should be calibrated for your specific IMU)
        accel_noise_density_ = 0.01;       // m/s^2/sqrt(Hz)
        gyro_noise_density_ = 0.001;       // rad/s/sqrt(Hz)
        accel_bias_random_walk_ = 0.0001;  // m/s^3/sqrt(Hz)
        gyro_bias_random_walk_ = 0.00001;  // rad/s^2/sqrt(Hz)
    }

    void setBiasState(const BiasState& bias) {
        bias_state_ = bias;
    }

    const BiasState& getBiasState() const {
        return bias_state_;
    }

    void addMeasurement(const types::ImuData& imu_data, double timestamp) {
        if (imu_measurements_.empty()) {
            start_time_ = timestamp;
            last_timestamp_ = timestamp;
            imu_measurements_.push_back(std::make_pair(imu_data, timestamp));
            return;
        }

        double dt = timestamp - last_timestamp_;
        if (dt <= 0 || dt > 1.0) {  // Skip invalid timestamps or gaps > 1 second
            LOG(WARNING) << "Invalid IMU timestamp interval: " << dt << "s, skipping measurement";
            return;
        }

        imu_measurements_.push_back(std::make_pair(imu_data, timestamp));

        // Apply bias correction
        Eigen::Vector3d corrected_accel =
            imu_data.linear_acceleration - bias_state_.accelerometer_bias;
        Eigen::Vector3d corrected_gyro = imu_data.angular_velocity - bias_state_.gyroscope_bias;

        // Use mid-point integration for better accuracy
        integrateStep(corrected_accel, corrected_gyro, dt, imu_data);

        delta_time_ += dt;
        last_timestamp_ = timestamp;
    }

    types::ImuPreintegratedMeasurement getPreintegratedMeasurement() const {
        types::ImuPreintegratedMeasurement measurement;
        measurement.preintegrated_position = preintegrated_position_;
        measurement.preintegrated_velocity = preintegrated_velocity_;
        measurement.preintegrated_rotation = preintegrated_rotation_;
        measurement.delta_time = delta_time_;
        measurement.accelerometer_bias = bias_state_.accelerometer_bias;
        measurement.gyroscope_bias = bias_state_.gyroscope_bias;
        measurement.covariance_matrix = covariance_matrix_;

        return measurement;
    }

    bool hasData() const {
        LOG(INFO) << "Current imu measurements: " << imu_measurements_.size();
        return !imu_measurements_.empty();
    }

    size_t getDataCount() const {
        return imu_measurements_.size();
    }

    double getDeltaTime() const {
        return delta_time_;
    }

    // Access to measurements for bias estimation
    const std::deque<std::pair<types::ImuData, double>>& getMeasurements() const {
        return imu_measurements_;
    }

private:
    std::deque<std::pair<types::ImuData, double>> imu_measurements_;
    Eigen::Vector3d preintegrated_position_;
    Eigen::Vector3d preintegrated_velocity_;
    Eigen::Quaterniond preintegrated_rotation_;
    Eigen::Matrix<double, 9, 9> covariance_matrix_;

    double delta_time_;
    double start_time_;
    double last_timestamp_;

    BiasState bias_state_;

    // Noise parameters (should be calibrated for specific IMU)
    double accel_noise_density_;
    double gyro_noise_density_;
    double accel_bias_random_walk_;
    double gyro_bias_random_walk_;

    void integrateStep(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt,
                       const types::ImuData& imu_data) {
        // Save previous state for mid-point integration
        Eigen::Vector3d prev_velocity = preintegrated_velocity_;
        Eigen::Quaterniond prev_rotation = preintegrated_rotation_;

        // Integrate rotation using Rodriguez formula for small angles
        Eigen::Vector3d delta_angle = gyro * dt;
        double angle_magnitude = delta_angle.norm();

        Eigen::Quaterniond delta_rotation;
        if (angle_magnitude > 1e-8) {
            Eigen::Vector3d axis = delta_angle.normalized();
            delta_rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angle_magnitude, axis));
        } else {
            // For very small angles, use linear approximation
            delta_rotation = Eigen::Quaterniond(1, 0.5 * delta_angle.x(), 0.5 * delta_angle.y(),
                                                0.5 * delta_angle.z());
            delta_rotation.normalize();
        }

        preintegrated_rotation_ = preintegrated_rotation_ * delta_rotation;

        // Mid-point integration for acceleration
        Eigen::Quaterniond mid_rotation = prev_rotation.slerp(0.5, preintegrated_rotation_);

        // Remove gravity and transform to world frame
        Eigen::Vector3d gravity(0, 0, -9.81);
        Eigen::Vector3d world_accel = mid_rotation * accel - gravity;

        // Integrate velocity and position
        preintegrated_velocity_ += world_accel * dt;
        preintegrated_position_ += prev_velocity * dt + 0.5 * world_accel * dt * dt;

        // Update covariance matrix (simplified discrete-time propagation)
        updateCovariance(dt, imu_data);
    }

    void updateCovariance(double dt, const types::ImuData& imu_data) {
        // Process noise matrix Q (simplified)
        Eigen::Matrix<double, 9, 9> Q = Eigen::Matrix<double, 9, 9>::Zero();

        // Position process noise (from acceleration uncertainty)
        double pos_noise = accel_noise_density_ * dt * dt;
        Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * pos_noise;

        // Velocity process noise
        double vel_noise = accel_noise_density_ * dt;
        Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * vel_noise;

        // Rotation process noise
        double rot_noise = gyro_noise_density_ * dt;
        Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * rot_noise;

        // Add measurement noise from IMU covariances
        if (imu_data.linear_acceleration_covariance.determinant() > 1e-12) {
            Q.block<3, 3>(0, 0) +=
                imu_data.linear_acceleration_covariance * dt * dt * dt * dt / 4.0;
            Q.block<3, 3>(3, 3) += imu_data.linear_acceleration_covariance * dt * dt;
        }

        if (imu_data.angular_velocity_covariance.determinant() > 1e-12) {
            Q.block<3, 3>(6, 6) += imu_data.angular_velocity_covariance * dt * dt;
        }

        // Simple covariance propagation: P = P + Q
        covariance_matrix_ += Q;

        // Ensure covariance matrix stays positive definite
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> solver(covariance_matrix_);
        if (solver.eigenvalues().minCoeff() < 1e-12) {
            covariance_matrix_ += Eigen::Matrix<double, 9, 9>::Identity() * 1e-12;
        }
    }
};

struct GraphCallbacks {
    std::function<void()> on_graph_updated;

    // New callback for storage-based visualization
    std::function<void(const core::storage::MapStore&,
                       const std::map<uint32_t, core::types::Keypoint>&,
                       uint64_t current_keyframe_id, uint64_t previous_keyframe_id)>
        on_storage_updated;
};

class GraphAdapter {
public:
    GraphAdapter(FactorGraph& graph, storage::MapStore& store);
    ~GraphAdapter();  // Need destructor to properly stop optimization thread

    void handleOdometryInput(const types::Pose& pose, double timestamp);
    void handleImageInput(const types::Image& image, double timestamp);
    void handleCameraInfo(const types::CameraInfo& camera_info, double timestamp);
    void handleImuInput(const types::ImuData& imu_data, double timestamp);
    void handleLoopClosure(uint64_t from_id, uint64_t to_id, const types::Pose& relative_pose);
    void setKeyframeDistanceThreshold(double threshold);
    void setCallbacks(const GraphCallbacks& callbacks);
    void handlePointCloudInput(const types::PointCloud& cloud, double timestamp);
    void maybeDumpGraph(bool force = false);
    std::shared_ptr<types::KeyFrame> createKeyframe(
        const types::Pose& pose, const std::optional<types::Image>& image = std::nullopt,
        const std::optional<types::CameraInfo>& camera_info = std::nullopt,
        const std::optional<types::PointCloud>& cloud = std::nullopt,
        const std::optional<types::ImuData>& imu_data = std::nullopt);

    void setTransformTree(std::shared_ptr<stf::TransformTree> transform_tree) {
        transform_tree_ = transform_tree;
        orb_tracker_.setTransformTree(transform_tree_);
        orb_tracker_.setBaseFrameId(base_link_frame_id_);
        keyframe_manager_.setOrbTracker(
            std::make_shared<tracking::image::OrbTracker>(orb_tracker_));
    }

    auto& getMapPoints() const {
        return map_keypoints_;
    }

    // Thread-safe access to map keypoints
    std::map<uint32_t, core::types::Keypoint> getMapPointsCopy() const {
        std::shared_lock<std::shared_mutex> lock(map_keypoints_mutex_);
        return map_keypoints_;
    }

    // Thread-safe access to specific map keypoint
    std::optional<core::types::Keypoint> getMapPoint(uint32_t id) const {
        std::shared_lock<std::shared_mutex> lock(map_keypoints_mutex_);
        auto it = map_keypoints_.find(id);
        if (it != map_keypoints_.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    // Thread-safe update of map keypoints (for optimization results)
    void updateMapPoints(const std::map<uint32_t, core::types::Keypoint>& updated_points) {
        std::unique_lock<std::shared_mutex> lock(map_keypoints_mutex_);
        for (const auto& [id, point] : updated_points) {
            if (map_keypoints_.find(id) != map_keypoints_.end()) {
                map_keypoints_[id] = point;
            }
        }
    }

    // Thread-safe access to in-memory keyframes
    std::shared_ptr<types::KeyFrame> getInMemoryKeyFrameSafe(uint64_t id) const {
        std::shared_lock<std::shared_mutex> lock(in_memory_keyframes_mutex_);
        return graph_.getKeyFramePtr(id);
    }

    // Thread-safe access to all in-memory keyframes
    std::vector<std::shared_ptr<types::KeyFrame>> getAllInMemoryKeyFramesSafe() const {
        std::shared_lock<std::shared_mutex> lock(in_memory_keyframes_mutex_);
        return graph_.getAllKeyFrames();
    }

    // Thread-safe update of in-memory keyframes with optimized poses
    void updateInMemoryKeyframesWithOptimizedPoses(
        const std::map<uint64_t, types::Pose>& optimized_poses) {
        std::unique_lock<std::shared_mutex> lock(in_memory_keyframes_mutex_);
        for (const auto& [keyframe_id, optimized_pose] : optimized_poses) {
            auto keyframe = graph_.getKeyFramePtr(keyframe_id);
            if (keyframe) {
                keyframe->pose = optimized_pose;
            }
        }
    }

    // Get current and previous keyframe IDs for visualization
    uint64_t getCurrentKeyframeId() const {
        return current_keyframe_id_;
    }
    uint64_t getPreviousKeyframeId() const {
        return current_keyframe_id_ > 1 ? current_keyframe_id_ - 1 : 0;
    }

    // Configure keyframe creation thresholds
    void setKeyframeThresholds(const KeyframeThresholds& thresholds) {
        keyframe_manager_.setThresholds(thresholds);
    }

    // Configure memory management for keyframes
    void setInMemoryKeyframeLimit(size_t limit) {
        in_memory_keyframe_limit_ = limit;
        LOG(INFO) << "Set in-memory keyframe limit to " << limit << " keyframes";
    }

    size_t getInMemoryKeyframeLimit() const {
        return in_memory_keyframe_limit_;
    }

    void setFrameIds(const std::string& base_link_frame_id, const std::string& camera_frame_id);

    void setFrameIds(const std::string& reference_frame_id, const std::string& base_link_frame_id,
                     const std::string& camera_frame_id);

    // Enable visual odometry mode for datasets without poses (like TUM)
    void enableVisualOdometryMode();

    // Visual odometry methods
    bool estimatePoseFromVisualOdometry(const types::Image& image,
                                        const types::CameraInfo& camera_info, double timestamp,
                                        types::Pose& estimated_pose);
    void initializeFirstFrame(const types::Image& image, const types::CameraInfo& camera_info,
                              double timestamp);
    std::optional<types::CameraInfo> findSynchronizedCameraInfo(double timestamp);

    // Debug method for map keypoints (easily toggleable)
    void dumpMapKeypointsToJson(const std::string& filepath = "/data/robot/map.json") const;

    // Storage-based optimization
    bool performOptimization();  // Optimize using storage data and update results

    // Step 5: Map keypoint final storage methods
    bool updateMapKeypointsAfterOptimization(
        const std::map<uint64_t, types::Pose>& optimized_poses);
    bool updateMapKeypointsFromOptimizedLandmarks(
        const std::map<uint32_t, Eigen::Vector3d>& optimized_landmarks);
    bool storeOptimizedMapKeypoints();
    std::map<uint32_t, core::types::Keypoint> recomputeMapKeypointsFromOptimizedPoses(
        const std::map<uint64_t, types::Pose>& optimized_poses);
    bool validateMapKeypointConsistency(const std::map<uint32_t, core::types::Keypoint>& keypoints);
    bool exportFinalOptimizedMap(
        const std::string& export_path = "/data/robot/final_optimized_map.json");

    // Step 3: Optimization thread configuration
    void setOptimizationInterval(size_t keyframe_interval);
    void enableOptimizationThread();
    void disableOptimizationThread();
    bool isOptimizationInProgress() const {
        return optimization_in_progress_.load();
    }
    size_t getKeyframesSinceLastOptimization() const {
        return keyframes_since_last_optimization_.load();
    }

    // Thread-safe optimization methods
    void startOptimizationThread();
    void stopOptimizationThread();
    void optimizationThreadLoop();
    void triggerOptimization();
    bool shouldTriggerOptimization() const;

    // Step 5: Helper methods for map keypoint triangulation
    bool triangulatePoint(const core::types::Keypoint& original_keypoint,
                          const std::vector<std::pair<uint64_t, types::Pose>>& observing_keyframes,
                          Eigen::Vector3d& triangulated_position);

private:
    FactorGraph& graph_;
    storage::MapStore& store_;
    utils::MessageSynchronizer<types::Image, types::CameraInfo, types::ImuData> synchronizer_;

    tracking::image::OrbTracker orb_tracker_;
    KeyframeManager keyframe_manager_;

    std::atomic<uint64_t> current_keyframe_id_{0};  // Make atomic for thread safety
    std::vector<uint64_t> keyframe_ids_with_images_;
    uint64_t last_keyframe_for_orb_ = 0;

    std::atomic<uint64_t> next_factor_id_{1};

    void addOdometryFactor(uint64_t from_id, uint64_t to_id, const types::Pose& relative_pose);
    GraphCallbacks callbacks_;

    void addKeyframeToGraph(const std::shared_ptr<types::KeyFrame>& keyframe);
    void manageInMemoryKeyframes();        // Maintain keyframe limit in factor graph
    void manageInMemoryKeyframesUnsafe();  // Unsafe version (mutex already held)
    double calculateDistance(const types::Pose& relative_pose);

    double cumulative_distance_;
    double next_dump_distance_;
    size_t odometry_count_ = 0;

    double total_keyframe_distance_ = 0.0;
    std::shared_ptr<stf::TransformTree> transform_tree_;

    std::map<uint32_t, core::types::Keypoint> map_keypoints_;

    // IMU preintegration
    std::unique_ptr<SimpleImuPreintegrator> imu_preintegrator_;
    uint64_t last_keyframe_for_imu_ = 0;

    // IMU bias estimation
    SimpleImuPreintegrator::BiasState current_bias_estimate_;
    size_t bias_estimation_window_ = 100;  // Number of IMU measurements for bias estimation
    bool bias_initialized_ = false;

    // IMU bias estimation methods
    void updateBiasEstimate();
    void initializeBiasFromStaticPeriod();

    std::string base_link_frame_id_;
    std::string camera_frame_id_;
    std::string reference_frame_id_;

    // Visual odometry state for pose-free datasets (like TUM)
    bool visual_odometry_mode_ = false;
    types::Pose current_pose_;
    bool first_frame_processed_ = false;
    double last_visual_timestamp_ = 0.0;
    cv::Mat previous_image_;
    std::vector<cv::KeyPoint> previous_keypoints_;
    cv::Mat previous_descriptors_;

    // Store most recent camera info for visual odometry
    std::optional<types::CameraInfo> latest_camera_info_;
    double latest_camera_info_timestamp_ = 0.0;

    // Thread synchronization for map keypoints
    mutable std::shared_mutex map_keypoints_mutex_;

    // Thread synchronization for in-memory keyframes (current + previous)
    mutable std::shared_mutex in_memory_keyframes_mutex_;

    // Memory management settings
    size_t in_memory_keyframe_limit_ = 2;  // Default: keep only current + previous keyframes

    // Step 3: Optimization thread infrastructure
    std::unique_ptr<std::thread> optimization_thread_;
    std::atomic<bool> optimization_thread_running_{false};
    std::atomic<bool> should_stop_optimization_{false};
    std::condition_variable optimization_trigger_;
    std::mutex optimization_mutex_;

    // Optimization configuration
    size_t optimization_keyframe_interval_ = 5;  // Optimize every 5 keyframes
    std::atomic<size_t> keyframes_since_last_optimization_{0};
    std::atomic<bool> optimization_pending_{false};
    std::atomic<bool> optimization_in_progress_{false};
};

}  // namespace graph
}  // namespace core
