#include "core/graph/keyframe_manager.hpp"
#include "logging/logging.hpp"
#include <sstream>
#include <iomanip>
#include <cmath>

namespace core {
namespace graph {

KeyframeManager::KeyframeManager(const KeyframeThresholds& thresholds)
    : thresholds_(thresholds) {}

bool KeyframeManager::shouldCreateKeyframe(
    const core::types::Pose& current_pose,
    const std::optional<core::types::Image>& current_image,
    const std::optional<core::types::CameraInfo>& camera_info,
    const std::optional<core::types::ImuData>& imu_data) {

    // Always create the first keyframe
    if (!has_last_keyframe_) {
        last_decision_reason_ = "First keyframe";
        return true;
    }

    double current_timestamp = current_pose.timestamp;

    // Check basic geometric thresholds (distance and orientation)
    bool distance_exceeded = checkDistanceThreshold(current_pose);
    bool rotation_exceeded = checkOrientationThreshold(current_pose);

    // Check visual odometry if enabled and data is available
    bool visual_exceeded = false;
    if (thresholds_.use_visual_odometry && current_image && camera_info &&
        last_image_ && last_camera_info_ && orb_tracker_) {
        visual_exceeded = checkVisualOdometry(*current_image, *camera_info);
    }

    // Check IMU-based motion if enabled and data is available
    bool imu_motion_exceeded = false;
    if (thresholds_.use_imu_motion && imu_data) {
        imu_motion_exceeded = checkImuMotion(current_pose, imu_data, current_timestamp);
    }

    // Adaptive distance threshold based on motion dynamics
    double adaptive_distance_threshold = thresholds_.distance_threshold;
    if (thresholds_.use_imu_motion && imu_data) {
        adaptive_distance_threshold = adaptDistanceThreshold(current_pose, imu_data, current_timestamp);
    }

    bool adaptive_distance_exceeded = false;
    if (has_last_keyframe_) {
        double distance = calculateDistance(last_pose_, current_pose);
        adaptive_distance_exceeded = distance > adaptive_distance_threshold;
    }

    // Build decision reason with comprehensive motion analysis
    std::stringstream reason;
    double distance = calculateDistance(last_pose_, current_pose);
    double rotation_deg = calculateRotationAngle(last_pose_, current_pose) * 180.0 / M_PI;
    double velocity = estimateVelocityFromPose(current_pose, current_timestamp);

    reason << std::fixed << std::setprecision(3)
           << "Distance: " << distance << "m/" << thresholds_.distance_threshold << "m"
           << " (adaptive: " << std::setprecision(3) << adaptive_distance_threshold << "m)"
           << ", Rotation: " << std::setprecision(1) << rotation_deg << "°/"
           << thresholds_.rotation_threshold << "°"
           << ", Velocity: " << std::setprecision(2) << velocity << "m/s";

    if (thresholds_.use_visual_odometry && current_image && camera_info &&
        last_image_ && last_camera_info_ && orb_tracker_) {
        reason << ", Visual: " << (visual_exceeded ? "EXCEEDED" : "OK");
    }

    if (thresholds_.use_imu_motion && imu_data) {
        reason << ", IMU: " << (imu_motion_exceeded ? "EXCEEDED" : "OK");
    }

    // Decision: create keyframe if any threshold is exceeded
    bool should_create = distance_exceeded || rotation_exceeded || visual_exceeded ||
                        imu_motion_exceeded || adaptive_distance_exceeded;

    reason << " -> " << (should_create ? "CREATE" : "SKIP");
    last_decision_reason_ = reason.str();

    LOG(INFO) << "Enhanced keyframe decision: " << last_decision_reason_;

    return should_create;
}

void KeyframeManager::updateLastKeyframe(
    const core::types::Pose& pose,
    const std::optional<core::types::Image>& image,
    const std::optional<core::types::CameraInfo>& camera_info,
    const std::optional<core::types::ImuData>& imu_data) {

    has_last_keyframe_ = true;
    last_pose_ = pose;
    last_image_ = image;
    last_camera_info_ = camera_info;
    last_imu_data_ = imu_data;
    last_keyframe_timestamp_ = pose.timestamp;
}

void KeyframeManager::reset() {
    has_last_keyframe_ = false;
    last_image_.reset();
    last_camera_info_.reset();
    last_imu_data_.reset();
    last_keyframe_timestamp_ = 0.0;
    last_decision_reason_.clear();
}

bool KeyframeManager::checkDistanceThreshold(const core::types::Pose& current_pose) const {
    double distance = calculateDistance(last_pose_, current_pose);
    return distance > thresholds_.distance_threshold;
}

bool KeyframeManager::checkOrientationThreshold(const core::types::Pose& current_pose) const {
    double rotation_angle = calculateRotationAngle(last_pose_, current_pose);
    double rotation_deg = rotation_angle * 180.0 / M_PI;
    return rotation_deg > thresholds_.rotation_threshold;
}

bool KeyframeManager::checkVisualOdometry(
    const core::types::Image& current_image,
    const core::types::CameraInfo& camera_info) const {

    if (!orb_tracker_ || !last_image_ || !last_camera_info_) {
        return false;
    }

    auto decision = orb_tracker_->evaluateKeyframeNecessity(
        current_image, *last_image_, camera_info);

    LOG(INFO) << "Visual odometry decision: " << decision.reason;

    return decision.should_create_keyframe;
}

double KeyframeManager::calculateDistance(
    const core::types::Pose& pose1, const core::types::Pose& pose2) const {
    return (pose1.position - pose2.position).norm();
}

double KeyframeManager::calculateRotationAngle(
    const core::types::Pose& pose1, const core::types::Pose& pose2) const {
    // Calculate relative rotation
    Eigen::Quaterniond relative_rotation = pose1.orientation.inverse() * pose2.orientation;

    // Convert to angle-axis to get rotation angle
    Eigen::AngleAxisd angle_axis(relative_rotation);
    return std::abs(angle_axis.angle());
}

bool KeyframeManager::checkImuMotion(
    const core::types::Pose& current_pose,
    const std::optional<core::types::ImuData>& current_imu,
    double current_timestamp) const {

    if (!current_imu || !last_imu_data_) {
        return false;  // Can't check IMU motion without both measurements
    }

    double dt = current_timestamp - last_keyframe_timestamp_;
    if (dt <= 0 || dt > thresholds_.imu_integration_window) {
        return false;  // Time window too large or invalid
    }

    // Check linear acceleration magnitude
    double linear_accel_magnitude = current_imu->linear_acceleration.norm();
    if (linear_accel_magnitude > thresholds_.max_acceleration) {
        LOG(INFO) << "High acceleration detected: " << linear_accel_magnitude
                  << " m/s² (threshold: " << thresholds_.max_acceleration << " m/s²)";
        return true;
    }

    // Check angular velocity magnitude
    double angular_vel_magnitude = current_imu->angular_velocity.norm();
    if (angular_vel_magnitude > thresholds_.max_angular_velocity) {
        LOG(INFO) << "High angular velocity detected: " << angular_vel_magnitude
                  << " rad/s (threshold: " << thresholds_.max_angular_velocity << " rad/s)";
        return true;
    }

    // Check estimated linear velocity from pose changes
    double estimated_velocity = estimateVelocityFromPose(current_pose, current_timestamp);
    if (estimated_velocity > thresholds_.max_linear_velocity) {
        LOG(INFO) << "High velocity detected: " << estimated_velocity
                  << " m/s (threshold: " << thresholds_.max_linear_velocity << " m/s)";
        return true;
    }

    return false;
}

double KeyframeManager::estimateVelocityFromPose(
    const core::types::Pose& current_pose, double current_timestamp) const {

    if (!has_last_keyframe_ || last_keyframe_timestamp_ <= 0) {
        return 0.0;
    }

    double dt = current_timestamp - last_keyframe_timestamp_;
    if (dt <= 0) {
        return 0.0;
    }

    double distance = calculateDistance(last_pose_, current_pose);
    return distance / dt;
}

double KeyframeManager::adaptDistanceThreshold(
    const core::types::Pose& current_pose,
    const std::optional<core::types::ImuData>& imu_data,
    double current_timestamp) const {

    double base_threshold = thresholds_.distance_threshold;

    if (!imu_data) {
        return base_threshold;
    }

    // Estimate current velocity
    double velocity = estimateVelocityFromPose(current_pose, current_timestamp);

    // Adaptive threshold: increase distance threshold for high-velocity motion
    // This prevents creating too many keyframes during fast motion
    double velocity_factor = 1.0;
    if (velocity > thresholds_.max_linear_velocity * 0.5) {
        // Scale threshold up when moving fast
        velocity_factor = 1.0 + (velocity / thresholds_.max_linear_velocity) * thresholds_.velocity_threshold_scale;
    }

    // Check for high dynamic motion (high acceleration or angular velocity)
    double linear_accel_magnitude = imu_data->linear_acceleration.norm();
    double angular_vel_magnitude = imu_data->angular_velocity.norm();

    double motion_factor = 1.0;
    if (linear_accel_magnitude > thresholds_.max_acceleration * 0.5 ||
        angular_vel_magnitude > thresholds_.max_angular_velocity * 0.5) {
        // Reduce threshold for high dynamic motion to capture more detail
        motion_factor = 0.7;
    }

    double adapted_threshold = base_threshold * velocity_factor * motion_factor;

    // Clamp to reasonable bounds
    adapted_threshold = std::max(adapted_threshold, base_threshold * 0.5);  // Don't go below 50% of base
    adapted_threshold = std::min(adapted_threshold, base_threshold * 3.0);  // Don't go above 300% of base

    return adapted_threshold;
}

}  // namespace graph
}  // namespace core