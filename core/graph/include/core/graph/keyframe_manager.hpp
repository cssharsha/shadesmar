#pragma once

#include <memory>
#include <optional>
#include "core/types/keyframe.hpp"
#include "core/types/pose.hpp"
#include "core/types/image.hpp"
#include "core/types/imu.hpp"
#include "2d/orb_tracker.hpp"

namespace core {
namespace graph {

struct KeyframeThresholds {
    double distance_threshold = 0.05;           // meters
    double rotation_threshold = 5.0;            // degrees
    uint32_t min_inliers = 30;                  // minimum inliers for visual odometry
    double min_translation_visual = 0.1;        // meters (visual threshold)
    bool use_visual_odometry = true;            // whether to use essential matrix

    // IMU-based motion thresholds
    bool use_imu_motion = true;                 // whether to use IMU motion estimation
    double max_linear_velocity = 2.0;          // m/s - threshold for high velocity motion
    double max_angular_velocity = 1.0;         // rad/s - threshold for high rotational motion
    double max_acceleration = 5.0;             // m/s² - threshold for high acceleration
    double imu_integration_window = 1.0;       // seconds - window for IMU motion analysis
    double velocity_threshold_scale = 0.5;     // scale factor for velocity-based distance threshold
};

class KeyframeManager {
public:
    explicit KeyframeManager(const KeyframeThresholds& thresholds = KeyframeThresholds{});

    // Enhanced decision function with IMU data support
    bool shouldCreateKeyframe(
        const core::types::Pose& current_pose,
        const std::optional<core::types::Image>& current_image = std::nullopt,
        const std::optional<core::types::CameraInfo>& camera_info = std::nullopt,
        const std::optional<core::types::ImuData>& imu_data = std::nullopt);

    // Update the last keyframe reference for distance/orientation calculations
    void updateLastKeyframe(const core::types::Pose& pose,
                           const std::optional<core::types::Image>& image = std::nullopt,
                           const std::optional<core::types::CameraInfo>& camera_info = std::nullopt,
                           const std::optional<core::types::ImuData>& imu_data = std::nullopt);

    // Reset for first keyframe
    void reset();

    // Get the last decision details
    const std::string& getLastDecisionReason() const { return last_decision_reason_; }

    // Configuration
    void setThresholds(const KeyframeThresholds& thresholds) { thresholds_ = thresholds; }
    const KeyframeThresholds& getThresholds() const { return thresholds_; }

    // Set ORB tracker (optional - for visual odometry)
    void setOrbTracker(std::shared_ptr<tracking::image::OrbTracker> tracker) {
        orb_tracker_ = tracker;
    }

private:
    KeyframeThresholds thresholds_;
    std::shared_ptr<tracking::image::OrbTracker> orb_tracker_;

    // Last keyframe data
    bool has_last_keyframe_ = false;
    core::types::Pose last_pose_;
    std::optional<core::types::Image> last_image_;
    std::optional<core::types::CameraInfo> last_camera_info_;
    std::optional<core::types::ImuData> last_imu_data_;
    double last_keyframe_timestamp_ = 0.0;

    // Decision tracking
    mutable std::string last_decision_reason_;

    // Helper functions
    bool checkDistanceThreshold(const core::types::Pose& current_pose) const;
    bool checkOrientationThreshold(const core::types::Pose& current_pose) const;
    bool checkVisualOdometry(const core::types::Image& current_image,
                           const core::types::CameraInfo& camera_info) const;

    // New IMU-based motion analysis
    bool checkImuMotion(const core::types::Pose& current_pose,
                       const std::optional<core::types::ImuData>& current_imu,
                       double current_timestamp) const;

    double estimateVelocityFromPose(const core::types::Pose& current_pose, double current_timestamp) const;
    double adaptDistanceThreshold(const core::types::Pose& current_pose,
                                 const std::optional<core::types::ImuData>& imu_data,
                                 double current_timestamp) const;

    double calculateDistance(const core::types::Pose& pose1, const core::types::Pose& pose2) const;
    double calculateRotationAngle(const core::types::Pose& pose1, const core::types::Pose& pose2) const;
};

}  // namespace graph
}  // namespace core