#pragma once

#include <iostream>
#include <map>
#include <memory>  // For std::unique_ptr
#include <optional>
#include <string>
#include <vector>

// OpenCV for cv::Mat
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <core/storage/map_store.hpp>
#include <core/types/image.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/keypoint.hpp>
#include <core/types/pose.hpp>

#include <stf/transform_tree.hpp>

#include <2d/gtsam_reconstruction.hpp>
#include <2d/reconstruction.hpp>

namespace tracking {
namespace image {

struct KeyframeDecision {
    bool should_create_keyframe;
    std::optional<core::types::Pose> relative_pose;
    int inlier_count;
    double translation_magnitude;
    std::string reason;
};

class OrbTracker {
public:
    OrbTracker(uint32_t num_features = 2000, float scal_factor = 1.2f, uint32_t levels = 8);

    void addCameraInfo(const core::types::CameraInfo& cam_info);
    void addCameraPose(const core::types::Pose& cam_pose, std::string camera_frame);
    std::optional<core::types::Pose> operator()(
        const core::types::KeyFrame& cur_kf, const core::types::KeyFrame& prev_kf,
        std::map<uint32_t, core::types::Keypoint>& map_keypoints);

    // New method to evaluate keyframe necessity based on essential matrix
    KeyframeDecision evaluateKeyframeNecessity(const core::types::Image& current_image,
                                               const core::types::Image& previous_image,
                                               const core::types::CameraInfo& camera_info);

    void setTransformTree(std::shared_ptr<stf::TransformTree>& tft) {
        tft_ = tft;
    }

    void setBaseFrameId(const std::string& base_link_frame_id) {
        base_link_frame_id_ = base_link_frame_id;
    }

    // Visual odometry methods (for TUM-style datasets without ground truth poses)
    void enableVisualOdometry(bool enable = true);
    void setVisualOdometryParams(double estimated_speed = 0.5);

    std::optional<core::types::Pose> estimateVisualOdometryPose(
        const core::types::Image& current_image, const core::types::Image& previous_image,
        const core::types::CameraInfo& camera_info, double timestamp, double previous_timestamp);

    // Direct triangulation method for visual odometry mode
    void performDirectTriangulation(const core::types::KeyFrame& current_kf,
                                    const core::types::KeyFrame& previous_kf,
                                    std::map<uint32_t, core::types::Keypoint>& map_keypoints);

    // New interface using map_store and keyframe_ids
    void performDirectTriangulationWithMapStore(
        uint64_t current_kf_id, uint64_t previous_kf_id, const core::storage::MapStore& map_store,
        std::map<uint32_t, core::types::Keypoint>& map_keypoints);

private:
    std::map<std::string, core::types::CameraInfo> cam_infos_;
    std::map<std::string, core::types::Pose> cam_poses_;
    std::map<std::string, cv::Mat> projection_matrix_;

    uint32_t num_features_;
    float scale_factor_;
    uint32_t n_levels_;

    uint32_t min_matches_for_matching_ = 50;
    uint32_t min_inliers_for_keyframe_ = 30;
    double min_translation_for_keyframe_ = 0.1;  // meters
    double min_rotation_for_keyframe_ = 5.0;     // degrees

    cv::Ptr<cv::ORB> orb_detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    GtsamReconstruct reconstruct;

    std::optional<core::types::Pose> track(
        const core::types::KeyFrame& cur_kf, const std::vector<cv::KeyPoint>& current_img_keypoints,
        const cv::Mat& current_img_descriptors,
        std::map<uint32_t, core::types::Keypoint>& map_keypoints,
        std::map<uint32_t, uint32_t>& current_img_to_map_keypoint_idx);

    // OLD ARCHITECTURE: Immediate triangulation (still available for legacy use)
    std::optional<core::types::Pose> matchAndTriangulate(
        const std::vector<cv::KeyPoint>& cur_img_kps, const cv::Mat& cur_img_desc,
        const std::vector<cv::KeyPoint>& prev_img_kps, const cv::Mat& prev_img_desc,
        const core::types::KeyFrame& cur_frame, const core::types::KeyFrame& prev_frame,
        const cv::Mat& K, const std::map<uint32_t, uint32_t>& current_img_to_map_keypoint_idx,
        std::map<uint32_t, core::types::Keypoint>& map_keypoints);

    // NEW ARCHITECTURE: Feature matching only (no triangulation) - defer to graph_adapter.cpp
    std::optional<core::types::Pose> matchFeaturesOnly(
        const std::vector<cv::KeyPoint>& cur_img_kps, const cv::Mat& cur_img_desc,
        const std::vector<cv::KeyPoint>& prev_img_kps, const cv::Mat& prev_img_desc,
        const core::types::KeyFrame& cur_frame, const core::types::KeyFrame& prev_frame,
        const cv::Mat& K, const std::map<uint32_t, uint32_t>& current_img_to_map_keypoint_idx,
        std::map<uint32_t, core::types::Keypoint>& map_keypoints);

    // Legacy version using map_store (still uses immediate triangulation)
    std::optional<core::types::Pose> matchAndTriangulateWithMapStore(
        const std::vector<cv::KeyPoint>& cur_img_kps, const cv::Mat& cur_img_desc,
        const std::vector<cv::KeyPoint>& prev_img_kps, const cv::Mat& prev_img_desc,
        uint64_t cur_frame_id, uint64_t prev_frame_id, const core::storage::MapStore& map_store,
        const cv::Mat& K, const std::map<uint32_t, uint32_t>& current_img_to_map_keypoint_idx,
        std::map<uint32_t, core::types::Keypoint>& map_keypoints);

    // Helper method for essential matrix computation
    KeyframeDecision computeEssentialMatrix(const std::vector<cv::Point2f>& prev_points,
                                            const std::vector<cv::Point2f>& cur_points,
                                            const cv::Mat& K);

    std::shared_ptr<stf::TransformTree> tft_;
    std::string base_link_frame_id_;

    // Visual odometry state (separate from tracking)
    bool visual_odometry_enabled_ = false;
    double vo_estimated_speed_ = 0.5;  // Default speed assumption for scale
    core::types::Pose current_visual_pose_;
    cv::Mat previous_vo_image_;
    std::vector<cv::KeyPoint> previous_vo_keypoints_;
    cv::Mat previous_vo_descriptors_;
    double last_vo_timestamp_ = 0.0;
    bool vo_initialized_ = false;

    // Structure to store visual odometry computation data for triangulation
    struct VisualOdometryData {
        std::vector<cv::Point2f> prev_matched_points;
        std::vector<cv::Point2f> curr_matched_points;
        std::vector<cv::KeyPoint> prev_keypoints;
        std::vector<cv::KeyPoint> curr_keypoints;
        cv::Mat prev_descriptors;
        cv::Mat curr_descriptors;
        Eigen::Isometry3d essential_transform;
        int inlier_count = 0;
        bool data_valid = false;

        void clear() {
            prev_matched_points.clear();
            curr_matched_points.clear();
            prev_keypoints.clear();
            curr_keypoints.clear();
            prev_descriptors = cv::Mat();
            curr_descriptors = cv::Mat();
            essential_transform = Eigen::Isometry3d::Identity();
            inlier_count = 0;
            data_valid = false;
        }
    };

    VisualOdometryData last_vo_data_;
};

}  // namespace image

}  // namespace tracking
