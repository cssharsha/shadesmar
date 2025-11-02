#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <core/types/image.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/keypoint.hpp>
#include <core/types/pose.hpp>
#include <stf/transform_tree.hpp>

namespace tracking {
namespace image {

// Structure to hold MASt3R inference results
struct MASt3RResult {
    // Dense 3D points in camera frame
    std::vector<Eigen::Vector3d> points_3d;

    // 2D pixel coordinates in previous image
    std::vector<Eigen::Vector2d> pixels_prev;

    // 2D pixel coordinates in current image
    std::vector<Eigen::Vector2d> pixels_curr;

    // Dense feature descriptors (optional, for tracking)
    std::vector<std::vector<float>> descriptors_prev;
    std::vector<std::vector<float>> descriptors_curr;

    // Confidence scores for each correspondence
    std::vector<float> confidence_scores;

    // Relative camera transformation (prev -> curr)
    Eigen::Isometry3d relative_transform;

    bool success = false;
};

/**
 * MASt3R-based tracker for dense 3D reconstruction and feature matching
 *
 * This tracker uses the MASt3R model to:
 * 1. Find dense correspondences between image pairs
 * 2. Estimate relative camera pose
 * 3. Generate dense 3D point clouds in world frame
 * 4. Extract feature descriptors for map point tracking
 */
class MASt3RTracker {
public:
    /**
     * Constructor
     * @param model_path Path to MASt3R model checkpoint
     * @param python_executable Path to Python interpreter with MASt3R installed
     * @param device Device to run inference on ("cuda" or "cpu")
     * @param max_points Maximum number of points to return (for performance)
     */
    explicit MASt3RTracker(
        const std::string& model_path = "naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric",
        const std::string& python_executable = "python3",
        const std::string& device = "cuda",
        size_t max_points = 10000);

    ~MASt3RTracker();

    /**
     * Match two keyframes using MASt3R
     * This is the main interface compatible with existing trackers
     *
     * @param prev_frame Previous keyframe
     * @param cur_frame Current keyframe
     * @param world_points Output vector of 3D points in world coordinates
     * @return Relative pose from prev to cur frame, or nullopt on failure
     */
    std::optional<core::types::Pose> match(
        const core::types::KeyFrame& prev_frame,
        const core::types::KeyFrame& cur_frame,
        std::vector<Eigen::Vector3d>& world_points);

    /**
     * Match two keyframes and populate map keypoints structure
     * This version creates proper Keypoint objects with observations
     *
     * @param prev_frame Previous keyframe
     * @param cur_frame Current keyframe
     * @param map_keypoints Map to populate with keypoints
     * @return Relative pose from prev to cur frame, or nullopt on failure
     */
    std::optional<core::types::Pose> matchAndCreateKeypoints(
        const core::types::KeyFrame& prev_frame,
        const core::types::KeyFrame& cur_frame,
        std::map<uint32_t, core::types::Keypoint>& map_keypoints);

    /**
     * Set the transform tree for coordinate transformations
     */
    void setTransformTree(std::shared_ptr<stf::TransformTree>& tft) {
        tft_ = tft;
    }

    /**
     * Set the base frame ID for transformations
     */
    void setBaseFrameId(const std::string& base_link_frame_id) {
        base_link_frame_id_ = base_link_frame_id;
    }

    /**
     * Configure confidence threshold for filtering correspondences
     */
    void setConfidenceThreshold(float threshold) {
        confidence_threshold_ = threshold;
    }

    /**
     * Enable/disable subsampling for performance
     */
    void setSubsampling(bool enable, int factor = 8) {
        enable_subsampling_ = enable;
        subsample_factor_ = factor;
    }

private:
    /**
     * Run MASt3R inference on two images
     * This communicates with the Python inference service
     */
    MASt3RResult runInference(
        const core::types::Image& prev_image,
        const core::types::Image& cur_image);

    /**
     * Transform 3D points from camera frame to world frame
     */
    std::vector<Eigen::Vector3d> transformToWorldFrame(
        const std::vector<Eigen::Vector3d>& points_camera,
        const core::types::KeyFrame& reference_frame);

    /**
     * Estimate relative pose from point correspondences
     * Uses RANSAC + PnP or essential matrix decomposition
     */
    std::optional<Eigen::Isometry3d> estimateRelativePose(
        const std::vector<Eigen::Vector3d>& points_3d,
        const std::vector<Eigen::Vector2d>& pixels_prev,
        const std::vector<Eigen::Vector2d>& pixels_curr,
        const core::types::CameraInfo& camera_info);

    /**
     * Filter correspondences based on confidence and geometric consistency
     */
    void filterCorrespondences(
        MASt3RResult& result,
        const core::types::CameraInfo& camera_info);

    /**
     * Initialize Python inference service
     */
    bool initializePythonService();

    /**
     * Shutdown Python inference service
     */
    void shutdownPythonService();

    // Configuration
    std::string model_path_;
    std::string python_executable_;
    std::string device_;
    size_t max_points_;
    float confidence_threshold_ = 0.5f;
    bool enable_subsampling_ = true;
    int subsample_factor_ = 8;

    // Transform tree for coordinate conversions
    std::shared_ptr<stf::TransformTree> tft_;
    std::string base_link_frame_id_;

    // Python service handle (opaque pointer to avoid Python.h in header)
    struct PythonServiceImpl;
    std::unique_ptr<PythonServiceImpl> python_service_;

    // Keypoint ID counter for creating new map points
    uint32_t next_keypoint_id_ = 1;

    // Statistics
    size_t total_inferences_ = 0;
    size_t successful_inferences_ = 0;
};

}  // namespace image
}  // namespace tracking
