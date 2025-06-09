#pragma once

#include <vector>
#include <memory>
#include <set>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include <core/storage/map_store.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/keypoint.hpp>
#include <stf/transform_tree.hpp>

namespace tracking {
namespace image {

// Common data structure for triangulation observations
struct ObservationData {
    std::shared_ptr<core::types::KeyFrame> keyframe;
    cv::Point2f pixel;
    std::string camera_frame_id;
    Eigen::Matrix3d K;
    Eigen::Isometry3d camera_pose_in_odom;
};

// Common utility functions for triangulation
namespace triangulation_utils {

// Ray-based debug function for multi-keyframe observation consistency visualization
void debugUnprojectReproject(const core::types::Keypoint& map_keypoint,
                             const std::vector<ObservationData>& observations,
                             const Eigen::Vector3d& triangulated_point_world,
                             const std::string& reference_frame_id);

// Helper to build ObservationData from map keypoint
std::vector<ObservationData> buildObservationData(
    const core::types::Keypoint& map_keypoint,
    const core::storage::MapStore& map_store,
    const stf::TransformTree& tft,
    const std::string& base_link_frame_id);

// Common coordinate frame transformation: odom_T_camera = odom_T_base_link * base_link_T_camera
Eigen::Isometry3d computeCameraPoseInOdom(
    const core::types::KeyFrame& keyframe,
    const stf::TransformTree& tft,
    const std::string& camera_frame_id,
    const std::string& base_link_frame_id);

}  // namespace triangulation_utils

}  // namespace image
}  // namespace tracking