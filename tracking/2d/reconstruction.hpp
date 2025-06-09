#pragma once

#include <vector>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include <core/storage/map_store.hpp>
#include <core/types/keyframe.hpp>
#include <stf/transform_tree.hpp>

namespace tracking {
namespace image {

// Helper structures for map keypoint triangulation
struct ObservationData {
    std::shared_ptr<core::types::KeyFrame> keyframe;
    cv::Point2f pixel;
    std::string camera_frame_id;
    Eigen::Matrix3d K;
    Eigen::Isometry3d camera_pose_in_odom;
};

class Reconstruct {
public:
    Reconstruct() {}

    bool triangulate(const core::types::KeyFrame& kf1, const core::types::KeyFrame& kf2,
                     const std::vector<cv::Point2f>& points1,
                     const std::vector<cv::Point2f>& points2, const stf::TransformTree& tft,
                     const Eigen::Isometry3d& essential_mat,
                     std::vector<Eigen::Vector3d>& triangulated_points_in_world,
                     bool use_essential_mat = false,
                     const std::string& base_link_frame_id = "base_link");

    // New API: Triangulate from map keypoint with multiple observations
    bool triangulateFromMapKeypoint(const core::types::Keypoint& map_keypoint,
                                   const core::storage::MapStore& map_store,
                                   const stf::TransformTree& tft,
                                   Eigen::Vector3d& triangulated_point_world,
                                   const std::string& base_link_frame_id = "base_link");

private:
    // Helper methods for map keypoint triangulation
    bool triangulateFromTwoObservations(const ObservationData& obs1, 
                                       const ObservationData& obs2,
                                       Eigen::Vector3d& triangulated_point_world);
    
    bool triangulateFromMultipleObservations(const std::vector<ObservationData>& observations,
                                            Eigen::Vector3d& triangulated_point_world);
};
}  // namespace image

}  // namespace tracking
