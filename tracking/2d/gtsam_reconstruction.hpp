#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include <core/storage/map_store.hpp>
#include <core/types/keyframe.hpp>
#include <core/types/keypoint.hpp>
#include <stf/transform_tree.hpp>
#include "triangulation_common.hpp"

namespace tracking {
namespace image {

class GtsamReconstruct {
public:
    GtsamReconstruct() {}

    bool triangulate(const core::types::KeyFrame& kf1, const core::types::KeyFrame& kf2,
                     const std::vector<cv::Point2f>& points1,
                     const std::vector<cv::Point2f>& points2, const stf::TransformTree& tft,
                     const Eigen::Isometry3d& essential_mat,
                     std::vector<Eigen::Vector3d>& triangulated_points_in_world,
                     bool use_essential_mat = false,
                     const std::string& base_link_frame_id = "base_link");

    // NEW API: Triangulate from map keypoint with multiple observations using GTSAM
    bool triangulateFromMapKeypoint(const core::types::Keypoint& map_keypoint,
                                   const core::storage::MapStore& map_store,
                                   const stf::TransformTree& tft,
                                   Eigen::Vector3d& triangulated_point_world,
                                   const std::string& base_link_frame_id = "base_link");

private:
    // Helper methods for map keypoint triangulation using GTSAM
    bool triangulateFromTwoObservationsGtsam(const ObservationData& obs1, 
                                            const ObservationData& obs2,
                                            Eigen::Vector3d& triangulated_point_world);
    
    bool triangulateFromMultipleObservationsGtsam(const std::vector<ObservationData>& observations,
                                                 Eigen::Vector3d& triangulated_point_world);
};
}  // namespace image

}  // namespace tracking
