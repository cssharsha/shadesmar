#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include <core/storage/map_store.hpp>
#include <core/types/keyframe.hpp>
#include <stf/transform_tree.hpp>

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
};
}  // namespace image

}  // namespace tracking
