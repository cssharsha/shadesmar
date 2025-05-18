#pragma once

#include <Eigen/Dense>           // For Eigen::Matrix3d, Eigen::Isometry3d
#include <opencv2/core/mat.hpp>  // For cv::Mat
#include <string>

// Forward declare core types to avoid full header includes if possible here
// However, for return types and direct usage, includes are often necessary.
#include "core/types/image.hpp"
#include "core/types/pose.hpp"
#include "viz/include/viz/rerun_viz.hpp"  // For rerun::archetypes::Pinhole

namespace kitti {
namespace conversions {

// Converts a KITTI 3x4 projection matrix (e.g., P_rect_xx) or a 3x4 Transform matrix
// to an Eigen::Isometry3d. For projection matrices, this typically means extracting
// the rotation and translation part if it represents a camera pose relative to another frame.
// For Tr_xxx_to_yyy which are rigid transforms (R|t), this is more direct.
Eigen::Isometry3d matrixToIsometry(const Eigen::Matrix<double, 3, 4>& mat);
Eigen::Isometry3d matrixToIsometry(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);

core::types::Pose toPose(const Eigen::Isometry3d& eigen_pose, double timestamp = 0.0,
                         std::string frame_id = "");

// Converts a cv::Mat image from KITTI to core::types::Image
core::types::Image toImage(const cv::Mat& cv_img, const std::string& frame_id = "");

// Converts a core::types::Image back to cv::Mat (useful for visualization or processing)
cv::Mat toCvMat(const core::types::Image& core_img);

// Converts KITTI GPS/IMU data (oxts) line to core::types::Pose
// This will involve parsing the raw string and converting geo-coordinates if necessary.
// The `first_oxts_data` can be used to establish a local ENU origin.
core::types::Pose toPoseFromOxts(const std::vector<double>& oxts_data,
                                 const std::vector<double>* first_oxts_data_ptr = nullptr);

// Converts KITTI Velodyne .bin file data to core::types::PointCloud
// The input `velo_data` would be a pointer to the raw float buffer and its size.
core::types::PointCloud toPointCloud(const float* velo_data, size_t num_points,
                                     const std::string& frame_id = "", double timestamp = 0.0);

// Converts KITTI calibration data (specifically a P_rect_xx projection matrix and image dimensions)
// to core::types::CameraInfo
core::types::CameraInfo toCameraInfo(const Eigen::Matrix<double, 3, 4>& P_rect,
                                     const std::string& frame_id = "");

// Converts core::types::CameraInfo to a Rerun Pinhole archetype for visualization
rerun::archetypes::Pinhole toRerunPinhole(const core::types::CameraInfo& cam_info);

}  // namespace conversions
}  // namespace kitti
