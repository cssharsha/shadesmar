#pragma once

#include <core/types/keyframe.hpp>
#include <core/types/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ros {
namespace conversions {

// Pose conversions
core::types::Pose toPose(const geometry_msgs::msg::PoseStamped& pose_msg);
core::types::Pose toPose(const nav_msgs::msg::Odometry& odom_msg);
geometry_msgs::msg::PoseStamped toPoseMsg(const core::types::Pose& pose);

// Image conversions
cv::Mat toOpenCVImage(const sensor_msgs::msg::Image& image_msg);
core::types::Image toImage(const sensor_msgs::msg::Image& image_msg);
sensor_msgs::msg::Image toImageMsg(const cv::Mat& image);

// Camera info conversions
core::proto::CameraInfo toCameraInfo(const sensor_msgs::msg::CameraInfo& camera_info_msg);
sensor_msgs::msg::CameraInfo toCameraInfoMsg(const core::proto::CameraInfo& camera_info);

// Point cloud conversions from depth image
core::types::PointCloud toPointCloud(const sensor_msgs::msg::Image& depth_msg,
                                     const sensor_msgs::msg::CameraInfo& camera_info_msg);
sensor_msgs::msg::PointCloud2 toPointCloud2Msg(const core::types::PointCloud& cloud);

}  // namespace conversions
}  // namespace ros
