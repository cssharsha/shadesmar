#include "ros/conversions.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace ros {
namespace conversions {

core::types::Pose toPose(const geometry_msgs::msg::PoseStamped &pose_msg) {
  core::types::Pose pose;
  pose.position =
      Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y,
                      pose_msg.pose.position.z);
  pose.orientation = Eigen::Quaterniond(
      pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
      pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);
  pose.timestamp =
      pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9;
  return pose;
}

core::types::Pose toPose(const nav_msgs::msg::Odometry &odometry_msg) {
  core::types::Pose pose;
  pose.position = Eigen::Vector3d(odometry_msg.pose.pose.position.x,
                                  odometry_msg.pose.pose.position.y,
                                  odometry_msg.pose.pose.position.z);
  pose.orientation = Eigen::Quaterniond(odometry_msg.pose.pose.orientation.w,
                                        odometry_msg.pose.pose.orientation.x,
                                        odometry_msg.pose.pose.orientation.y,
                                        odometry_msg.pose.pose.orientation.z);
  pose.timestamp =
      odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nanosec * 1e-9;
  return pose;
}

geometry_msgs::msg::PoseStamped toPoseMsg(const core::types::Pose &pose) {
  geometry_msgs::msg::PoseStamped msg;
  msg.pose.position.x = pose.position.x();
  msg.pose.position.y = pose.position.y();
  msg.pose.position.z = pose.position.z();
  msg.pose.orientation.w = pose.orientation.w();
  msg.pose.orientation.x = pose.orientation.x();
  msg.pose.orientation.y = pose.orientation.y();
  msg.pose.orientation.z = pose.orientation.z();
  return msg;
}

cv::Mat toOpenCVImage(const sensor_msgs::msg::Image &image_msg) {
  // Determine OpenCV type based on ROS2 encoding
  int cv_type;
  if (image_msg.encoding == "mono8") {
    cv_type = CV_8UC1;
  } else if (image_msg.encoding == "bgr8") {
    cv_type = CV_8UC3;
  } else if (image_msg.encoding == "rgb8") {
    cv_type = CV_8UC3;
  } else if (image_msg.encoding == "32FC1") {
    cv_type = CV_32FC1;
  } else {
    throw std::runtime_error("Unsupported image encoding: " +
                             image_msg.encoding);
  }

  // Create OpenCV Mat with correct size and type
  cv::Mat image(image_msg.height, image_msg.width, cv_type);

  // Copy data
  memcpy(image.data, image_msg.data.data(), image_msg.data.size());

  // Convert RGB to BGR if necessary
  if (image_msg.encoding == "rgb8") {
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
  }

  return image;
}

core::types::Image toImage(const sensor_msgs::msg::Image &image_msg) {
  cv::Mat cv_image = toOpenCVImage(image_msg);
  return core::types::Image::fromCvMat(cv_image, image_msg.encoding);
}

sensor_msgs::msg::Image toImageMsg(const cv::Mat &image) {
  sensor_msgs::msg::Image msg;

  // Set image metadata
  msg.height = image.rows;
  msg.width = image.cols;
  msg.step = image.cols * image.elemSize();

  // Set encoding based on image type
  switch (image.type()) {
  case CV_8UC1:
    msg.encoding = "mono8";
    break;
  case CV_8UC3:
    msg.encoding = "bgr8";
    break;
  case CV_32FC1:
    msg.encoding = "32FC1";
    break;
  default:
    throw std::runtime_error("Unsupported OpenCV image type");
  }

  // Copy image data
  size_t size = msg.step * msg.height;
  msg.data.resize(size);
  memcpy(msg.data.data(), image.data, size);

  return msg;
}

core::proto::CameraInfo
toCameraInfo(const sensor_msgs::msg::CameraInfo &camera_info_msg) {
  core::proto::CameraInfo info;
  info.set_width(camera_info_msg.width);
  info.set_height(camera_info_msg.height);
  info.set_distortion_model(camera_info_msg.distortion_model);

  // Set intrinsic matrix K (3x3)
  for (size_t i = 0; i < 9; ++i) {
    info.add_k(camera_info_msg.k[i]);
  }

  // Set distortion parameters
  for (const auto &d : camera_info_msg.d) {
    info.add_d(d);
  }

  return info;
}

core::types::PointCloud
toPointCloud(const sensor_msgs::msg::Image &depth_msg,
             const sensor_msgs::msg::CameraInfo &camera_info_msg) {
  cv::Mat depth = toImage(depth_msg).toCvMat();
  core::types::PointCloud cloud;

  // Get camera intrinsics from K matrix
  float fx = camera_info_msg.k[0];
  float fy = camera_info_msg.k[4];
  float cx = camera_info_msg.k[2];
  float cy = camera_info_msg.k[5];

  for (int v = 0; v < depth.rows; v++) {
    for (int u = 0; u < depth.cols; u++) {
      float z = depth.at<float>(v, u);
      if (z > 0) {
        float x = (u - cx) * z / fx;
        float y = (v - cy) * z / fy;
        cloud.points.emplace_back(x, y, z);
        cloud.colors.emplace_back(1.0, 1.0, 1.0); // Default white color
      }
    }
  }

  return cloud;
}

} // namespace conversions
} // namespace ros