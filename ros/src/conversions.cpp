#include "ros/conversions.hpp"
#include <cv_bridge/cv_bridge.h>

namespace ros {
namespace conversions {

core::types::Pose toPose(const geometry_msgs::msg::PoseStamped &pose_msg) {
    core::types::Pose pose;
    pose.position = Eigen::Vector3d(
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z);
    pose.orientation = Eigen::Quaterniond(
        pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z);
    pose.timestamp = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9;
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

cv::Mat toImage(const sensor_msgs::msg::Image &image_msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(std::make_shared<sensor_msgs::msg::Image>(image_msg));
        return cv_ptr->image.clone();
    } catch (cv_bridge::Exception& e) {
        throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
    }
}

sensor_msgs::msg::Image toImageMsg(const cv::Mat &image) {
    cv_bridge::CvImage cv_image;
    cv_image.encoding = image.channels() == 1 ? "mono8" : "bgr8";
    cv_image.image = image;
    return *cv_image.toImageMsg();
}

core::proto::CameraInfo toCameraInfo(const sensor_msgs::msg::CameraInfo &camera_info_msg) {
    core::proto::CameraInfo info;
    info.set_width(camera_info_msg.width);
    info.set_height(camera_info_msg.height);

    // Set intrinsic matrix K (3x3)
    for (size_t i = 0; i < 9; ++i) {
        info.add_k(camera_info_msg.k[i]);
    }

    // Set distortion parameters
    for (const auto& d : camera_info_msg.d) {
        info.add_d(d);
    }

    return info;
}

core::types::PointCloud toPointCloud(const sensor_msgs::msg::Image &depth_msg,
                                   const sensor_msgs::msg::CameraInfo &camera_info_msg) {
    cv::Mat depth = toImage(depth_msg);
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
                cloud.colors.emplace_back(1.0, 1.0, 1.0);  // Default white color
            }
        }
    }

    return cloud;
}

} // namespace conversions
} // namespace ros