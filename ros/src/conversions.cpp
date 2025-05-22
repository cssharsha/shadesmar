#include "ros/conversions.hpp"
#include <cstdint>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <logging/logging.hpp>
#include <ostream>

namespace ros {
namespace conversions {

core::types::Pose toPose(const geometry_msgs::msg::PoseStamped& pose_msg) {
    core::types::Pose pose;
    pose.position = Eigen::Vector3d(pose_msg.pose.position.x, pose_msg.pose.position.y,
                                    pose_msg.pose.position.z);
    pose.orientation = Eigen::Quaterniond(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x,
                                          pose_msg.pose.orientation.y, pose_msg.pose.orientation.z);
    pose.timestamp = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9;
    pose.frame_id = pose_msg.header.frame_id;
    return pose;
}

core::types::Pose toPose(const nav_msgs::msg::Odometry& odometry_msg) {
    core::types::Pose pose;
    pose.position =
        Eigen::Vector3d(odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y,
                        odometry_msg.pose.pose.position.z);
    pose.orientation = Eigen::Quaterniond(
        odometry_msg.pose.pose.orientation.w, odometry_msg.pose.pose.orientation.x,
        odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z);
    pose.timestamp = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nanosec * 1e-9;
    pose.frame_id = odometry_msg.header.frame_id;
    return pose;
}

geometry_msgs::msg::PoseStamped toPoseMsg(const core::types::Pose& pose) {
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = pose.position.x();
    msg.pose.position.y = pose.position.y();
    msg.pose.position.z = pose.position.z();
    msg.pose.orientation.w = pose.orientation.w();
    msg.pose.orientation.x = pose.orientation.x();
    msg.pose.orientation.y = pose.orientation.y();
    msg.pose.orientation.z = pose.orientation.z();
    msg.header.frame_id = pose.frame_id;
    return msg;
}

cv::Mat toOpenCVImage(const sensor_msgs::msg::Image& image_msg) {
    // Determine OpenCV type based on ROS2 encoding
    int cv_type;
    std::cout << "Current encoding: " << image_msg.encoding << std::endl;
    if (image_msg.encoding == "mono8") {
        cv_type = CV_8UC1;
    } else if (image_msg.encoding == "bgr8") {
        cv_type = CV_8UC3;
    } else if (image_msg.encoding == "rgb8") {
        cv_type = CV_8UC3;
    } else if (image_msg.encoding == "32FC1") {
        cv_type = CV_32FC1;
    } else {
        throw std::runtime_error("Unsupported image encoding: " + image_msg.encoding);
    }

    // Create OpenCV Mat with correct size and type
    cv::Mat image(image_msg.height, image_msg.width, cv_type);

    // Copy data
    memcpy(image.data, image_msg.data.data(), image_msg.data.size());

    // cv::Mat cp_image = image.clone();

    // Convert RGB to BGR if necessary
    // if (image_msg.encoding == "rgb8") {
    //     cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    // }

    return image;
}

core::types::Image toImage(const sensor_msgs::msg::Image& image_msg) {
    cv::Mat cv_image = toOpenCVImage(image_msg);
    auto image = core::types::Image::fromCvMat(cv_image, image_msg.encoding);
    image.frame_id = image_msg.header.frame_id;
    return image;
}

sensor_msgs::msg::Image toImageMsg(const cv::Mat& image) {
    sensor_msgs::msg::Image msg;

    // Set image metadata
    msg.height = image.rows;
    msg.width = image.cols;
    msg.step = image.cols * image.elemSize();
    msg.header.frame_id = "default_camera";

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

core::proto::CameraInfo toCameraInfoProto(const sensor_msgs::msg::CameraInfo& camera_info_msg) {
    core::proto::CameraInfo info;
    info.set_width(camera_info_msg.width);
    info.set_height(camera_info_msg.height);
    info.set_distortion_model(camera_info_msg.distortion_model);
    info.set_frame_id(camera_info_msg.header.frame_id);

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

template <typename T>
void log_vector_as_3x3_matrix_glog(const T& vec) {
    std::ostringstream oss;

    // Optional: Set precision and format for double values
    oss << std::fixed << std::setprecision(4);  // Example: 4 decimal places, fixed notation

    if (vec.size() == 9) {
        // Format as a 3x3 matrix structure within the single line
        oss << " (3x3 row-major): [";
        for (int r = 0; r < 3; ++r) {
            oss << "[";  // Start of a row
            for (int c = 0; c < 3; ++c) {
                oss << vec[r * 3 + c];
                if (c < 2) {  // If not the last element in the current row
                    oss << ", ";
                }
            }
            oss << "]";       // End of a row
            if (r < 2) {      // If not the last row
                oss << ", ";  // Separator between rows
            }
        }
        oss << "]";  // End of the matrix structure
    } else {
        // Fallback for vectors not of size 9: log as a simple flat list
        oss << " (size " << vec.size() << ", not 3x3): [";
        if (!vec.empty()) {
            for (size_t i = 0; i < vec.size(); ++i) {
                oss << vec[i];
                if (i < vec.size() - 1) {
                    oss << ", ";
                }
            }
        }
        oss << "]";
    }

    LOG(INFO) << oss.str();  // Log the entire formatted string
}

core::types::CameraInfo toCameraInfo(const sensor_msgs::msg::CameraInfo& camera_info_msg) {
    core::types::CameraInfo info;
    info.width = camera_info_msg.width;
    info.height = camera_info_msg.height;
    info.distortion_model = camera_info_msg.distortion_model;
    info.frame_id = camera_info_msg.header.frame_id;

    // Copy intrinsic matrix K (3x3)
    info.k = std::vector<double>(camera_info_msg.k.begin(), camera_info_msg.k.end());

    LOG(INFO) << "Converted K: \n";
    log_vector_as_3x3_matrix_glog(info.k);

    LOG(INFO) << "Incoming camera message: \n";
    log_vector_as_3x3_matrix_glog(camera_info_msg.k);

    // Copy distortion parameters
    info.d = std::vector<double>(camera_info_msg.d.begin(), camera_info_msg.d.end());

    return info;
}

core::types::PointCloud toPointCloud(const sensor_msgs::msg::Image& depth_msg,
                                     const sensor_msgs::msg::CameraInfo& camera_info_msg) {
    cv::Mat depth = toImage(depth_msg).toCvMat();
    core::types::PointCloud cloud;
    cloud.frame_id = depth_msg.header.frame_id;

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

// IMU conversions
core::types::ImuData toImuData(const sensor_msgs::msg::Imu& imu_msg) {
    core::types::ImuData imu_data;

    // Convert linear acceleration
    imu_data.linear_acceleration = Eigen::Vector3d(
        imu_msg.linear_acceleration.x,
        imu_msg.linear_acceleration.y,
        imu_msg.linear_acceleration.z);

    // Convert linear acceleration covariance (3x3 from row-major 9 elements)
    if (imu_msg.linear_acceleration_covariance[0] >= 0) {  // Valid covariance
        imu_data.linear_acceleration_covariance = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            imu_msg.linear_acceleration_covariance.data());
    } else {
        imu_data.linear_acceleration_covariance = Eigen::Matrix3d::Identity() * 0.01;  // Default 0.01 m/s² std
    }

    // Convert angular velocity
    imu_data.angular_velocity = Eigen::Vector3d(
        imu_msg.angular_velocity.x,
        imu_msg.angular_velocity.y,
        imu_msg.angular_velocity.z);

    // Convert angular velocity covariance (3x3 from row-major 9 elements)
    if (imu_msg.angular_velocity_covariance[0] >= 0) {  // Valid covariance
        imu_data.angular_velocity_covariance = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            imu_msg.angular_velocity_covariance.data());
    } else {
        imu_data.angular_velocity_covariance = Eigen::Matrix3d::Identity() * 0.001;  // Default 0.001 rad/s std
    }

    // Convert orientation
    imu_data.orientation = Eigen::Quaterniond(
        imu_msg.orientation.w,
        imu_msg.orientation.x,
        imu_msg.orientation.y,
        imu_msg.orientation.z);

    // Convert orientation covariance (3x3 from row-major 9 elements)
    if (imu_msg.orientation_covariance[0] >= 0) {  // Valid covariance
        imu_data.orientation_covariance = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            imu_msg.orientation_covariance.data());
    } else {
        imu_data.orientation_covariance = Eigen::Matrix3d::Identity() * 0.01;  // Default 0.01 rad std
    }

    // Convert metadata
    imu_data.timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;
    imu_data.frame_id = imu_msg.header.frame_id;
    // Note: sensor_msgs::Imu doesn't have sequence field in ROS2

    return imu_data;
}

sensor_msgs::msg::Imu toImuMsg(const core::types::ImuData& imu_data) {
    sensor_msgs::msg::Imu imu_msg;

    // Convert linear acceleration
    imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x();
    imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y();
    imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z();

    // Convert linear acceleration covariance (3x3 to row-major 9 elements)
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        imu_msg.linear_acceleration_covariance.data()) = imu_data.linear_acceleration_covariance;

    // Convert angular velocity
    imu_msg.angular_velocity.x = imu_data.angular_velocity.x();
    imu_msg.angular_velocity.y = imu_data.angular_velocity.y();
    imu_msg.angular_velocity.z = imu_data.angular_velocity.z();

    // Convert angular velocity covariance (3x3 to row-major 9 elements)
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        imu_msg.angular_velocity_covariance.data()) = imu_data.angular_velocity_covariance;

    // Convert orientation
    imu_msg.orientation.w = imu_data.orientation.w();
    imu_msg.orientation.x = imu_data.orientation.x();
    imu_msg.orientation.y = imu_data.orientation.y();
    imu_msg.orientation.z = imu_data.orientation.z();

    // Convert orientation covariance (3x3 to row-major 9 elements)
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        imu_msg.orientation_covariance.data()) = imu_data.orientation_covariance;

    // Convert metadata
    int64_t sec = static_cast<int64_t>(imu_data.timestamp);
    uint32_t nanosec = static_cast<uint32_t>((imu_data.timestamp - sec) * 1e9);
    imu_msg.header.stamp.sec = sec;
    imu_msg.header.stamp.nanosec = nanosec;
    imu_msg.header.frame_id = imu_data.frame_id;

    return imu_msg;
}

}  // namespace conversions
}  // namespace ros
