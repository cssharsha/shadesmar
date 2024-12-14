#include <gtest/gtest.h>
#include "ros/conversions.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class ConversionsTest : public ::testing::Test {
protected:
    geometry_msgs::msg::PoseStamped createTestPoseMsg() {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = rclcpp::Time(1234, 0);
        pose_msg.pose.position.x = 1.0;
        pose_msg.pose.position.y = 2.0;
        pose_msg.pose.position.z = 3.0;
        pose_msg.pose.orientation.w = 1.0;
        return pose_msg;
    }

    sensor_msgs::msg::CameraInfo createTestCameraInfo() {
        sensor_msgs::msg::CameraInfo info_msg;
        info_msg.height = 480;
        info_msg.width = 640;
        info_msg.distortion_model = "plumb_bob";

        // Set camera matrix (K)
        info_msg.k = {500.0, 0.0, 320.0,
                     0.0, 500.0, 240.0,
                     0.0, 0.0, 1.0};

        // Set distortion coefficients
        info_msg.d = {0.1, 0.2, 0.3, 0.4, 0.5};

        return info_msg;
    }
};

TEST_F(ConversionsTest, PoseConversion) {
    auto pose_msg = createTestPoseMsg();
    auto pose = ros::conversions::toPose(pose_msg);

    // Check position
    EXPECT_NEAR(pose.position.x(), 1.0, 1e-6);
    EXPECT_NEAR(pose.position.y(), 2.0, 1e-6);
    EXPECT_NEAR(pose.position.z(), 3.0, 1e-6);

    // Check orientation
    EXPECT_NEAR(pose.orientation.w(), 1.0, 1e-6);
    EXPECT_NEAR(pose.orientation.x(), 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.y(), 0.0, 1e-6);
    EXPECT_NEAR(pose.orientation.z(), 0.0, 1e-6);

    // Check timestamp
    EXPECT_NEAR(pose.timestamp, 1234, 1e-6);
}

TEST_F(ConversionsTest, PoseRoundTrip) {
    auto original_msg = createTestPoseMsg();
    auto pose = ros::conversions::toPose(original_msg);
    auto converted_msg = ros::conversions::toPoseMsg(pose);

    EXPECT_NEAR(original_msg.pose.position.x, converted_msg.pose.position.x, 1e-6);
    EXPECT_NEAR(original_msg.pose.position.y, converted_msg.pose.position.y, 1e-6);
    EXPECT_NEAR(original_msg.pose.position.z, converted_msg.pose.position.z, 1e-6);

    EXPECT_NEAR(original_msg.pose.orientation.w, converted_msg.pose.orientation.w, 1e-6);
    EXPECT_NEAR(original_msg.pose.orientation.x, converted_msg.pose.orientation.x, 1e-6);
    EXPECT_NEAR(original_msg.pose.orientation.y, converted_msg.pose.orientation.y, 1e-6);
    EXPECT_NEAR(original_msg.pose.orientation.z, converted_msg.pose.orientation.z, 1e-6);
}

TEST_F(ConversionsTest, CameraInfoConversion) {
    auto camera_info_msg = createTestCameraInfo();
    auto camera_info = ros::conversions::toCameraInfo(camera_info_msg);

    EXPECT_EQ(camera_info.width(), camera_info_msg.width);
    EXPECT_EQ(camera_info.height(), camera_info_msg.height);
    EXPECT_EQ(camera_info.distortion_model(), camera_info_msg.distortion_model);

    // Check K matrix
    for (size_t i = 0; i < 9; ++i) {
        EXPECT_NEAR(camera_info.k(i), camera_info_msg.k[i], 1e-6);
    }

    // Check distortion coefficients
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_NEAR(camera_info.d(i), camera_info_msg.d[i], 1e-6);
    }
}

TEST_F(ConversionsTest, PointCloudConversion) {
    // Create test depth image
    sensor_msgs::msg::Image depth_msg;
    depth_msg.height = 4;
    depth_msg.width = 4;
    depth_msg.encoding = "32FC1";
    depth_msg.step = depth_msg.width * sizeof(float);
    depth_msg.data.resize(depth_msg.height * depth_msg.width * sizeof(float));

    float* depth_data = reinterpret_cast<float*>(depth_msg.data.data());
    // Set some test depth values (in meters)
    for (size_t i = 0; i < 16; ++i) {
        depth_data[i] = 1.0;  // 1 meter depth
    }

    auto camera_info_msg = createTestCameraInfo();

    // Convert to point cloud
    auto point_cloud = ros::conversions::toPointCloud(depth_msg, camera_info_msg);

    // Verify point cloud properties
    EXPECT_EQ(point_cloud.points.size(), 16);  // All pixels should be converted

    // Check a few sample points
    // For center pixel (2,2) with 1m depth
    float fx = camera_info_msg.k[0];
    float fy = camera_info_msg.k[4];
    float cx = camera_info_msg.k[2];
    float cy = camera_info_msg.k[5];

    // Calculate expected 3D position for center point
    int u = 2, v = 2;
    float z = 1.0;
    float expected_x = (u - cx) * z / fx;
    float expected_y = (v - cy) * z / fy;
    float expected_z = z;

    // Find and verify the point
    bool found_point = false;
    for (const auto& point : point_cloud.points) {
        if (std::abs(point.x() - expected_x) < 1e-6 &&
            std::abs(point.y() - expected_y) < 1e-6 &&
            std::abs(point.z() - expected_z) < 1e-6) {
            found_point = true;
            break;
        }
    }
    EXPECT_TRUE(found_point);
}