#include <gtest/gtest.h>
#include "ros/rosbag_reader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <filesystem>

class RosbagReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        graph_ = std::make_unique<core::graph::FactorGraph>();
        store_ = std::make_unique<core::storage::MapStore>();
        createTestBag();
        config_.odom_topic = "/odom";
        config_.color_topic = "/camera/color/image_raw";
        config_.depth_topic = "/camera/depth/image_rect_raw";
        config_.camera_info_topic = "/camera/color/camera_info";
    }

    void TearDown() override {
        if (std::filesystem::exists(test_bag_path_)) {
            std::filesystem::remove(test_bag_path_);
        }
        rclcpp::shutdown();
    }

    void createTestBag() {
        rosbag2_cpp::Writer writer;
        writer.open(test_bag_path_);

        rclcpp::Time time(0);
        for (int i = 0; i < 10; ++i) {
            time += rclcpp::Duration::from_seconds(0.1);

            // Add odometry
            auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
            odom_msg->header.stamp = time;
            odom_msg->header.frame_id = "odom";
            odom_msg->child_frame_id = "base_link";
            odom_msg->pose.pose.position.x = i * 0.1;
            odom_msg->pose.pose.orientation.w = 1.0;
            writer.write(odom_msg, config_.odom_topic, time);

            // Add images and camera info
            auto color_msg = std::make_shared<sensor_msgs::msg::Image>();
            color_msg->header.stamp = time;
            color_msg->height = 480;
            color_msg->width = 640;
            color_msg->encoding = "rgb8";
            color_msg->data.resize(640 * 480 * 3, i);
            writer.write(color_msg, config_.color_topic, time);

            auto depth_msg = std::make_shared<sensor_msgs::msg::Image>();
            depth_msg->header.stamp = time;
            depth_msg->height = 480;
            depth_msg->width = 640;
            depth_msg->encoding = "16UC1";
            depth_msg->data.resize(640 * 480 * 2, i);
            writer.write(depth_msg, config_.depth_topic, time);

            auto camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>();
            camera_info->height = 480;
            camera_info->width = 640;
            camera_info->k = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
            writer.write(camera_info, config_.camera_info_topic, time);
        }
    }

    std::string test_bag_path_ = "/tmp/test_ros2.db3";
    ros::Config config_;
    std::unique_ptr<core::graph::FactorGraph> graph_;
    std::unique_ptr<core::storage::MapStore> store_;
};

TEST_F(RosbagReaderTest, Initialize) {
    ros::RosbagReader reader(test_bag_path_, *graph_, *store_, config_);
    EXPECT_TRUE(reader.initialize());
}

TEST_F(RosbagReaderTest, ProcessEntireBag) {
    ros::RosbagReader reader(test_bag_path_, *graph_, *store_, config_);
    EXPECT_TRUE(reader.initialize());
    EXPECT_TRUE(reader.processBag());

    const auto& store = *store_;
    EXPECT_GT(store.getAllKeyFrames().size(), 0);
    EXPECT_GT(store.getAllFactors().size(), 0);
}

TEST_F(RosbagReaderTest, InvalidBagFile) {
    ros::RosbagReader reader("/nonexistent/path.db3", *graph_, *store_, config_);
    EXPECT_FALSE(reader.initialize());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}