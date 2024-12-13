#include <gtest/gtest.h>
#include "ros/rosbag_reader.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <filesystem>
#include <cstdlib>  // For getenv and setenv

#include <nav_msgs/msg/odometry.hpp>
class RosbagReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set the ROS logging directory to the test temporary directory
        const char* test_tmpdir = std::getenv("TEST_TMPDIR");
        if (test_tmpdir) {
            setenv("ROS_LOG_DIR", test_tmpdir, 1);
            setenv("ROS_HOME", test_tmpdir, 1);
        }

        rclcpp::InitOptions init_options;
        rclcpp::init(0, nullptr, init_options);

        graph_ = std::make_unique<core::graph::FactorGraph>();
        store_ = std::make_unique<core::storage::MapStore>();
        config_.odom_topic = "/odom";
        config_.color_topic = "/camera/color/image_raw";
        config_.depth_topic = "/camera/depth/image_rect_raw";
        config_.camera_info_topic = "/camera/color/camera_info";
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::string test_bag_path_ = "/mnt/remote-storage/all_topics_2024_12_05_19_23_52";
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