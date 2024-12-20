#include "ros/rosbag_reader.hpp"
#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

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

        // Update config with new structure
        config_.odom_topic = "/base/odom";
        config_.color_topic = "/camera/camera/color/image_raw";
        config_.camera_info_topic = "/camera/camera/color/camera_info";
        config_.keyframe_distance_threshold = 0.1;  // 10cm default
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    std::string test_bag_path_ = std::string(getenv("TEST_SRCDIR")) + "/" +
                                 std::string(getenv("TEST_WORKSPACE")) +
                                 "/ros/test/data/test_sequence.db3";
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

    // Verify that keyframes were created
    const auto keyframes = store.getAllKeyFrames();
    EXPECT_GT(keyframes.size(), 0);

    // Verify that factors were created
    const auto factors = store.getAllFactors();
    EXPECT_GT(factors.size(), 0);

    // Verify keyframe spacing
    for (size_t i = 1; i < keyframes.size(); i++) {
        const auto& kf1 = keyframes[i - 1];
        const auto& kf2 = keyframes[i];

        double dx = kf2->pose.position.x() - kf1->pose.position.x();
        double dy = kf2->pose.position.y() - kf1->pose.position.y();
        double dz = kf2->pose.position.z() - kf1->pose.position.z();

        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        EXPECT_GE(distance, config_.keyframe_distance_threshold);
    }
}

TEST_F(RosbagReaderTest, InvalidBagFile) {
    ros::RosbagReader reader("/nonexistent/path.db3", *graph_, *store_, config_);
    EXPECT_FALSE(reader.initialize());
}

TEST_F(RosbagReaderTest, DifferentDistanceThresholds) {
    // Test with larger threshold
    config_.keyframe_distance_threshold = 0.2;  // 20cm
    ros::RosbagReader reader1(test_bag_path_, *graph_, *store_, config_);
    EXPECT_TRUE(reader1.initialize());
    EXPECT_TRUE(reader1.processBag());
    size_t keyframes_with_large_threshold = store_->getAllKeyFrames().size();

    // Reset store and graph
    store_ = std::make_unique<core::storage::MapStore>();
    graph_ = std::make_unique<core::graph::FactorGraph>();

    // Test with smaller threshold
    config_.keyframe_distance_threshold = 0.05;  // 5cm
    ros::RosbagReader reader2(test_bag_path_, *graph_, *store_, config_);
    EXPECT_TRUE(reader2.initialize());
    EXPECT_TRUE(reader2.processBag());
    size_t keyframes_with_small_threshold = store_->getAllKeyFrames().size();

    // We should get more keyframes with a smaller threshold
    EXPECT_GT(keyframes_with_small_threshold, keyframes_with_large_threshold);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
